/*
 * SafeLifts Wristband v3.4 — BLE + watchdog + OLED display + haptics
 *
 * Board: Seeed XIAO nRF52840 Sense
 * Libraries (Library Manager): Adafruit SSD1306, Adafruit GFX Library,
 * Adafruit DRV2605 Library
 * (Bluefruit and TinyUSB come with the Seeed board package.)
 *
 * Role: GATT PERIPHERAL / SERVER. The hub writes 6-byte hazard packets
 * into the Hazard State characteristic. We receive, show, and watch for
 * staleness.
 *
 * Changes in v3.4 (from v3.3):
 *  - OLED presence check: the SSD1306 library's begin() does NOT verify the
 *    display answers, so v3.3 could report "OLED init OK" with the display
 *    unplugged. setup() now probes the bus first, and prints an I2C scan
 *    (expect 0x3C = OLED, 0x5A = DRV2605L).
 *  - Watchdog elapsed-time calc can no longer underflow if a packet lands
 *    between the two reads (that caused a rare one-off false LOST SIGNAL).
 *  - The per-packet RX debug line is now gated by DEBUG_STATUS.
 *
 * Changes in v3.3 (from v3.2):
 *  - A sensor-fault flag with no hazard now shows a SENSOR FAULT headline
 *    ("CLEAR NOT CONFIRMED") instead of a big CLEAR, plus its own slow
 *    triple-click haptic. Real hazards still take priority over it.
 *  - WATCHDOG_TIMEOUT_MS set to 400 and TEST_MUTE_LOST_HAPTIC set to false
 *    (real-use values, now that the hub streams every 100 ms)
 *  - RX debug line also prints flags
 *
 * Changes since the BLE-only baseline:
 *  - OLED screens: Searching, Waiting, Lost Signal, Hazard, Clear
 *  - Display only redraws when content changes (a full I2C frame takes
 *    ~25 ms, so we never redraw every loop pass)
 *  - loop() reads currentHazard through a version counter so it can't
 *    see a half-updated packet
 *  - Haptics: DRV2605L in LRA mode, auto-calibrated on every boot,
 *    driven in Real-Time Playback with non-blocking millis() patterns
 *  - Display and haptic init happen AFTER BLE is advertising, so a dead
 *    peripheral can never stop the wristband from connecting
 */

#include <bluefruit.h>

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h> // for Serial
#endif

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_DRV2605.h>

// ---------------------------------------------------------------------
// UUIDs (stored least-significant byte first, as Bluefruit expects)
//   Service: f20270e0-4d91-4592-8d79-61117906f549
//   Hazard:  095d4f3c-7f73-447f-bd75-97a0e4712903
//   Config:  9d2f5683-d54c-43cf-9a2e-14b7490f3e27
// ---------------------------------------------------------------------
const uint8_t HAZARD_SERVICE_UUID[16] =
{
  0x49, 0xF5, 0x06, 0x79, 0x11, 0x61, 0x79, 0x8D,
  0x92, 0x45, 0x91, 0x4D, 0xE0, 0x70, 0x02, 0xF2
};

const uint8_t HAZARD_CHAR_UUID[16] =
{
  0x03, 0x29, 0x71, 0xE4, 0xA0, 0x97, 0x75, 0xBD,
  0x7F, 0x44, 0x73, 0x7F, 0x3C, 0x4F, 0x5D, 0x09
};

// Reserved for the future Config characteristic. Not used yet.
const uint8_t CONFIG_CHAR_UUID[16] =
{
  0x27, 0x3E, 0x0F, 0x49, 0xB7, 0x14, 0x2E, 0x9A,
  0xCF, 0x43, 0x4C, 0xD5, 0x83, 0x56, 0x2F, 0x9D
};

BLEService        hazardService(HAZARD_SERVICE_UUID);
BLECharacteristic hazardChar(HAZARD_CHAR_UUID);

// ---------------------------------------------------------------------
// Packet layout (6 bytes)
// ---------------------------------------------------------------------
struct HazardPacket
{
  uint8_t seq;
  uint8_t sector;       // 0-7, 0xFF = clear
  uint8_t distance_cm;  // 255 = out of range / unknown
  uint8_t urgency;      // 0=clear, 1=caution, 2=warning, 3=danger
  uint8_t flags;        // bit0 = multi-object, bit1 = hub sensor fault
  uint8_t reserved;
};

enum Sector : uint8_t
{
  SECTOR_FRONT = 0, SECTOR_FRONT_RIGHT, SECTOR_RIGHT, SECTOR_BACK_RIGHT,
  SECTOR_BACK, SECTOR_BACK_LEFT, SECTOR_LEFT, SECTOR_FRONT_LEFT,
  SECTOR_CLEAR = 0xFF
};

enum Urgency : uint8_t
{
  URGENCY_CLEAR = 0, URGENCY_CAUTION, URGENCY_WARNING, URGENCY_DANGER
};

const uint8_t FLAG_MULTI_OBJECT  = 0x01;
const uint8_t FLAG_SENSOR_FAULT  = 0x02;

// ---------------------------------------------------------------------
// Display setup
// ---------------------------------------------------------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS  0x3C   // confirmed by the I2C scanner

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool displayOK = false;

enum ScreenId : uint8_t
{
  SCREEN_NONE, SCREEN_SEARCHING, SCREEN_WAITING, SCREEN_LOST, SCREEN_HAZARD
};
ScreenId     lastScreen = SCREEN_NONE;   // what is currently on the panel
HazardPacket lastDrawn  = { 0, 0, 0, 0, 0, 0 };

const char* const SECTOR_NAMES[8] =
{
  "FRONT", "FRONT-RIGHT", "RIGHT", "BACK-RIGHT",
  "BACK", "BACK-LEFT", "LEFT", "FRONT-LEFT"
};

// ---------------------------------------------------------------------
// Haptics: pattern tables and state
// Each pattern is a looping list of steps: {duration_ms, amplitude}.
// Amplitude is 0-127 (RTP signed mode); 0 = motor off.
// These numbers are a first guess. Tune them by feel on a wrist.
// ---------------------------------------------------------------------
Adafruit_DRV2605 drv;
bool hapticOK = false;

#define HAP_REG_GO      0x0C   // GO bit (bit0) clears when auto-cal finishes
#define HAP_REG_STATUS  0x00   // bit3 = DIAG_RESULT (1 = auto-cal failed)

enum HapticPattern : uint8_t
{
  HAP_NONE, HAP_CAUTION, HAP_WARNING, HAP_DANGER, HAP_LOST, HAP_FAULT
};

struct HapticStep { uint16_t ms; uint8_t amp; };

const HapticStep PAT_CAUTION[] = { {  80,  80 }, { 1120, 0 } };                          // 1 click / 1.2 s
const HapticStep PAT_WARNING[] = { {  70, 100 }, {   90, 0 }, {  70, 100 }, { 370, 0 } }; // double click / 0.6 s
const HapticStep PAT_DANGER[]  = { { 120, 127 }, {  130, 0 } };                          // rapid pulses / 0.25 s
const HapticStep PAT_LOST[]    = { { 700, 110 }, { 1300, 0 } };                          // long buzz / 2 s
const HapticStep PAT_FAULT[]   = { {  50,  80 }, {   90, 0 }, {  50,  80 }, {   90, 0 }, {  50, 80 }, { 2600, 0 } }; // slow triple click / 2.9 s (sensor fault, no hazard)

struct PatternDef { const HapticStep* steps; uint8_t count; };

const PatternDef PATTERNS[] =
{
  { nullptr,     0 },                                                 // HAP_NONE
  { PAT_CAUTION, sizeof(PAT_CAUTION) / sizeof(PAT_CAUTION[0]) },
  { PAT_WARNING, sizeof(PAT_WARNING) / sizeof(PAT_WARNING[0]) },
  { PAT_DANGER,  sizeof(PAT_DANGER)  / sizeof(PAT_DANGER[0])  },
  { PAT_LOST,    sizeof(PAT_LOST)    / sizeof(PAT_LOST[0])    },
  { PAT_FAULT,   sizeof(PAT_FAULT)   / sizeof(PAT_FAULT[0])   }   // HAP_FAULT
};

HapticPattern hapPattern   = HAP_NONE;
uint8_t       hapStep      = 0;
uint32_t      hapStepStart = 0;
uint8_t       hapLastAmp   = 0;   // last value written, avoids redundant I2C writes

// ---------------------------------------------------------------------
// Watchdog / link-state machine
// ---------------------------------------------------------------------
enum LinkState
{
  STATE_ADVERTISING,    // not connected, waiting for the hub
  STATE_WAITING_FIRST,  // connected, no packet yet on this connection
  STATE_OK,             // connected AND receiving fresh packets
  STATE_LOST_SIGNAL     // stale, never-started, or disconnected
};

const uint32_t WATCHDOG_TIMEOUT_MS     = 400;     // between packets, once data is flowing (hub writes every 100 ms)
const uint32_t FIRST_PACKET_TIMEOUT_MS = 3000;  // grace for hub to discover services and start writing

// Debug heartbeat: prints a STATUS line every 2 s so you can read the state
// any time the Serial Monitor is open, without catching the boot messages.
// state: 0=advertising 1=waiting-first 2=ok 3=lost-signal
// Set to false when you no longer need it.
const bool DEBUG_STATUS = true;

// TEST ONLY: silences the repeating lost-signal buzz so you can bench-test with
// nRF Connect (a single write can't keep the link alive, so the watchdog fires
// and the lost-signal buzz would otherwise run forever). The screen still goes
// to LOST SIGNAL. MUST be false for real use: the buzz is part of the fail-safe.
const bool TEST_MUTE_LOST_HAPTIC = false;

volatile LinkState currentState = STATE_ADVERTISING;
volatile uint8_t   lastSeq = 0;
volatile uint32_t  lastPacketMs = 0;

HazardPacket currentHazard = { 0, SECTOR_CLEAR, 255, URGENCY_CLEAR, 0, 0 };

// Version counter guarding currentHazard: odd = write in progress.
volatile uint32_t hazardVersion = 0;

// Copy currentHazard without ever returning a half-written packet.
void readHazard(HazardPacket& out)
{
  for (int tries = 0; tries < 5; tries++)
  {
    uint32_t v1 = hazardVersion;
    __sync_synchronize();
    out = currentHazard;
    __sync_synchronize();
    uint32_t v2 = hazardVersion;
    if (v1 == v2 && (v1 & 1) == 0) return;
  }
  // Gave up after 5 tries: 'out' holds the last attempt. The next loop
  // pass (20 ms later) reads again, so this is harmless.
}

// ---------------------------------------------------------------------
// BLE callbacks
// ---------------------------------------------------------------------

// Fires when the hub connects to us.
void connect_callback(uint16_t conn_handle)
{
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));

  Serial.print("Connected to: ");
  Serial.println(peer_name);

  // Connected is not the same as healthy. Start the clock now so a hub
  // that connects but never writes still times out into fail-safe.
  currentState = STATE_WAITING_FIRST;
  lastPacketMs = millis();
  Serial.println("State: WAITING_FIRST");
}

// Fires on ANY disconnect — the fast path to fail-safe.
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;

  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);

  currentState = STATE_LOST_SIGNAL;
  // Advertising restarts automatically (restartOnDisconnect(true) in setup).
}

// Fires when the hub WRITES a new hazard packet into our characteristic.
void hazard_write_callback(uint16_t conn_handle, BLECharacteristic* chr,
                           uint8_t* data, uint16_t len)
{
  (void) conn_handle;
  (void) chr;

  if (len != sizeof(HazardPacket))
  {
    // Malformed packet: don't apply it, don't count it as a watchdog tick.
    Serial.println("Hazard packet: bad length, ignoring");
    return;
  }

  HazardPacket incoming;
  memcpy(&incoming, data, sizeof(HazardPacket));

  hazardVersion++;                 // odd: write in progress
  __sync_synchronize();
  currentHazard = incoming;
  __sync_synchronize();
  hazardVersion++;                 // even: write complete

  lastSeq = incoming.seq;
  lastPacketMs = millis();         // keep this BEFORE the state change
  currentState = STATE_OK;

  if (DEBUG_STATUS)
  {
    Serial.printf("RX seq=%u sector=%u dist=%u urg=%u flags=0x%02X\n",
                  incoming.seq, incoming.sector, incoming.distance_cm, incoming.urgency, incoming.flags);
  }
}

// ---------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);   // wait briefly for USB serial
  Serial.println("Boot");

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("SafeLifts-Wristband");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // --- Hazard Service ---
  hazardService.begin();

  hazardChar.setProperties(CHR_PROPS_WRITE_WO_RESP);
  // Open permission for bring-up. Once bonding is validated, switch the
  // WRITE permission to SECMODE_ENC_NO_MITM so only the bonded hub can write.
  hazardChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  hazardChar.setFixedLen(sizeof(HazardPacket));
  hazardChar.setWriteCallback(hazard_write_callback);
  hazardChar.begin();

  // --- Advertising ---
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(hazardService);   // 3 + 3 + 18 = 24 of 31 bytes
  Bluefruit.ScanResponse.addName();                  // full name goes in scan response
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);        // 20ms - 152.5ms, units of 0.625ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);                    // 0 = advertise forever until connected

  currentState = STATE_ADVERTISING;
  Serial.println("Advertising started");

  // Display LAST. The radio is already advertising, so a display problem
  // (loose wire, wrong address) can never stop the wristband from
  // connecting. If the display is dead, displayOK stays false and every
  // draw function quietly does nothing.
  Serial.println("Starting OLED...");
  Wire.begin();   // XIAO nRF52840: SDA = D4, SCL = D5

  // List everything that answers on the I2C bus. Expect 0x3C (OLED) and
  // 0x5A (DRV2605L). Anything missing here is a wiring/power problem.
  Serial.print("I2C scan:");
  int i2cFound = 0;
  for (uint8_t a = 1; a < 127; a++)
  {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); i2cFound++; }
  }
  Serial.println(i2cFound ? "" : " (nothing found)");

  // display.begin() returns true even with no display attached, so check
  // that the OLED actually answers before trusting it.
  Wire.beginTransmission(OLED_ADDRESS);
  bool oledPresent = (Wire.endTransmission() == 0);
  displayOK = oledPresent && display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (displayOK)
  {
    display.clearDisplay();
    display.display();
  }
  Serial.println(displayOK ? "OLED init OK" : "OLED init FAILED - check wiring/address");

  // Haptics last, same reasoning. The DRV2605L forgets its calibration
  // whenever it loses power, so calibrate for this LRA on every boot.
  Serial.println("Starting haptics...");
  hapticOK = drv.begin();
  if (hapticOK)
  {
    drv.useLRA();            // actuator is an LRA, not an ERM
    drv.selectLibrary(6);    // TI LRA effect library

    Serial.println("Haptic auto-calibration (motor buzzes briefly)...");
    drv.setMode(DRV2605_MODE_AUTOCAL);
    drv.go();
    uint32_t t0 = millis();
    while ((drv.readRegister8(HAP_REG_GO) & 0x01) && (millis() - t0 < 2000)) delay(10);
    bool calFailed = (drv.readRegister8(HAP_REG_STATUS) & 0x08) != 0;
    Serial.println(calFailed ? "WARNING: auto-cal reported failure" : "Auto-cal OK");

    drv.setMode(DRV2605_MODE_REALTIME);   // manual amplitude control
    drv.setRealtimeValue(0);
    hapLastAmp = 0;
    Serial.println("DRV2605L init OK");
  }
  else
  {
    Serial.println("DRV2605L init FAILED - check wiring/address");
  }
}

// ---------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------
void loop()
{
  // --- Watchdog: runs whenever we're connected, whether or not data has arrived ---
  LinkState s = currentState;                      // read state FIRST...
  uint32_t lastPkt = lastPacketMs;                 // ...then the timestamp
  int32_t  dt = (int32_t)(millis() - lastPkt);     // a packet landing mid-read must not look like a huge gap
  uint32_t elapsed = (dt < 0) ? 0 : (uint32_t)dt;

  if (s == STATE_OK && elapsed > WATCHDOG_TIMEOUT_MS)
  {
    currentState = STATE_LOST_SIGNAL;
    Serial.println("WATCHDOG: no fresh packet in time — fail-safe");
  }
  else if (s == STATE_WAITING_FIRST && elapsed > FIRST_PACKET_TIMEOUT_MS)
  {
    currentState = STATE_LOST_SIGNAL;
    Serial.println("WATCHDOG: connected but no first packet — fail-safe");
  }

  // --- Act on current state ---
  switch (currentState)
  {
    case STATE_ADVERTISING:
      updateDisplay_Searching();
      triggerHaptic_None();         // nothing to warn about yet
      break;

    case STATE_WAITING_FIRST:
      updateDisplay_Waiting();
      triggerHaptic_None();
      break;

    case STATE_OK:
    {
      HazardPacket h;
      readHazard(h);
      updateDisplay_Hazard(h);
      triggerHaptic_Hazard(h.urgency, h.flags);
      break;
    }

    case STATE_LOST_SIGNAL:
      updateDisplay_LostSignal();
      triggerHaptic_LostSignal();   // MUST be distinct from any real hazard level
      break;
  }

  hapticTick();   // advance the current pattern (non-blocking)

  static uint32_t lastStatusMs = 0;
  if (DEBUG_STATUS && millis() - lastStatusMs >= 2000)
  {
    lastStatusMs = millis();
    Serial.printf("STATUS state=%d oled=%d haptic=%d\n", (int)currentState, (int)displayOK, (int)hapticOK);
  }

  delay(20);  // simple loop pacing; replace with a proper scheduler later
}

// ---------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------

// Print text horizontally centered, top of text at y.
void drawCentered(const char* text, int16_t y, uint8_t size, uint16_t color = SSD1306_WHITE)
{
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.setTextColor(color);
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - (int16_t)w) / 2, y);
  display.print(text);
}

// ---------------------------------------------------------------------
// Screens
// ---------------------------------------------------------------------
void updateDisplay_Searching()
{
  if (!displayOK || lastScreen == SCREEN_SEARCHING) return;

  display.clearDisplay();
  drawCentered("SEARCHING", 16, 2);
  drawCentered("for hub...", 40, 1);
  display.display();
  lastScreen = SCREEN_SEARCHING;
}

void updateDisplay_Waiting()
{
  if (!displayOK || lastScreen == SCREEN_WAITING) return;

  display.clearDisplay();
  drawCentered("CONNECTED", 16, 2);
  drawCentered("waiting for data...", 40, 1);
  display.display();
  lastScreen = SCREEN_WAITING;
}

// Fully inverted screen, so it can't be mistaken for any hazard screen.
void updateDisplay_LostSignal()
{
  if (!displayOK || lastScreen == SCREEN_LOST) return;

  display.clearDisplay();
  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  drawCentered("LOST",   6, 2, SSD1306_BLACK);
  drawCentered("SIGNAL", 26, 2, SSD1306_BLACK);
  drawCentered("CHECK HUB", 52, 1, SSD1306_BLACK);
  display.display();
  lastScreen = SCREEN_LOST;
}

void updateDisplay_Hazard(const HazardPacket& p)
{
  if (!displayOK) return;

  // Skip the redraw if nothing visible changed (seq is not shown).
  if (lastScreen == SCREEN_HAZARD &&
      p.sector == lastDrawn.sector && p.distance_cm == lastDrawn.distance_cm &&
      p.urgency == lastDrawn.urgency && p.flags == lastDrawn.flags)
  {
    return;
  }

  display.clearDisplay();

  if (p.urgency == URGENCY_CLEAR)
  {
    if (p.flags & FLAG_SENSOR_FAULT)
    {
      // Degraded: the hub is not sure the area is clear, so the headline is
      // the fault, never a big CLEAR. A double border without inversion keeps
      // this distinct from LOST SIGNAL (fully inverted) and from hazard screens.
      display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
      display.drawRect(2, 2, SCREEN_WIDTH - 4, SCREEN_HEIGHT - 4, SSD1306_WHITE);
      drawCentered("SENSOR", 8, 2);
      drawCentered("FAULT", 27, 2);
      drawCentered("CLEAR NOT CONFIRMED", 50, 1);   // 19 chars x 6 px = 114 px
    }
    else
    {
      // --- All clear ---
      drawCentered("CLEAR", 14, 3);
    }
  }
  else
  {
    uint8_t urg = (p.urgency > URGENCY_DANGER) ? URGENCY_DANGER : p.urgency;  // clamp bad values up
    bool hasSector = (p.sector <= SECTOR_FRONT_LEFT);

    // --- Direction ring (top = FRONT of the lift, clockwise) ---
    const int cx = 26, cy = 24, r = 18;
    display.drawCircle(cx, cy, r, SSD1306_WHITE);
    display.fillCircle(cx, cy, 1, SSD1306_WHITE);
    if (hasSector)
    {
      float a  = p.sector * (PI / 4.0f);
      int   dx = cx + (int)roundf(r * sinf(a));
      int   dy = cy - (int)roundf(r * cosf(a));
      display.fillCircle(dx, dy, 4, SSD1306_WHITE);
    }

    // --- Sector name ---
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(54, 2);
    display.print(hasSector ? SECTOR_NAMES[p.sector] : "UNKNOWN");

    // --- Distance ---
    char buf[8];
    bool numeric = (p.distance_cm != 255);
    if (numeric) snprintf(buf, sizeof(buf), "%u", p.distance_cm);
    else         snprintf(buf, sizeof(buf), "--");
    display.setTextSize(3);
    display.setCursor(54, 14);
    display.print(buf);
    if (numeric)
    {
      display.setTextSize(1);
      display.setCursor(54 + (int)strlen(buf) * 18 + 2, 28);
      display.print("cm");
    }

    // --- Flags line (fault takes priority over multi-object) ---
    display.setTextSize(1);
    display.setCursor(54, 38);
    if      (p.flags & FLAG_SENSOR_FAULT) display.print("SENSOR FAULT");
    else if (p.flags & FLAG_MULTI_OBJECT) display.print("MULTIPLE");

    // --- Urgency bar (bottom). DANGER is a solid inverted bar. ---
    const char* label = (urg == URGENCY_DANGER) ? "DANGER" :
                        (urg == URGENCY_WARNING) ? "WARNING" : "CAUTION";
    if (urg == URGENCY_DANGER)
    {
      display.fillRect(0, 47, SCREEN_WIDTH, 17, SSD1306_WHITE);
      drawCentered(label, 48, 2, SSD1306_BLACK);
    }
    else if (urg == URGENCY_WARNING)
    {
      display.drawRect(0, 47, SCREEN_WIDTH, 17, SSD1306_WHITE);
      drawCentered(label, 48, 2, SSD1306_WHITE);
    }
    else
    {
      drawCentered(label, 48, 2, SSD1306_WHITE);
    }
  }

  display.display();
  lastDrawn  = p;
  lastScreen = SCREEN_HAZARD;
}

// ---------------------------------------------------------------------
// Haptics
// ---------------------------------------------------------------------

// Write an amplitude, skipping the I2C transaction if it hasn't changed.
void hapticWrite(uint8_t amp)
{
  if (!hapticOK || amp == hapLastAmp) return;
  drv.setRealtimeValue(amp);
  hapLastAmp = amp;
}

// Select a pattern. Does nothing if it's already playing; otherwise
// restarts from the first step so a new level is felt immediately.
void hapticSet(HapticPattern p)
{
  if (p == hapPattern) return;
  hapPattern   = p;
  hapStep      = 0;
  hapStepStart = millis();
  if (p == HAP_NONE) hapticWrite(0);
  else               hapticWrite(PATTERNS[p].steps[0].amp);
}

// Call once per loop pass. Advances to the next step when due.
void hapticTick()
{
  if (hapPattern == HAP_NONE) return;

  const PatternDef& def = PATTERNS[hapPattern];
  uint32_t now = millis();
  if (now - hapStepStart >= def.steps[hapStep].ms)
  {
    hapStep = (hapStep + 1) % def.count;
    hapStepStart = now;
    hapticWrite(def.steps[hapStep].amp);
  }
}

void triggerHaptic_None()
{
  hapticSet(HAP_NONE);
}

void triggerHaptic_Hazard(uint8_t urgency, uint8_t flags)
{
  switch (urgency)
  {
    case URGENCY_CLEAR:
      // No hazard, but if the hub reports a sensor fault, "clear" is not
      // confirmed: give the fault its own quiet, distinct cue.
      hapticSet((flags & FLAG_SENSOR_FAULT) ? HAP_FAULT : HAP_NONE);
      break;
    case URGENCY_CAUTION: hapticSet(HAP_CAUTION); break;
    case URGENCY_WARNING: hapticSet(HAP_WARNING); break;
    default:              hapticSet(HAP_DANGER);  break;  // 3, and any invalid value, = strongest
  }
}

void triggerHaptic_LostSignal()
{
  if (TEST_MUTE_LOST_HAPTIC)
  {
    hapticSet(HAP_NONE);   // bench testing only
    return;
  }
  hapticSet(HAP_LOST);     // long slow buzz: unlike any hazard rhythm
}

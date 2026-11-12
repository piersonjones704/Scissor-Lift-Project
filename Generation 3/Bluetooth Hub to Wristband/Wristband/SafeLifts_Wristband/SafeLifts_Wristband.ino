#include <Arduino_SensorKit.h>
#include <Arduino_SensorKit_BMP280.h>
#include <Arduino_SensorKit_LIS3DHTR.h>

/*
 * SafeLifts Wristband — BLE GATT setup + watchdog / fail-safe state machine
 *
 * Board: Seeed XIAO nRF52840 Sense
 * Library: Adafruit nRF52 Arduino core (Bluefruit), bundled with the
 * Seeed nRF52 board package — no separate install needed for bluefruit.h
 * or Adafruit_TinyUSB.h.
 *
 * This file covers ONLY the BLE + watchdog logic. Display and haptic
 * driving are left as stub functions. Wire in the validated
 * Adafruit_DRV2605 and OLED code there.
 *
 * Role: this device is the GATT PERIPHERAL / SERVER. The hub (central/
 * client) connects to us and WRITES hazard packets into our Hazard
 * State characteristic. We never notify anyone — we just receive,
 * apply, and watch for staleness.
 *
 * Changes since handoff:
 *  - Watchdog gap closed (STATE_WAITING_FIRST + first-packet timeout)
 *  - Device name moved to scan response (avoids truncation)
 *  - Real UUIDs
 *  - Debug prints (RX line, state changes, "Advertising started")
 */

#include <bluefruit.h>

// TinyUSB's Serial object isn't linked in on this board unless this
// header is included, even though Serial.print()/begin() compile fine
// without it — omitting it causes an "undefined reference to Serial"
// LINKER error, not a compile error.
#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h> // for Serial
#endif

// ---------------------------------------------------------------------
// UUIDs
// Source of truth (string form, what nRF Connect and the hub will show):
//   Service: f20270e0-4d91-4592-8d79-61117906f549
//   Hazard:  095d4f3c-7f73-447f-bd75-97a0e4712903
//   Config:  9d2f5683-d54c-43cf-9a2e-14b7490f3e27
// Arrays are stored least-significant byte first (reversed), as
// Bluefruit expects.
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

// Reserved for the future Config characteristic. Not used by any code yet.
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

const uint32_t WATCHDOG_TIMEOUT_MS     = 400;   // between packets, once data is flowing
const uint32_t FIRST_PACKET_TIMEOUT_MS = 3000;  // grace for hub to discover services and start writing

volatile LinkState currentState = STATE_ADVERTISING;
volatile uint8_t   lastSeq = 0;
volatile uint32_t  lastPacketMs = 0;

HazardPacket currentHazard = { 0, SECTOR_CLEAR, 255, URGENCY_CLEAR, 0, 0 };

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
    // Malformed packet — do NOT apply it, and do NOT count it as a
    // fresh watchdog tick.
    Serial.println("Hazard packet: bad length, ignoring");
    return;
  }

  HazardPacket incoming;
  memcpy(&incoming, data, sizeof(HazardPacket));

  currentHazard = incoming;
  lastSeq = incoming.seq;
  lastPacketMs = millis();      // keep this BEFORE the state change
  currentState = STATE_OK;

  Serial.printf("RX seq=%u sector=%u dist=%u urg=%u\n",
                incoming.seq, incoming.sector, incoming.distance_cm, incoming.urgency);
}

// ---------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);   // wait briefly for USB serial

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
}

// ---------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------
void loop()
{
  // --- Watchdog: runs whenever we're connected, whether or not data has arrived ---
  LinkState s = currentState;                      // read state FIRST...
  uint32_t elapsed = millis() - lastPacketMs;      // ...then the timestamp

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
      // No haptic — nothing to warn about yet.
      break;

    case STATE_WAITING_FIRST:
      updateDisplay_Waiting();
      break;

    case STATE_OK:
      updateDisplay_Hazard(currentHazard);
      triggerHaptic_Hazard(currentHazard.urgency);
      break;

    case STATE_LOST_SIGNAL:
      updateDisplay_LostSignal();
      triggerHaptic_LostSignal();   // MUST be distinct from any real hazard level
      break;
  }

  delay(20);  // simple loop pacing; replace with a proper scheduler later
}

// ---------------------------------------------------------------------
// Stubs — wire up validated OLED + DRV2605L code here
// ---------------------------------------------------------------------
void updateDisplay_Searching()  { /* TODO: "Searching for hub..." */ }
void updateDisplay_Waiting()    { /* TODO: "Hub connected, waiting..." */ }
void updateDisplay_LostSignal() { /* TODO: distinct "LOST SIGNAL" screen */ }
void updateDisplay_Hazard(const HazardPacket& p) { (void)p; /* TODO: sector + distance */ }
void triggerHaptic_Hazard(uint8_t urgency)       { (void)urgency; /* TODO: cadence mapping */ }
void triggerHaptic_LostSignal() { /* TODO: distinct slow steady pulse */ }

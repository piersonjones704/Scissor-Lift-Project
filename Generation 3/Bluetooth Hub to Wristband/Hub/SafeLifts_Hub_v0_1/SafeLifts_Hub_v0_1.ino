/*
  SafeLifts_Hub_v0_1  --  Track B, build steps 2-3 (minimal BLE central)

  Job: find the wristband (scan filtered by Hazard Service UUID), connect,
  discover the Hazard State characteristic, then write a 6-byte test packet
  every ~100 ms using Write Without Response. Reconnect automatically.

  NOT in this version: sensors, hazard logic, bonding/security.
  The packet contents are fixed test packets chosen from the Serial Monitor.

  Target:   ESP32-S3 (Espressif) dev board
  Library:  NimBLE-Arduino by h2zero, version 2.x  (API differs from 1.4.x)
  Board:    "ESP32S3 Dev Module", USB CDC On Boot = Enabled

  Serial commands (115200 baud):
    1..6  select test packet (see TEST_PACKETS below)
    p     pause / resume writes while STAYING connected
          (wristband should fall to LOST SIGNAL ~400 ms after pausing)
*/

#include <Arduino.h>
#include <NimBLEDevice.h>

// ---------------------------------------------------------------------------
// Interface contract (strings are the source of truth; NimBLE handles order)
// ---------------------------------------------------------------------------
static const char* HAZARD_SERVICE_UUID = "f20270e0-4d91-4592-8d79-61117906f549";
static const char* HAZARD_STATE_UUID   = "095d4f3c-7f73-447f-bd75-97a0e4712903";

static const size_t PACKET_LEN = 6;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static const uint32_t WRITE_INTERVAL_MS  = 100;   // wristband watchdog is 400 ms
static const uint32_t STATUS_INTERVAL_MS = 2000;
static const uint32_t CONNECT_TIMEOUT_MS = 5000;

// Connection parameters. Units: interval = 1.25 ms, supervision timeout = 10 ms.
static const uint16_t CONN_INTERVAL_MIN = 12;     // 15 ms
static const uint16_t CONN_INTERVAL_MAX = 24;     // 30 ms
static const uint16_t CONN_LATENCY      = 0;
static const uint16_t CONN_TIMEOUT      = 400;    // 4 s

// ---------------------------------------------------------------------------
// Test packets from the handoff (byte 0 = seq is overwritten on every write)
// ---------------------------------------------------------------------------
static const uint8_t TEST_PACKETS[6][PACKET_LEN] = {
  {0x01, 0x00, 0x32, 0x02, 0x00, 0x00},  // 1: FRONT 50 cm WARNING
  {0x02, 0x02, 0x1E, 0x03, 0x00, 0x00},  // 2: RIGHT 30 cm DANGER
  {0x03, 0x05, 0x64, 0x01, 0x01, 0x00},  // 3: BACK-LEFT 100 cm CAUTION, multi-object
  {0x04, 0xFF, 0xFF, 0x00, 0x00, 0x00},  // 4: CLEAR
  {0x05, 0x03, 0x0F, 0x03, 0x02, 0x00},  // 5: BACK-RIGHT 15 cm DANGER, sensor fault
  {0x06, 0x07, 0xFF, 0x01, 0x00, 0x00},  // 6: FRONT-LEFT, distance unknown
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum HubState : uint8_t { ST_SCANNING = 0, ST_CONNECTING = 1, ST_STREAMING = 2 };

static HubState state = ST_SCANNING;

static NimBLEClient*               client   = nullptr;
static NimBLERemoteCharacteristic* hazardChr = nullptr;
static NimBLEAddress*              targetAddr = nullptr;   // heap copy of the found wristband

static volatile bool foundWristband = false;   // set by scan callback
static volatile bool linkDropped    = false;   // set by disconnect callback

static uint8_t  seqCounter   = 0;
static uint8_t  packetIndex  = 0;              // 0..5 -> TEST_PACKETS
static bool     paused       = false;

static uint32_t lastWriteMs  = 0;
static uint32_t lastStatusMs = 0;
static uint32_t writeCount   = 0;
static uint32_t writeFails   = 0;

// ---------------------------------------------------------------------------
// Callbacks (keep them tiny: set flags only, do the work in loop())
// ---------------------------------------------------------------------------
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (foundWristband) return;                                  // already have one
    if (!dev->isAdvertisingService(NimBLEUUID(HAZARD_SERVICE_UUID))) return;

    Serial.printf("[HUB] Found wristband: %s name='%s' rssi=%d\n",
                  dev->getAddress().toString().c_str(),
                  dev->getName().c_str(),
                  dev->getRSSI());

    if (targetAddr) { delete targetAddr; targetAddr = nullptr; }
    targetAddr = new NimBLEAddress(dev->getAddress());
    foundWristband = true;
    NimBLEDevice::getScan()->stop();
  }
};

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override {
    Serial.println("[HUB] Link up");
  }
  void onDisconnect(NimBLEClient* c, int reason) override {
    Serial.printf("[HUB] Link down (reason=%d)\n", reason);
    linkDropped = true;
  }
};

static ScanCallbacks   scanCallbacks;
static ClientCallbacks clientCallbacks;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void startScan() {
  foundWristband = false;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);      // also fetch scan response (device name) for the log
  scan->setInterval(100);
  scan->setWindow(100);
  scan->start(0, false, true);    // duration 0 = scan until stopped
  state = ST_SCANNING;
  Serial.println("[HUB] Scanning for Hazard Service...");
}

// Connect + discover. Returns true only if the characteristic is ready to write.
// Blocking is acceptable in this test sketch (worst case ~5 s).
static bool connectAndDiscover() {
  if (!targetAddr) return false;

  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCallbacks, false);
    client->setConnectionParams(CONN_INTERVAL_MIN, CONN_INTERVAL_MAX,
                                CONN_LATENCY, CONN_TIMEOUT);
    client->setConnectTimeout(CONNECT_TIMEOUT_MS);
  }

  linkDropped = false;
  hazardChr = nullptr;

  Serial.println("[HUB] Connecting...");
  if (!client->connect(*targetAddr)) {
    Serial.println("[HUB] Connect failed");
    return false;
  }
  Serial.println("[HUB] Connected, discovering...");

  NimBLERemoteService* svc = client->getService(HAZARD_SERVICE_UUID);
  if (!svc) {
    Serial.println("[HUB] Hazard Service NOT found on peer");
    client->disconnect();
    return false;
  }

  NimBLERemoteCharacteristic* chr = svc->getCharacteristic(HAZARD_STATE_UUID);
  if (!chr) {
    Serial.println("[HUB] Hazard State characteristic NOT found");
    client->disconnect();
    return false;
  }

  Serial.printf("[HUB] Characteristic found, WriteNoResponse=%s\n",
                chr->canWriteNoResponse() ? "yes" : "NO");
  if (!chr->canWriteNoResponse()) {
    Serial.println("[HUB] Peer does not allow Write Without Response; aborting");
    client->disconnect();
    return false;
  }

  hazardChr = chr;
  return true;
}

static void sendPacket() {
  uint8_t pkt[PACKET_LEN];
  memcpy(pkt, TEST_PACKETS[packetIndex], PACKET_LEN);
  pkt[0] = seqCounter++;                       // wraps at 255 naturally

  // response=false -> Write Without Response
  bool ok = hazardChr->writeValue(pkt, PACKET_LEN, false);
  if (ok) writeCount++; else writeFails++;
}

static void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c >= '1' && c <= '6') {
      packetIndex = (uint8_t)(c - '1');
      Serial.printf("[HUB] Test packet %d selected\n", packetIndex + 1);
    } else if (c == 'p' || c == 'P') {
      paused = !paused;
      Serial.printf("[HUB] Writes %s (link stays up)\n", paused ? "PAUSED" : "RESUMED");
      if (!paused) lastWriteMs = millis() - WRITE_INTERVAL_MS;  // write immediately
    }
  }
}

static void printStatus() {
  Serial.printf("[HUB] STATUS state=%u writes=%lu fails=%lu pkt=%d paused=%d\n",
                (unsigned)state, (unsigned long)writeCount,
                (unsigned long)writeFails, packetIndex + 1, paused ? 1 : 0);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }   // output before monitor opens is lost
  delay(300);

  Serial.println();
  Serial.println("[HUB] SafeLifts hub v0.1 starting (test sketch, fixed packets)");

  NimBLEDevice::init("SafeLifts-Hub");
  startScan();
}

void loop() {
  handleSerial();
  uint32_t now = millis();

  switch (state) {

    case ST_SCANNING:
      if (foundWristband) {
        state = ST_CONNECTING;
      }
      break;

    case ST_CONNECTING:
      if (connectAndDiscover()) {
        state = ST_STREAMING;
        // Wristband only waits 3 s for the first write: send one right now.
        lastWriteMs = now - WRITE_INTERVAL_MS;
        Serial.printf("[HUB] Streaming packet %d every %lu ms\n",
                      packetIndex + 1, (unsigned long)WRITE_INTERVAL_MS);
      } else {
        delay(500);               // brief back-off, then look again
        startScan();
      }
      break;

    case ST_STREAMING:
      if (linkDropped || !client || !client->isConnected()) {
        Serial.println("[HUB] Lost the wristband, rescanning");
        hazardChr = nullptr;
        startScan();
        break;
      }
      if (!paused && (now - lastWriteMs) >= WRITE_INTERVAL_MS) {
        lastWriteMs = now;
        sendPacket();
      }
      break;
  }

  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    printStatus();
  }

  delay(1);
}

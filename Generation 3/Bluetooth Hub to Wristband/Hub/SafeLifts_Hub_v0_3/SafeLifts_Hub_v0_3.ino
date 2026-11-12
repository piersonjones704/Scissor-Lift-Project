/*
  SafeLifts_Hub_v0_2  --  build steps 4-6: simulated modules + hazard logic

  What's new vs v0.1:
    - Two SIMULATED sensor modules (0 = FRONT, 1 = BACK) that you drive from
      the Serial Monitor. They produce real 15-byte module packets that go
      through the same parser a real BLE module will use later.
    - Hazard aggregation: per-sector nearest object across both modules,
      thresholds -> urgency, pick the worst sector, build the 6-byte wristband
      packet.
    - Staleness: a module that stops reporting is treated as UNKNOWN, never
      as clear. The hub then sets the "sensor fault" flag (bit1).
    - The BLE link to the wristband is unchanged from v0.1.

  Files: keep safelifts_module_packet.h in the same folder as this sketch.
  Library: NimBLE-Arduino 2.x.  Board: ESP32S3 Dev Module, USB CDC On Boot Enabled.
  Serial Monitor: 115200 baud, line ending set to "Newline".

  Commands (type, then Enter). Sectors: f fr r br b bl l fl  (or 0-7)
    m0 f 50         module 0 (FRONT) sees something 50 cm away in sector FRONT
    m1 b clear      module 1 (BACK) sector BACK: nothing in range
    m0 fl unk       module 0 sector FRONT-LEFT: unknown reading
    m0 off / on     stop / restart module 0 reports (tests staleness)
    m0 fault tofa|tofb|tofc|tofd|radar|none   set module 0 fault flags
    m0 multi f      toggle "multiple objects" for sector FRONT on module 0
    reset           all simulated modules back to clear, no faults, running
    pause           stop writing to the wristband (link stays up)
    status          print module details
    help
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "safelifts_module_packet.h"

// ---------------------------------------------------------------------------
// Wristband interface contract
// ---------------------------------------------------------------------------
static const char* HAZARD_SERVICE_UUID = "f20270e0-4d91-4592-8d79-61117906f549";
static const char* HAZARD_STATE_UUID   = "095d4f3c-7f73-447f-bd75-97a0e4712903";
static const size_t PACKET_LEN = 6;   // [seq, sector, distance_cm, urgency, flags, reserved]

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static const uint32_t WRITE_INTERVAL_MS  = 100;   // wristband watchdog is 400 ms
static const uint32_t STATUS_INTERVAL_MS = 2000;
static const uint32_t CONNECT_TIMEOUT_MS = 5000;

static const uint16_t CONN_INTERVAL_MIN = 12;     // x1.25 ms = 15 ms
static const uint16_t CONN_INTERVAL_MAX = 24;     // x1.25 ms = 30 ms
static const uint16_t CONN_LATENCY      = 0;
static const uint16_t CONN_TIMEOUT      = 400;    // x10 ms = 4 s

// ---------------------------------------------------------------------------
// Hazard thresholds (PLACEHOLDERS: real values are an open decision)
// urgency: 0 clear, 1 caution, 2 warning, 3 danger
// ---------------------------------------------------------------------------
static const uint8_t DANGER_CM  = 40;    // below this: urgency 3
static const uint8_t WARNING_CM = 80;    // below this: urgency 2
static const uint8_t CAUTION_CM = 150;   // below this: urgency 1

// A module is stale if nothing arrived for STALE_FACTOR x its own frame period.
static const uint8_t STALE_FACTOR = 3;

static const char* SECTOR_NAMES[SL_NUM_SECTORS] = {
  "FRONT", "FRONT-RIGHT", "RIGHT", "BACK-RIGHT", "BACK", "BACK-LEFT", "LEFT", "FRONT-LEFT"
};

// ---------------------------------------------------------------------------
// Module state (what the hub knows about each module)
// ---------------------------------------------------------------------------
struct ModuleState {
  bool           everRx;
  uint32_t       lastRxMs;
  SLModuleReport last;
  uint32_t       rxCount;
  uint32_t       gaps;
};
static ModuleState mods[SL_NUM_MODULES];
static uint32_t    badPackets = 0;

// ---------------------------------------------------------------------------
// Simulated modules (replace with real BLE/radio reception later)
// ---------------------------------------------------------------------------
struct SimModule {
  bool           enabled;
  uint8_t        seq;
  uint32_t       lastSendMs;
  SLModuleReport rep;
};
static SimModule sims[SL_NUM_MODULES];

// ---------------------------------------------------------------------------
// BLE / link state (same as v0.1)
// ---------------------------------------------------------------------------
enum HubState : uint8_t { ST_SCANNING = 0, ST_CONNECTING = 1, ST_STREAMING = 2 };
static HubState state = ST_SCANNING;

static NimBLEClient*               client     = nullptr;
static NimBLERemoteCharacteristic* hazardChr  = nullptr;
static NimBLEAddress*              targetAddr = nullptr;

static volatile bool foundWristband = false;
static volatile bool linkDropped    = false;

static uint8_t  seqCounter   = 0;
static bool     paused       = false;
static uint32_t lastTickMs   = 0;
static uint32_t lastStatusMs = 0;
static uint32_t writeCount   = 0;
static uint32_t writeFails   = 0;

static char     lineBuf[64];
static uint8_t  lineLen = 0;

// ---------------------------------------------------------------------------
// Module packet intake (the ONLY place module data enters the hub)
// ---------------------------------------------------------------------------
static void onModulePacket(const uint8_t* d, size_t len, uint32_t now) {
  SLModuleReport r;
  if (!slDecode(d, len, r)) {
    badPackets++;
    Serial.println("[HUB] Bad module packet dropped");
    return;
  }
  ModuleState& ms = mods[r.module_id];
  if (ms.everRx) {
    uint8_t diff = (uint8_t)(r.seq - ms.last.seq);
    if (diff == 0 || diff > 128) return;        // duplicate or out-of-order: ignore
    if (diff > 1) ms.gaps += (uint32_t)(diff - 1);
  }
  ms.last     = r;
  ms.everRx   = true;
  ms.lastRxMs = now;
  ms.rxCount++;
}

static bool moduleFresh(int m, uint32_t now) {
  const ModuleState& ms = mods[m];
  if (!ms.everRx) return false;
  uint32_t limit = (uint32_t)STALE_FACTOR * ms.last.frame_period_10ms * 10UL;
  return (now - ms.lastRxMs) <= limit;
}

// ---------------------------------------------------------------------------
// Hazard aggregation
// ---------------------------------------------------------------------------
static uint8_t urgencyForDistance(uint8_t cm) {
  if (cm < DANGER_CM)  return 3;
  if (cm < WARNING_CM) return 2;
  if (cm < CAUTION_CM) return 1;
  return 0;
}

// Fills body[1..5] (sector, distance, urgency, flags, reserved). body[0] (seq) is set at write time.
static void computeWristbandPacket(uint32_t now, uint8_t* body) {
  bool fresh[SL_NUM_MODULES];
  bool degraded = false;

  for (int m = 0; m < SL_NUM_MODULES; m++) {
    fresh[m] = moduleFresh(m, now);
    if (fresh[m] && (mods[m].last.fault_flags & SL_FAULT_SENSOR_MASK)) degraded = true;
  }

  uint8_t bestUrg = 0, bestSector = 0xFF, bestDist = 0xFF;
  bool    bestMulti = false;

  for (int s = 0; s < SL_NUM_SECTORS; s++) {
    bool    haveDist = false, unknown = false, multi = false;
    uint8_t d = 0xFF;

    for (int m = 0; m < SL_NUM_MODULES; m++) {
      if (!((SL_MODULE_COVERAGE[m] >> s) & 1)) continue;   // hub-side config: not this module's sector
      if (!fresh[m]) { unknown = true; continue; }          // silent module: sector is unknown

      uint8_t v = mods[m].last.sector[s];
      if (v <= SL_DIST_MAX_CM) {
        if (!haveDist || v < d) d = v;
        haveDist = true;
        if ((mods[m].last.multi_mask >> s) & 1) multi = true;
      } else if (v == SL_DIST_CLEAR) {
        // a clear vote: nothing to add
      } else {
        unknown = true;      // 0xFE, 0xFD (config mismatch) or any reserved value
      }
    }

    if (unknown) degraded = true;

    if (haveDist) {
      uint8_t u = urgencyForDistance(d);
      if (u > bestUrg || (u == bestUrg && u > 0 && d < bestDist)) {
        bestUrg = u; bestSector = (uint8_t)s; bestDist = d; bestMulti = multi;
      }
    }
  }

  uint8_t flags = 0;
  if (bestUrg > 0 && bestMulti) flags |= 0x01;   // bit0: multiple objects
  if (degraded)                 flags |= 0x02;   // bit1: sensor fault / module offline / unknown sector

  body[1] = (bestUrg > 0) ? bestSector : 0xFF;
  body[2] = (bestUrg > 0) ? bestDist   : 0xFF;
  body[3] = bestUrg;
  body[4] = flags;
  body[5] = 0;
}

static void reportIfChanged(const uint8_t* body) {
  static uint8_t last[4] = {0xAA, 0xAA, 0xAA, 0xAA};
  if (memcmp(last, body + 1, 4) == 0) return;
  memcpy(last, body + 1, 4);

  Serial.printf("[HUB] HAZARD sector=%s ", body[1] == 0xFF ? "CLEAR" : SECTOR_NAMES[body[1] & 7]);
  if (body[2] == 0xFF) Serial.print("dist=-- "); else Serial.printf("dist=%ucm ", body[2]);
  Serial.printf("urg=%u flags=0x%02X%s%s\n", body[3], body[4],
                (body[4] & 0x01) ? " MULTI" : "", (body[4] & 0x02) ? " DEGRADED" : "");
}

// ---------------------------------------------------------------------------
// Simulated modules
// ---------------------------------------------------------------------------
static void simReset() {
  for (int m = 0; m < SL_NUM_MODULES; m++) {
    SLModuleReport& r = sims[m].rep;
    memset(&r, 0, sizeof(r));
    r.version           = SL_PKT_VERSION;
    r.module_id         = (uint8_t)m;
    r.frame_period_10ms = 10;                           // 100 ms
    for (int s = 0; s < SL_NUM_SECTORS; s++)
      r.sector[s] = ((SL_MODULE_COVERAGE[m] >> s) & 1) ? SL_DIST_CLEAR : SL_DIST_NOT_COVERED;
    sims[m].enabled = true;
  }
}

static void simTick(uint32_t now) {
  for (int m = 0; m < SL_NUM_MODULES; m++) {
    if (!sims[m].enabled) continue;
    uint32_t period = (uint32_t)sims[m].rep.frame_period_10ms * 10UL;
    if (now - sims[m].lastSendMs >= period) {
      sims[m].lastSendMs = now;
      sims[m].rep.seq = sims[m].seq++;
      uint8_t buf[SL_PKT_LEN];
      slEncode(sims[m].rep, buf);
      onModulePacket(buf, SL_PKT_LEN, now);
    }
  }
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------
static int parseSector(const char* t) {
  static const char* names[SL_NUM_SECTORS] = {"f", "fr", "r", "br", "b", "bl", "l", "fl"};
  for (int i = 0; i < SL_NUM_SECTORS; i++) if (strcmp(t, names[i]) == 0) return i;
  if (t[0] >= '0' && t[0] <= '7' && t[1] == 0) return t[0] - '0';
  return -1;
}

static void printHelp() {
  Serial.println("[HUB] Commands: m0|m1 <sector> <cm|clear|unk> | m0|m1 off|on | m0|m1 fault tofa|tofb|tofc|tofd|radar|none");
  Serial.println("[HUB]           m0|m1 multi <sector> | reset | pause | status | help");
  Serial.println("[HUB] Sectors: f fr r br b bl l fl (or 0-7). Module 0 = FRONT, module 1 = BACK.");
}

static void printModules(uint32_t now) {
  for (int m = 0; m < SL_NUM_MODULES; m++) {
    const ModuleState& ms = mods[m];
    Serial.printf("[HUB] module %d: %s rx=%lu gaps=%lu faults=0x%02X multi=0x%02X sim=%s sectors:",
                  m, moduleFresh(m, now) ? "FRESH" : (ms.everRx ? "STALE" : "NEVER-HEARD"),
                  (unsigned long)ms.rxCount, (unsigned long)ms.gaps,
                  ms.last.fault_flags, ms.last.multi_mask, sims[m].enabled ? "on" : "off");
    for (int s = 0; s < SL_NUM_SECTORS; s++) {
      uint8_t v = ms.last.sector[s];
      if (!ms.everRx)                       Serial.print(" ?");
      else if (v <= SL_DIST_MAX_CM)         Serial.printf(" %u", v);
      else if (v == SL_DIST_CLEAR)          Serial.print(" clr");
      else if (v == SL_DIST_NOT_COVERED)    Serial.print(" n/c");
      else                                  Serial.print(" unk");
    }
    Serial.println();
  }
  Serial.printf("[HUB] bad module packets: %lu\n", (unsigned long)badPackets);
}

static void handleModuleCmd(int m, char** a, int na) {
  if (na == 1 && strcmp(a[0], "off") == 0) {
    sims[m].enabled = false;
    Serial.printf("[SIM] module %d OFF (stops sending)\n", m);
    return;
  }
  if (na == 1 && strcmp(a[0], "on") == 0) {
    sims[m].enabled = true;
    Serial.printf("[SIM] module %d ON\n", m);
    return;
  }
  if (na == 2 && strcmp(a[0], "fault") == 0) {
    uint8_t f = 0;
    if      (strcmp(a[1], "tofa")  == 0) f = SL_FAULT_TOF_A;
    else if (strcmp(a[1], "tofb")  == 0) f = SL_FAULT_TOF_B;
    else if (strcmp(a[1], "tofc")  == 0) f = SL_FAULT_TOF_C;
    else if (strcmp(a[1], "tofd")  == 0) f = SL_FAULT_TOF_D;
    else if (strcmp(a[1], "radar") == 0) f = SL_FAULT_RADAR;
    else if (strcmp(a[1], "none")  == 0) f = 0;
    else { Serial.println("[SIM] fault must be tofa, tofb, tofc, tofd, radar or none"); return; }
    sims[m].rep.fault_flags = f;
    Serial.printf("[SIM] module %d fault flags = 0x%02X\n", m, f);
    return;
  }
  if (na == 2 && strcmp(a[0], "multi") == 0) {
    int s = parseSector(a[1]);
    if (s < 0) { Serial.println("[SIM] unknown sector"); return; }
    sims[m].rep.multi_mask ^= (uint8_t)(1u << s);
    Serial.printf("[SIM] module %d multi_mask = 0x%02X\n", m, sims[m].rep.multi_mask);
    return;
  }
  if (na == 2) {
    int s = parseSector(a[0]);
    if (s < 0) { Serial.println("[SIM] unknown sector (use f fr r br b bl l fl or 0-7)"); return; }
    if (!((SL_MODULE_COVERAGE[m] >> s) & 1)) {
      Serial.printf("[SIM] module %d does not cover %s, ignored\n", m, SECTOR_NAMES[s]);
      return;
    }
    uint8_t v;
    if      (strcmp(a[1], "clear") == 0) v = SL_DIST_CLEAR;
    else if (strcmp(a[1], "unk")   == 0) v = SL_DIST_UNKNOWN;
    else {
      char* end = nullptr;
      long cm = strtol(a[1], &end, 10);
      if (end == a[1] || *end != 0 || cm < 0 || cm > SL_DIST_MAX_CM) {
        Serial.println("[SIM] distance must be 0-250, clear or unk");
        return;
      }
      v = (uint8_t)cm;
    }
    sims[m].rep.sector[s] = v;
    Serial.printf("[SIM] module %d %s = %s\n", m, SECTOR_NAMES[s], a[1]);
    return;
  }
  Serial.println("[SIM] unknown module command (type help)");
}

static void handleCommand(char* line, uint32_t now) {
  for (char* p = line; *p; ++p) *p = (char)tolower((unsigned char)*p);

  char* tok[4];
  int   n = 0;
  char* save = nullptr;
  for (char* t = strtok_r(line, " \t\r", &save); t && n < 4; t = strtok_r(nullptr, " \t\r", &save))
    tok[n++] = t;
  if (n == 0) return;

  if (strcmp(tok[0], "pause") == 0) {
    paused = !paused;
    Serial.printf("[HUB] Wristband writes %s (link stays up)\n", paused ? "PAUSED" : "RESUMED");
  } else if (strcmp(tok[0], "reset") == 0) {
    simReset();
    Serial.println("[SIM] all modules reset: clear, no faults, running");
  } else if (strcmp(tok[0], "status") == 0) {
    printModules(now);
  } else if (strcmp(tok[0], "help") == 0) {
    printHelp();
  } else if (tok[0][0] == 'm' && tok[0][1] >= '0' && tok[0][1] < '0' + SL_NUM_MODULES && tok[0][2] == 0) {
    handleModuleCmd(tok[0][1] - '0', tok + 1, n - 1);
  } else {
    Serial.println("[HUB] Unknown command (type help)");
  }
}

static void handleSerial(uint32_t now) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        lineLen = 0;
        handleCommand(lineBuf, now);
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// BLE client (same as v0.1)
// ---------------------------------------------------------------------------
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (foundWristband) return;
    if (!dev->isAdvertisingService(NimBLEUUID(HAZARD_SERVICE_UUID))) return;

    Serial.printf("[HUB] Found wristband: %s name='%s' rssi=%d\n",
                  dev->getAddress().toString().c_str(), dev->getName().c_str(), dev->getRSSI());

    if (targetAddr) { delete targetAddr; targetAddr = nullptr; }
    targetAddr = new NimBLEAddress(dev->getAddress());
    foundWristband = true;
    NimBLEDevice::getScan()->stop();
  }
};

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override { Serial.println("[HUB] Link up"); }
  void onDisconnect(NimBLEClient* c, int reason) override {
    Serial.printf("[HUB] Link down (reason=%d)\n", reason);
    linkDropped = true;
  }
};

static ScanCallbacks   scanCallbacks;
static ClientCallbacks clientCallbacks;

static void startScan() {
  foundWristband = false;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(100);
  scan->start(0, false, true);
  state = ST_SCANNING;
  Serial.println("[HUB] Scanning for Hazard Service...");
}

static bool connectAndDiscover() {
  if (!targetAddr) return false;

  if (!client) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCallbacks, false);
    client->setConnectionParams(CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, CONN_LATENCY, CONN_TIMEOUT);
    client->setConnectTimeout(CONNECT_TIMEOUT_MS);
  }

  linkDropped = false;
  hazardChr   = nullptr;

  Serial.println("[HUB] Connecting...");
  if (!client->connect(*targetAddr)) { Serial.println("[HUB] Connect failed"); return false; }
  Serial.println("[HUB] Connected, discovering...");

  NimBLERemoteService* svc = client->getService(HAZARD_SERVICE_UUID);
  if (!svc) { Serial.println("[HUB] Hazard Service NOT found on peer"); client->disconnect(); return false; }

  NimBLERemoteCharacteristic* chr = svc->getCharacteristic(HAZARD_STATE_UUID);
  if (!chr) { Serial.println("[HUB] Hazard State characteristic NOT found"); client->disconnect(); return false; }

  if (!chr->canWriteNoResponse()) {
    Serial.println("[HUB] Peer does not allow Write Without Response; aborting");
    client->disconnect();
    return false;
  }
  hazardChr = chr;
  return true;
}

static void sendToWristband(const uint8_t* body) {
  if (!hazardChr) return;
  uint8_t pkt[PACKET_LEN];
  memcpy(pkt, body, PACKET_LEN);
  pkt[0] = seqCounter++;
  if (hazardChr->writeValue(pkt, PACKET_LEN, false)) writeCount++; else writeFails++;
}

static void printStatus(uint32_t now) {
  Serial.printf("[HUB] STATUS link=%u writes=%lu fails=%lu paused=%d m0=%s m1=%s\n",
                (unsigned)state, (unsigned long)writeCount, (unsigned long)writeFails, paused ? 1 : 0,
                moduleFresh(0, now) ? "ok" : "STALE", moduleFresh(1, now) ? "ok" : "STALE");
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }
  delay(300);

  Serial.println();
  Serial.println("[HUB] SafeLifts hub v0.2 starting (simulated modules, type 'help')");

  memset(mods, 0, sizeof(mods));
  simReset();

  NimBLEDevice::init("SafeLifts-Hub");
  startScan();
}

void loop() {
  uint32_t now = millis();
  handleSerial(now);
  simTick(now);

  switch (state) {
    case ST_SCANNING:
      if (foundWristband) state = ST_CONNECTING;
      break;

    case ST_CONNECTING:
      if (connectAndDiscover()) {
        state = ST_STREAMING;
        lastTickMs = millis() - WRITE_INTERVAL_MS;     // write right away
        Serial.printf("[HUB] Streaming to wristband every %lu ms\n", (unsigned long)WRITE_INTERVAL_MS);
      } else {
        delay(500);
        startScan();
      }
      break;

    case ST_STREAMING:
      if (linkDropped || !client || !client->isConnected()) {
        Serial.println("[HUB] Lost the wristband, rescanning");
        hazardChr = nullptr;
        startScan();
      }
      break;
  }

  now = millis();
  simTick(now);   // catch up after a blocking connect so the modules don't look stale
  if (now - lastTickMs >= WRITE_INTERVAL_MS) {
    lastTickMs = now;
    uint8_t body[PACKET_LEN];
    computeWristbandPacket(now, body);       // runs even without a wristband, so logic is testable alone
    reportIfChanged(body);
    if (state == ST_STREAMING && !paused) sendToWristband(body);
  }

  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    printStatus(now);
  }

  delay(1);
}

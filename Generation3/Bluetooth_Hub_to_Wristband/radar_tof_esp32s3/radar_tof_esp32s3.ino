// IWRL6432FSPEVM + 2x VL53L8CX -> ESP32-S3 (radar + dual ToF, streams JSON lines over USB)
// Board: "ESP32S3 Dev Module", Tools > USB CDC On Boot: Enabled
// Serial Monitor: 115200
//
// Wiring:  ESP 3V3 -> EVM 3V3      ESP GND -> EVM GND
//          ESP TX pin (GPIO43) -> EVM UART RX
//          ESP RX pin (GPIO44) <- EVM UART TX
//          EVM SOP0 -> EVM 3V3 (run mode)   EVM SOP1 -> EVM GND
//          ToF: SDA=GPIO8, SCL=GPIO9 (shared), LPn1=GPIO5, LPn2=GPIO6, VIN=3V3, SPI/I2C pin -> GND
// Power-cycle the EVM after changing SOP jumpers.
//
// Output lines (everything else starts with '#'):
//   {"f":N,"p":[[x,y,z,v,snr],...]}                 radar frame
//   {"tof":1|2,"d":[64 ints, mm, row-major, -1 = invalid]}   ToF 8x8 frame

#include <Arduino.h>
#include <Wire.h>
#include <vl53l8cx.h>

#define RADAR_RX   44   // board pin labeled RX (UART0 RX)
#define RADAR_TX   43   // board pin labeled TX (UART0 TX)
#define RADAR_BAUD 115200

// 1 = ESP32 acts as a dumb USB<->UART cable (type commands like "version"
//     in Serial Monitor, set line ending to "Newline"). 0 = normal run.
#define BRIDGE_MODE 0

HardwareSerial Radar(1);

// Based on TI's xWRL6432 motion/presence example configs.
// clutterRemoval is 0 so STATIC objects (walls, people standing still) show up.
// If any line prints "Error", replace this list with the .cfg from
// mmWave-L-SDK: examples/mmw_demo/motion_and_presence_detection/profiles/xwrL64xx-evm
const char* CFG[] = {
  "sensorStop 0",
  "channelCfg 7 3 0",
  "chirpComnCfg 16 0 0 128 4 28 0",
  "chirpTimingCfg 6 32 0 40 60.5",
  "frameCfg 2 0 200 64 100 0",
  "antGeometryCfg 0 0 1 1 0 2 0 1 1 2 0 3 2.418 2.418",
  "guiMonitor 2 0 0 0 0 1 0 0 0 0 0",
  "sigProcChainCfg 32 2 3 2 8 8 1 0.3",
  "cfarCfg 2 8 4 3 0 12.0 0 0.5 0 1 1 1",
  "aoaFovCfg -70 70 -40 40",
  "rangeSelCfg 0.1 10.0",
  "clutterRemoval 0",
  "compRangeBiasAndRxChanPhase 0.0 1.00000 0.00000 -1.00000 0.00000 1.00000 0.00000 -1.00000 0.00000 1.00000 0.00000 -1.00000 0.00000",
  "adcDataSource 0 adc_data_0001_CtestAdc6Ant.bin",
  "adcLogging 0",
  "lowPowerCfg 0",
  "factoryCalibCfg 1 0 40 0 0x1ff000",
  "sensorStart 0 0 0 0",
};
const int CFG_N = sizeof(CFG) / sizeof(CFG[0]);

// ---------- CLI ----------
bool sendCmd(const char* cmd, uint32_t timeoutMs = 1500) {
  while (Radar.available()) Radar.read();          // flush
  Radar.print(cmd);
  Radar.print("\n");
  String resp;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Radar.available()) resp += (char)Radar.read();
    if (resp.indexOf("Done") >= 0 || resp.indexOf("Error") >= 0 ||
        resp.indexOf("not recognized") >= 0) break;
  }
  bool ok = resp.indexOf("Done") >= 0;
  Serial.printf("# %s  %s\n", ok ? "[ok] " : "[ERR]", cmd);
  if (!ok) Serial.printf("#       reply: %s\n", resp.length() ? resp.c_str() : "(nothing)");
  return ok;
}

// ---------- Frame parser ----------
static const uint8_t MAGIC[8] = {0x02,0x01,0x04,0x03,0x06,0x05,0x08,0x07};
static const size_t HDR_LEN = 40;
static uint8_t buf[16384];
static size_t  len = 0, need = 0, mi = 0;
enum State { SYNC, HDR, BODY } st = SYNC;

static inline uint32_t rd32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline int16_t  rd16s(const uint8_t* p) { int16_t v; memcpy(&v, p, 2); return v; }
static inline float    rdf(const uint8_t* p)  { float v; memcpy(&v, p, 4); return v; }

#define MAX_PTS 64
struct Pt { float x, y, z, v, snr; };
static Pt pts[MAX_PTS];
static size_t npts = 0;

static void addPoint(float x, float y, float z, float v, float snr) {
  if (npts < MAX_PTS && isfinite(x) && isfinite(y) && isfinite(z) && isfinite(v))
    pts[npts++] = {x, y, z, v, snr};
}

void parseFrame(const uint8_t* f, size_t n) {
  uint32_t frameNum = rd32(f + 20);
  uint32_t numTlv   = rd32(f + 32);
  npts = 0;

  size_t off = HDR_LEN;
  for (uint32_t t = 0; t < numTlv; t++) {
    if (off + 8 > n) break;
    uint32_t type = rd32(f + off);
    uint32_t tlen = rd32(f + off + 4);
    off += 8;
    if (off + tlen > n) break;
    const uint8_t* p = f + off;

    if (type == 301 && tlen >= 20) {
      // Compressed point cloud: header floats xyzUnit, dopplerUnit, snrUnit, noiseUnit; uint16 num[2]
      // point: int16 x, y, z, doppler; uint8 snr, noise (10 bytes)
      float xyzU = rdf(p), dopU = rdf(p + 4), snrU = rdf(p + 8);
      size_t np = (tlen - 20) / 10;
      for (size_t i = 0; i < np; i++) {
        const uint8_t* q = p + 20 + i * 10;
        addPoint(rd16s(q) * xyzU, rd16s(q + 2) * xyzU, rd16s(q + 4) * xyzU,
                 rd16s(q + 6) * dopU, q[8] * snrU);
      }
    } else if (type == 1) {
      // Classic float point cloud: x, y, z, v (16 bytes each)
      size_t np = tlen / 16;
      for (size_t i = 0; i < np; i++) {
        const uint8_t* q = p + i * 16;
        addPoint(rdf(q), rdf(q + 4), rdf(q + 8), rdf(q + 12), 0);
      }
    }
    off += tlen;
  }

  Serial.printf("{\"f\":%lu,\"p\":[", (unsigned long)frameNum);
  for (size_t i = 0; i < npts; i++) {
    Serial.printf("%s[%.2f,%.2f,%.2f,%.2f,%.0f]", i ? "," : "",
                  pts[i].x, pts[i].y, pts[i].z, pts[i].v, pts[i].snr);
  }
  Serial.print("]}\n");
}

void pumpRadar() {
  while (Radar.available()) {
    uint8_t b = Radar.read();
    switch (st) {
      case SYNC:
        if (b == MAGIC[mi]) {
          buf[mi++] = b;
          if (mi == 8) { len = 8; st = HDR; }
        } else {
          mi = (b == MAGIC[0]) ? 1 : 0;
          if (mi) buf[0] = b;
        }
        break;
      case HDR:
        buf[len++] = b;
        if (len == HDR_LEN) {
          need = rd32(buf + 12);                    // totalPacketLen
          if (need < HDR_LEN || need > sizeof(buf)) { st = SYNC; mi = 0; }
          else if (need == HDR_LEN) { parseFrame(buf, len); st = SYNC; mi = 0; }
          else st = BODY;
        }
        break;
      case BODY:
        buf[len++] = b;
        if (len >= need) { parseFrame(buf, len); st = SYNC; mi = 0; }
        break;
    }
  }
}

// ---------- ToF (2x VL53L8CX, shared I2C) ----------
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;
static const int LP1_PIN = 5;
static const int LP2_PIN = 6;
// The ST library uses 8-bit addresses: 0x52 == 7-bit 0x29
static const uint16_t TOF_ADDR[2] = {0x2A << 1, 0x2B << 1};
static const uint8_t TOF_RES = VL53L8CX_RESOLUTION_8X8;
static const uint8_t TOF_HZ = 15;

VL53L8CX tof[2] = {VL53L8CX(&Wire, LP1_PIN), VL53L8CX(&Wire, LP2_PIN)};
bool tofOk[2] = {false, false};
VL53L8CX_ResultsData tofData;

// A failed sensor is skipped (no halt) so the radar keeps working.
bool bringUpTof(int i) {
  VL53L8CX &t = tof[i];
  t.begin();                       // pulses this sensor's LPn LOW->HIGH; default address until changed
  delay(10);
  Serial.printf("# TOF%d: uploading firmware...\n", i + 1);
  uint8_t status = t.init();
  if (status) { Serial.printf("# TOF%d init failed (status %u)\n", i + 1, status); return false; }
  status = t.set_i2c_address(TOF_ADDR[i]);
  if (status) { Serial.printf("# TOF%d set_i2c_address failed (status %u)\n", i + 1, status); return false; }
  Wire.beginTransmission(TOF_ADDR[i] >> 1);
  if (Wire.endTransmission() != 0) { Serial.printf("# TOF%d init failed (not found at 0x%02X)\n", i + 1, TOF_ADDR[i] >> 1); return false; }
  Serial.printf("# TOF%d ready at 0x%02X\n", i + 1, TOF_ADDR[i] >> 1);
  return true;                     // LPn stays HIGH: dropping it would revert to 0x29
}

bool configureTof(int i) {
  VL53L8CX &t = tof[i];
  uint8_t status = t.set_resolution(TOF_RES);
  if (!status) status = t.set_ranging_frequency_hz(TOF_HZ);
  if (!status) status = t.start_ranging();
  if (status) { Serial.printf("# TOF%d init failed (config status %u)\n", i + 1, status); return false; }
  Serial.printf("# TOF%d ranging\n", i + 1);
  return true;
}

void emitTof(int id, const VL53L8CX_ResultsData &r) {
  Serial.printf("{\"tof\":%d,\"d\":[", id);
  for (uint8_t z = 0; z < 64; z++) {
    uint8_t idx = VL53L8CX_NB_TARGET_PER_ZONE * z;
    bool valid = r.nb_target_detected[z] > 0 && (r.target_status[idx] == 5 || r.target_status[idx] == 9);
    Serial.printf("%s%d", z ? "," : "", valid ? (int)r.distance_mm[idx] : -1);
  }
  Serial.print("]}\n");
}

void pollTof(int i) {
  if (!tofOk[i]) return;
  uint8_t ready = 0;
  if (tof[i].check_data_ready(&ready) == 0 && ready && tof[i].get_ranging_data(&tofData) == 0) emitTof(i + 1, tofData);
}

// ---------- Main ----------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Radar.setRxBufferSize(8192);
  Radar.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(300);

#if BRIDGE_MODE
  Serial.println("Bridge mode: type CLI commands (e.g. 'version').");
#else
  // ToF first: its firmware upload takes seconds and the radar is not streaming yet.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  pinMode(LP1_PIN, OUTPUT);
  pinMode(LP2_PIN, OUTPUT);
  digitalWrite(LP1_PIN, LOW);      // hold both off so only one answers at 0x29 at a time
  digitalWrite(LP2_PIN, LOW);
  delay(20);
  for (int i = 0; i < 2; i++) tofOk[i] = bringUpTof(i);
  for (int i = 0; i < 2; i++) if (tofOk[i]) tofOk[i] = configureTof(i);

  Serial.println("\n# --- IWRL6432 bring-up ---");
  if (!sendCmd("version", 2000)) {
    Serial.println("# No reply to 'version'. Check: TX/RX not swapped, SOP0->3V3,");
    Serial.println("# SOP1->GND, EVM power-cycled, and that demo firmware is flashed.");
  }
  for (int i = 0; i < CFG_N; i++) {
    sendCmd(CFG[i]);
    delay(20);
  }
  Serial.println("# --- streaming ---");
#endif
}

void loop() {
#if BRIDGE_MODE
  while (Serial.available()) Radar.write(Serial.read());
  while (Radar.available())  Serial.write(Radar.read());
#else
  pumpRadar();
  pollTof(0);
  pollTof(1);
#endif
}

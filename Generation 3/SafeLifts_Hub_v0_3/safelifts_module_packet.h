/*
  safelifts_module_packet.h  --  module -> hub report, version 2

  History: v2 widened fault_flags for four ToF sensors per module (v1 was never
  built against by any firmware).

  Plain C++ (no Arduino or BLE dependencies) so the same file can be used on
  the hub and on the sensor-module firmware. If you change anything here, bump
  SL_PKT_VERSION and tell the whole team.

  Wire format: 15 bytes, fits in one default BLE payload (20 bytes).

    byte 0     version            (SL_PKT_VERSION)
    byte 1     module_id          0 = FRONT module, 1 = BACK module
    byte 2     seq                counts 0..255 then wraps
    byte 3     frame_period       send period in units of 10 ms (10 = every 100 ms), must be > 0
    byte 4     fault_flags        SL_FAULT_* bits (4 ToF sensors, radar, battery, degraded)
    byte 5     multi_mask         bit n = more than one object in sector n
    byte 6     moving_mask        bit n = radar says moving in sector n (reserved, send 0)
    byte 7..14 sector[0..7]       per-sector nearest distance, see below

  Sector numbering is lift-relative and matches the wristband:
    0 FRONT, 1 FRONT-RIGHT, 2 RIGHT, 3 BACK-RIGHT, 4 BACK, 5 BACK-LEFT, 6 LEFT, 7 FRONT-LEFT

  Sector byte values:
    0..250  nearest object in that sector, in cm
    0xFF    looked, nothing in range (CLEAR)
    0xFE    unknown (sensor error, saturated, no valid reading)
    0xFD    this module does not cover that sector
    other   reserved; the hub treats any other value as UNKNOWN (never as clear)

  RULE: never send 0xFF for a sector that no working sensor can currently see.
  Unknown is not clear.
*/

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define SL_PKT_VERSION   2
#define SL_PKT_LEN       15

enum {
  SL_MODULE_FRONT = 0,
  SL_MODULE_BACK  = 1,
  SL_NUM_MODULES  = 2
};

enum {
  SL_SEC_FRONT       = 0,
  SL_SEC_FRONT_RIGHT = 1,
  SL_SEC_RIGHT       = 2,
  SL_SEC_BACK_RIGHT  = 3,
  SL_SEC_BACK        = 4,
  SL_SEC_BACK_LEFT   = 5,
  SL_SEC_LEFT        = 6,
  SL_SEC_FRONT_LEFT  = 7,
  SL_NUM_SECTORS     = 8
};

#define SL_DIST_MAX_CM       250
#define SL_DIST_NOT_COVERED  0xFD
#define SL_DIST_UNKNOWN      0xFE
#define SL_DIST_CLEAR        0xFF

#define SL_FAULT_TOF_A       0x01
#define SL_FAULT_TOF_B       0x02
#define SL_FAULT_TOF_C       0x04
#define SL_FAULT_TOF_D       0x08
#define SL_FAULT_RADAR       0x10
#define SL_FAULT_BATT_LOW    0x20
#define SL_FAULT_DEGRADED    0x40
// bit 7 reserved
#define SL_FAULT_SENSOR_MASK (SL_FAULT_TOF_A | SL_FAULT_TOF_B | SL_FAULT_TOF_C | \
                              SL_FAULT_TOF_D | SL_FAULT_RADAR)

/*
  Which sectors each module is meant to cover (bit n = sector n).
  Placement: FRONT module at front center of the basket, BACK module at back
  center, each looking outward over 180 degrees. Sectors are 45 degrees wide
  and centered on 0, 45, 90, ..., so a 180 degree view covers three sectors
  fully and the two side sectors (RIGHT, LEFT) only by half. The front module
  sees the front half of RIGHT/LEFT, the back module the back half, so the hub
  treats both modules as covering those two sectors.

  Update this table if the real sensor coverage turns out to differ.
*/
static const uint8_t SL_MODULE_COVERAGE[SL_NUM_MODULES] = {
  /* FRONT module: LEFT, FRONT-LEFT, FRONT, FRONT-RIGHT, RIGHT */
  (uint8_t)((1u << SL_SEC_LEFT) | (1u << SL_SEC_FRONT_LEFT) | (1u << SL_SEC_FRONT) |
            (1u << SL_SEC_FRONT_RIGHT) | (1u << SL_SEC_RIGHT)),
  /* BACK module: RIGHT, BACK-RIGHT, BACK, BACK-LEFT, LEFT */
  (uint8_t)((1u << SL_SEC_RIGHT) | (1u << SL_SEC_BACK_RIGHT) | (1u << SL_SEC_BACK) |
            (1u << SL_SEC_BACK_LEFT) | (1u << SL_SEC_LEFT))
};

struct SLModuleReport {
  uint8_t version;
  uint8_t module_id;
  uint8_t seq;
  uint8_t frame_period_10ms;
  uint8_t fault_flags;
  uint8_t multi_mask;
  uint8_t moving_mask;
  uint8_t sector[SL_NUM_SECTORS];
};

// Write r into out[SL_PKT_LEN]. The version byte is always SL_PKT_VERSION.
inline void slEncode(const SLModuleReport& r, uint8_t* out) {
  out[0] = SL_PKT_VERSION;
  out[1] = r.module_id;
  out[2] = r.seq;
  out[3] = r.frame_period_10ms;
  out[4] = r.fault_flags;
  out[5] = r.multi_mask;
  out[6] = r.moving_mask;
  for (int i = 0; i < SL_NUM_SECTORS; i++) out[7 + i] = r.sector[i];
}

// Parse a packet. Returns false (and leaves r untouched) if it is malformed.
inline bool slDecode(const uint8_t* in, size_t len, SLModuleReport& r) {
  if (in == NULL || len != SL_PKT_LEN) return false;
  if (in[0] != SL_PKT_VERSION) return false;
  if (in[1] >= SL_NUM_MODULES) return false;
  if (in[3] == 0) return false;                 // a zero period would make staleness meaningless
  r.version           = in[0];
  r.module_id         = in[1];
  r.seq               = in[2];
  r.frame_period_10ms = in[3];
  r.fault_flags       = in[4];
  r.multi_mask        = in[5];
  r.moving_mask       = in[6];
  for (int i = 0; i < SL_NUM_SECTORS; i++) r.sector[i] = in[7 + i];
  return true;
}

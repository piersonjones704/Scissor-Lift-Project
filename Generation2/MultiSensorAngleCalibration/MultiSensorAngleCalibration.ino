/*
  MultiSensorAngleCalibration.ino

  Purpose:
  1. Round-robin poll N VL53L8CX sensors through a TCA9548A/PCA9548A mux,
     tagging every reading with WHICH physical sensor/channel it came from.
  2. Provide a calibration-verification mode: place a single object in
     front of ONE sensor at a time and confirm the serial output reports
     it under the correct label/angle BEFORE trusting the angle table
     in later trig math.
*/

#include <Wire.h>
#include "vl53l8cx_api.h"

// sensor_init() and the LOG/LOGF macros come from power_test_common.h.
// NOTE: this header already #defines SDA_PIN, SCL_PIN, and I2C_FREQ (8, 9,
// 400000) -- do NOT redefine them here, or you'll get macro-redefinition
// warnings. sensor_init() also calls Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ)
// internally EVERY time it's called, so it'll silently re-run that 6 times
// during setup() below (once per sensor) -- harmless on ESP32, but it means
// this sketch should let power_test_common.h's Wire setup be the source of
// truth rather than configuring Wire separately with different values.
#include "power_test_common.h"

#define TCAADDR   0x70

#define FREQ_FAR 5 // matches AdaptiveProximity_1Sensor.ino

// Defined here (near the top) rather than near where it's used, because
// Arduino's IDE auto-generates function prototypes and inserts them very
// early in the file -- before any struct defined further down would be
// visible. correctToLiftFrame() returns Point3D, so Point3D must be
// declared before that auto-inserted prototype, or you'll get an error
// like "'Point3D' does not name a type".
// Declared here (near the top), same reason as Point3D above: Arduino's
// auto-generated prototype for zoneAngularOffsetDeg() needs this struct
// to already be visible, or you'll get "'AngleOffset' does not name a type".
struct Point3D { float x; float y; float z; };
struct AngleOffset { float dYawDeg; float dPitchDeg; };

// ---- SENSOR CONFIG TABLE -------------------------------------------------
// Confirmed 2x3 mount geometry (channel 4 intentionally unused -- 6 sensors
// on 6 channels: 0,1,2,3,5,6). Reference is channel 2 (Top Middle): mounted
// dead level and perpendicular to the lift side, 0 deg yaw, 0 deg pitch.
// Top row is level (pitch 0); bottom row is tilted 45 deg down (pitch -45).
// Each corner sensor is an INDEPENDENT 45 deg yaw AND 45 deg pitch rotation
// off the reference -- not a single diagonal tilt.

struct SensorConfig {
  uint8_t     channel;
  const char* label;
  float       mountYawDeg;
  float       mountPitchDeg;
};

SensorConfig sensors[] = {
  { 2, "Top-Middle (reference)",  0.0f,  0.0f },
  { 0, "Top-Right",               45.0f,  0.0f },
  { 5, "Top-Left",               -45.0f,  0.0f },
  { 3, "Bottom-Middle",            0.0f, -45.0f },
  { 1, "Bottom-Right",            45.0f, -45.0f },
  { 6, "Bottom-Left",            -45.0f, -45.0f },
};
const int NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// One Configuration + Results struct PER PHYSICAL SENSOR -- the ULD API
// tracks device state per struct instance, not just per I2C address, so
// each of the 6 sensors (even though they all answer at the same 0x29)
// needs its own entry here.
VL53L8CX_Configuration Dev[NUM_SENSORS];
VL53L8CX_ResultsData    Results[NUM_SENSORS];

// ── Zone selection hysteresis state (per sensor) ───────────────────────────
// Without this, a stationary object near the boundary of two zones can
// cause the "closest zone" to flicker between reads (from ordinary noise),
// which flickers the reported angle/z even though nothing moved. This
// mirrors the modeStabilityCount pattern already used in the single-sensor
// firmware's FAR/NEAR mode switching: require a challenger zone to win
// ZONE_STABILITY_COUNT consecutive cycles before it replaces the current
// "stable" zone.
#define ZONE_STABILITY_COUNT 2

int8_t  stableZone[NUM_SENSORS];    // last confirmed stable zone, -1 = none yet
int8_t  pendingZone[NUM_SENSORS];   // challenger zone currently being confirmed, -1 = none
uint8_t pendingCount[NUM_SENSORS];  // consecutive cycles the challenger has won

// ---- MODE SELECT ----------------------------------------------------------
// true  = calibration mode: slow, verbose, one sensor's reading highlighted
// false = normal mode: fast round-robin polling of all sensors
bool CALIBRATION_MODE = false;

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// ── Per-zone angular offset within a single sensor's FOV ──────────────────
// VL53L8CX has roughly a 45deg x 45deg field of view. At 4x4 resolution,
// each zone subtends FOV/4 = 11.25deg per axis.
//
// CONFIRMED EMPIRICALLY on the Top-Middle sensor (channel 2) using
// TopMiddleZoneOrientation.ino:
//   zone 12 (row=3, col=0) -> scene Top-Left
//   zone 0  (row=0, col=0) -> scene Top-Right
//   zone 3  (row=0, col=3) -> scene Bottom-Right
//   zone 15 (row=3, col=3) -> scene Bottom-Left
// This means COL determines top/bottom (col=0 -> top, col=3 -> bottom),
// and ROW determines left/right (row=0 -> right, row=3 -> left) -- the
// OPPOSITE pairing from the datasheet-derived guess (which had col->yaw,
// row->pitch). This matches the ~90 deg physical rotation of this
// breakout vs. the reference photo. The (1.5 - index) magnitude/sign
// formula itself, derived from ST's documented lens flip, is still
// correct -- only which axis (row vs col) it's applied to has changed.
//
// This applies to ALL 6 sensors: the zone-orientation flip is intrinsic
// to the sensor chip + how each board is physically mounted, and per the
// mount description, all 6 boards are mounted with identical orientation
// (only their overall yaw/pitch angle differs, not this internal
// rotation) -- so one confirmed test covers the whole array.
#define SENSOR_FOV_DEG      45.0f
#define ZONES_PER_ROW       4
#define ZONE_STEP_DEG       (SENSOR_FOV_DEG / ZONES_PER_ROW)

AngleOffset zoneAngularOffsetDeg(uint8_t zoneIndex) {
  uint8_t row = zoneIndex / ZONES_PER_ROW; // confirmed: row -> left/right
  uint8_t col = zoneIndex % ZONES_PER_ROW; // confirmed: col -> top/bottom

  AngleOffset off;
  off.dYawDeg   = (1.5f - (float)row) * ZONE_STEP_DEG; // row=0 -> right(+), row=3 -> left(-)
  off.dPitchDeg = (1.5f - (float)col) * ZONE_STEP_DEG; // col=0 -> up(+),    col=3 -> down(-)
  return off;
}

// Same filtering logic as your single-sensor get_closest_distance():
// status 5 = valid, status 6 = valid but lower confidence.
// Returns 0 if NO valid target was found in any of the 16 zones --
// this is "nothing detected," not "object at 0mm." Callers must check
// for > 0, not just >= 0, before treating it as a real reading.
// outZoneIndex is set to the zone that produced the closest reading,
// so the caller can look up that zone's angular offset within the FOV.
int16_t getClosestDistance(VL53L8CX_ResultsData &res, uint8_t &outZoneIndex) {
  int16_t closest = 0;
  outZoneIndex = 0;
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t status = res.target_status[i];
    int16_t dist   = res.distance_mm[i];
    if ((status == 5 || status == 6) && dist > 0) {
      if (closest == 0 || dist < closest) {
        closest = dist;
        outZoneIndex = i;
      }
    }
  }
  return closest;
}

// Converts a raw along-boresight distance into a 3D point in the lift's
// reference frame, using this sensor's known yaw AND pitch offset from
// the Top-Middle reference sensor. This is the multi-axis equivalent of
// Gen 1's trig correction (which only had to handle a single servo sweep
// angle in one plane) -- here every sensor can be off-axis in two planes
// at once, so distance must be projected along a full 3D boresight vector.
//
// Convention (matches the mount description): at yaw=0, pitch=0 the
// sensor points straight out from the lift side (+X), parallel to the
// ground. +yaw = rightward (as seen from the sensor's own POV looking
// outward). +pitch = up, so the tilted-down bottom row uses negative pitch.
//
// x = "straight out" from the lift side
// y = lateral (left/right along the side of the lift)
// z = vertical (up/down)
Point3D correctToLiftFrame(float rawDistanceMm, float mountYawDeg, float mountPitchDeg) {
  float yawRad   = mountYawDeg   * PI / 180.0f;
  float pitchRad = mountPitchDeg * PI / 180.0f;
  Point3D p;
  p.x = rawDistanceMm * cos(pitchRad) * cos(yawRad);
  p.y = rawDistanceMm * cos(pitchRad) * sin(yawRad);
  p.z = rawDistanceMm * sin(pitchRad);
  return p;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Wire.begin() here just gets the bus up so tcaSelect() can talk to the
  // mux before the first sensor_init() call. sensor_init() will re-run
  // Wire.begin() with these same values per sensor -- that's fine, this
  // just avoids talking to the mux on an unconfigured bus on the very
  // first call.
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);

  Serial.println();
  if (CALIBRATION_MODE) {
    Serial.println("=== CALIBRATION MODE ===");
    Serial.println("Place an object in front of ONE sensor at a time.");
    Serial.println("Confirm the label printed matches the sensor you placed it near.");
    Serial.println("Do this for every sensor before trusting mountYawDeg values above.");
  } else {
    Serial.println("=== NORMAL POLLING MODE ===");
  }
  Serial.println();

  Serial.println("Initializing sensors...");
  for (int i = 0; i < NUM_SENSORS; i++) {
    stableZone[i]   = -1;
    pendingZone[i]  = -1;
    pendingCount[i] = 0;

    tcaSelect(sensors[i].channel);
    delay(10);

    if (!sensor_init(&Dev[i])) {
      Serial.print("  FAILED to init ");
      Serial.print(sensors[i].label);
      Serial.print(" (channel ");
      Serial.print(sensors[i].channel);
      Serial.println(") -- skipping, will show as no-reading in the loop.");
      continue; // don't halt the whole array over one bad sensor during bring-up
    }

    vl53l8cx_set_resolution(&Dev[i], VL53L8CX_RESOLUTION_4X4);
    delay(10);
    vl53l8cx_set_ranging_mode(&Dev[i], VL53L8CX_RANGING_MODE_AUTONOMOUS);
    delay(10);
    vl53l8cx_set_ranging_frequency_hz(&Dev[i], FREQ_FAR);
    delay(10);
    vl53l8cx_set_target_order(&Dev[i], VL53L8CX_TARGET_ORDER_CLOSEST);
    delay(10);
    vl53l8cx_start_ranging(&Dev[i]);
    delay(10);

    Serial.print("  OK: ");
    Serial.println(sensors[i].label);
  }
  Serial.println("Done initializing. Starting polling loop.");
  Serial.println();
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(sensors[i].channel);
    delay(2); // let the mux settle before talking to the selected sensor

    uint8_t ready = 0;
    vl53l8cx_check_data_ready(&Dev[i], &ready);

    int16_t dist = -1; // -1 = no fresh data this pass
    int8_t  winningZone = -1; // -1 = no valid zone this pass, for debugging which zone fired
    float effectiveYawDeg   = sensors[i].mountYawDeg;
    float effectivePitchDeg = sensors[i].mountPitchDeg;

    if (ready) {
      vl53l8cx_get_ranging_data(&Dev[i], &Results[i]);
      uint8_t rawZone = 0;
      int16_t rawClosest = getClosestDistance(Results[i], rawZone);

      if (rawClosest > 0) {
        // --- Zone hysteresis ---
        // Only let the "stable" zone change after a challenger wins
        // ZONE_STABILITY_COUNT consecutive cycles. The very first
        // detection is accepted immediately (no delay) since a slow
        // FIRST detection is worse for a safety system than a stable
        // one arriving a cycle or two later.
        if (stableZone[i] == -1) {
          stableZone[i]   = rawZone;
          pendingZone[i]  = -1;
          pendingCount[i] = 0;
        } else if (rawZone != stableZone[i]) {
          if (rawZone == pendingZone[i]) {
            pendingCount[i]++;
          } else {
            pendingZone[i]  = rawZone;
            pendingCount[i] = 1;
          }
          if (pendingCount[i] >= ZONE_STABILITY_COUNT) {
            stableZone[i]   = rawZone;
            pendingZone[i]  = -1;
            pendingCount[i] = 0;
          }
        } else {
          // Raw winner agrees with the current stable zone -- any
          // challenger that was building up loses its progress.
          pendingZone[i]  = -1;
          pendingCount[i] = 0;
        }

        // Report distance/angle from the STABLE zone, not necessarily
        // this instant's raw winner, so distance and angle stay
        // self-consistent. If the stable zone itself has gone invalid
        // this cycle (object actually moved away from it), fall back to
        // the raw closest zone immediately rather than reporting stale
        // or wrong data, and resync tracking to that new zone.
        uint8_t sz       = stableZone[i];
        uint8_t szStatus = Results[i].target_status[sz];
        int16_t szDist   = Results[i].distance_mm[sz];
        bool stableZoneValid = (szStatus == 5 || szStatus == 6) && szDist > 0;

        if (stableZoneValid) {
          dist        = szDist;
          winningZone = sz;
        } else {
          dist        = rawClosest;
          winningZone = rawZone;
          stableZone[i]   = rawZone;
          pendingZone[i]  = -1;
          pendingCount[i] = 0;
        }

        // Combine the sensor's overall mounting angle with the specific
        // zone's offset within that sensor's own FOV. Simple addition is
        // an approximation (true compound rotation would use rotation
        // matrices), but it's accurate enough at these angle magnitudes
        // and is a major improvement over ignoring zone position entirely.
        AngleOffset zoneOff = zoneAngularOffsetDeg(winningZone);
        effectiveYawDeg   += zoneOff.dYawDeg;
        effectivePitchDeg += zoneOff.dPitchDeg;
      } else {
        // No valid target anywhere this cycle -- reset tracking so the
        // next detection is treated as fresh rather than compared
        // against a stale stable zone from a previous, unrelated object.
        stableZone[i]   = -1;
        pendingZone[i]  = -1;
        pendingCount[i] = 0;
      }
    }

    if (CALIBRATION_MODE) {
      // Verbose per-sensor printout so you can visually confirm, while
      // walking around the rig with an object, that the RIGHT label
      // lights up when you're in front of that physical sensor.
      Serial.print("Ch ");
      Serial.print(sensors[i].channel);
      Serial.print(" [");
      Serial.print(sensors[i].label);
      Serial.print("]  yaw=");
      Serial.print(sensors[i].mountYawDeg);
      Serial.print("deg  pitch=");
      Serial.print(sensors[i].mountPitchDeg);
      Serial.print("deg  raw_dist=");
      if (dist < 0) {
        Serial.print("no object / not ready");
      } else {
        Serial.print(dist);
        Serial.print("mm");
        if (dist < 300) { // arbitrary "something is close" threshold for calibration
          Serial.print("   <<< OBJECT DETECTED HERE");
        }
      }
      Serial.println();
    } else {
      // Normal mode: tag + transform each reading, ready for downstream
      // fusion / clearance calculation against the lift's actual geometry.
      if (dist > 0) {
        Point3D p = correctToLiftFrame((float)dist, effectiveYawDeg, effectivePitchDeg);
        Serial.print(sensors[i].label);
        Serial.print(": zone=");
        Serial.print(winningZone);
        Serial.print("  raw=");
        Serial.print(dist);
        Serial.print("mm  lift_x=");
        Serial.print(p.x, 1);
        Serial.print("  lift_y=");
        Serial.print(p.y, 1);
        Serial.print("  lift_z=");
        Serial.println(p.z, 1);
      }
    }
  }

  if (CALIBRATION_MODE) {
    Serial.println("---");
    delay(3500); // slow cadence so you have time to move the object and read output
  } else {
    delay(3500); // faster cadence for normal operation
  }
}
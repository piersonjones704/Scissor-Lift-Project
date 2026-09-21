/*
  TopMiddleZoneOrientation.ino

  Purpose: determine the VL53L8CX's actual zone index -> physical
  direction mapping empirically, using ONLY the Top-Middle sensor
  (channel 2 on the mux). Since the lens flip is intrinsic to the sensor
  chip itself, this result applies to all 6 sensors -- only their
  separate mounting angle (yaw/pitch) differs, not their internal zone
  orientation.

  How to use:
  1. Upload this sketch.
  2. Point the Top-Middle sensor at open space / a flat wall.
  3. Hold a small object at the FAR-LEFT edge of what the sensor can
     see (from your own point of view, standing where the sensor looks
     outward). Note which zone id(s) show a short distance.
  4. Repeat for far-right, top, and bottom edges.
  5. Compare against the predicted ranges below (from ST's documented
     lens flip):
       - Object at scene LEFT   -> expect zone id % 4 == 3  (ids 3,7,11,15)
       - Object at scene RIGHT  -> expect zone id % 4 == 0  (ids 0,4,8,12)
       - Object at scene TOP    -> expect zone id / 4 == 0  (ids 0,1,2,3)
       - Object at scene BOTTOM -> expect zone id / 4 == 3  (ids 12,13,14,15)
     If your results don't match this, that tells us exactly how much
     extra rotation this sensor's physical mounting adds beyond the
     documented intrinsic flip.
*/

#include <Wire.h>
#include "vl53l8cx_api.h"
#include "power_test_common.h" // provides sensor_init(), SDA_PIN, SCL_PIN, I2C_FREQ

#define TCAADDR        0x70
#define TOP_MIDDLE_CH  2   // channel for the Top-Middle sensor
#define FREQ_FAR       5   // matches your single-sensor firmware

VL53L8CX_Configuration Dev;
VL53L8CX_ResultsData    Results;

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);

  tcaSelect(TOP_MIDDLE_CH);
  delay(10);

  Serial.println("Initializing Top-Middle sensor (channel 2)...");
  if (!sensor_init(&Dev)) {
    Serial.println("FAILED to init sensor. Check wiring/channel.");
    while (1) delay(1000);
  }

  vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
  delay(10);
  vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
  delay(10);
  vl53l8cx_set_ranging_frequency_hz(&Dev, FREQ_FAR);
  delay(10);
  vl53l8cx_start_ranging(&Dev);
  delay(10);

  Serial.println("Ready. Move an object to each edge of the sensor's view.");
  Serial.println();
}

void loop() {
  tcaSelect(TOP_MIDDLE_CH);
  delay(2);

  uint8_t ready = 0;
  vl53l8cx_check_data_ready(&Dev, &ready);
  if (!ready) {
    delay(20);
    return;
  }

  vl53l8cx_get_ranging_data(&Dev, &Results);

  // Print the full 4x4 grid, zone ids in natural id/4, id%4 order.
  // This is just a readable printout -- it does NOT attempt to
  // pre-orient itself to "real world" left/right/top/bottom. Compare
  // whatever id you see against the predicted ranges in the header
  // comment above.
  int16_t closestDist = 0;
  uint8_t closestZone = 0;

  for (int8_t row = 3; row >= 0; row--) { // print id/4=3 group first, then down to 0
    for (uint8_t col = 0; col < 4; col++) {
      uint8_t id = row * 4 + col;
      uint8_t status = Results.target_status[id];
      int16_t dist   = (Results.distance_mm[id]);
      bool valid = (status == 5 || status == 6) && dist > 0;

      char buf[16];
      if (valid) {
        snprintf(buf, sizeof(buf), "[%2u]%4dmm", id, dist);
        if (closestDist == 0 || dist < closestDist) {
          closestDist = dist;
          closestZone = id;
        }
      } else {
        snprintf(buf, sizeof(buf), "[%2u]  ----", id);
      }
      Serial.print(buf);
      Serial.print("  ");
    }
    Serial.println();
  }

  if (closestDist > 0) {
    Serial.print(">>> Closest: zone ");
    Serial.print(closestZone);
    Serial.print(" (row=");
    Serial.print(closestZone / 4);
    Serial.print(", col=");
    Serial.print(closestZone % 4);
    Serial.print(") at ");
    Serial.print(closestDist);
    Serial.println("mm");
  } else {
    Serial.println(">>> No valid target in any zone");
  }
  Serial.println("---");

  delay(1000);
}
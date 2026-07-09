#include <Wire.h>
#include <vl53l8cx.h>

#define SDA_PIN  8
#define SCL_PIN  9
#define LP_PIN   1

VL53L8CX* sensor;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // pinMode(LP_PIN, OUTPUT);
  // digitalWrite(LP_PIN, LOW);
  // delay(100);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // digitalWrite(LP_PIN, HIGH);
  // delay(200);

  sensor = new VL53L8CX(&Wire, -1);
  sensor->begin();

  uint8_t isAlive = 0;
  sensor->is_alive(&isAlive);
  Serial.printf("is_alive: %u\n", isAlive);

  if (isAlive) {
    sensor->init();
    sensor->set_resolution(VL53L8CX_RESOLUTION_4X4);
    sensor->set_ranging_frequency_hz(10);
    sensor->start_ranging();
    Serial.println("Sensor OK, ranging started.");
  } else {
    Serial.println("Sensor not found.");
  }
}

void loop() {
  uint8_t ready = 0;
  sensor->check_data_ready(&ready);
  if (!ready) return;

  VL53L8CX_ResultsData results;
  sensor->get_ranging_data(&results);

  uint16_t minDist = 0xFFFF;
  bool found = false;
  for (uint8_t z = 0; z < 16; z++) {
    if (results.nb_target_detected[z] > 0) {
      uint16_t d = results.distance_mm[z * VL53L8CX_NB_TARGET_PER_ZONE];
      if (d > 0 && d < minDist) { minDist = d; found = true; }
    }
  }

  if (found) Serial.printf("Distance: %u mm\n", minDist);
  else Serial.println("No target");

  delay(100);
}
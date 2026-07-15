#include "arduino_secrets.h"

// Two-channel ultrasonic, servo, speaker (single Arduino) with shared TRIG pin
// Shared TRIG = 10
// Sensor1: ECHO1 = 11, SERVO1 = 12, SPEAKER1 = 13
// Sensor2: ECHO2 = 3,  SERVO2 = 4,  SPEAKER2 = 5
// Place 330 Î© resistor in series with each speaker +

#include <Servo.h>

// --- configuration ---
const int TRIG_SHARED = 10;
const int ECHO1 = 11;
const int SERVO1 = 12;
const int SPEAKER1 = 13;
const int ECHO2 = 3;
const int SERVO2 = 4;
const int SPEAKER2 = 5;

// --- optional battery monitor ---
#define ENABLE_BATT_MON true
const int BATTERY_SENSE_PIN = A0;      // analog input via voltage divider
const float VDIV_FACTOR = 47.0 / 147.0; // Rtop=100k, Rbot=47k
const float LOW_BATT_THRESHOLD_V = 4.5; // warning threshold

// --- behavior parameters ---
const int MAX_DETECT_CM = 30;
const unsigned long MEASURE_INTERVAL = 120;
const unsigned long SERVO_INTERVAL   = 40;
const int SERVO_STEP = 2;

// --- servo state ---
Servo servoA, servoB;
int posA = 90, posB = 90;
int dirA = 1, dirB = 1;
unsigned long lastServoMove = 0;
unsigned long lastMeasureTime = 0;
bool toggleMeasure = false;

// ---------- helpers ----------
float readBatteryVoltage() {
  int adc = analogRead(BATTERY_SENSE_PIN);
  float v = (adc * (5.0 / 1023.0)) / VDIV_FACTOR; // convert back to battery volts
  return v;
}

int measureDistanceSharedTrig(int echoPin) {
  digitalWrite(TRIG_SHARED, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_SHARED, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_SHARED, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return 999;
  return (int)((duration / 58.0) + 0.5);
}

void softBeep(int speakerPin, int distCm) {
  const int minD = 2;
  const int maxD = MAX_DETECT_CM;
  distCm = constrain(distCm, minD, maxD);
  int freq = map(distCm, minD, maxD, 1200, 600);
  int onMs = map(distCm, minD, maxD, 20, 120);
  tone(speakerPin, freq);
  delay(onMs);
  noTone(speakerPin);
  delay(map(distCm, minD, maxD, 40, 160));
}
// ---------- end helpers ----------

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_SHARED, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(SPEAKER1, OUTPUT);
  pinMode(SPEAKER2, OUTPUT);

  servoA.attach(SERVO1);
  servoB.attach(SERVO2);
  servoA.write(posA);
  servoB.write(posB);

  digitalWrite(TRIG_SHARED, LOW);
  delay(300); // let power stabilize
  Serial.println("Two-channel shared-TRIG system ready");

  if (ENABLE_BATT_MON) {
    float vb = readBatteryVoltage();
    Serial.print("Battery voltage at startup: ");
    Serial.println(vb, 2);
    if (vb < LOW_BATT_THRESHOLD_V) {
      Serial.println("â ï¸  Warning: Battery voltage is low!");
    }
  }
}

void loop() {
  unsigned long now = millis();

  // servo sweep
  if (now - lastServoMove >= SERVO_INTERVAL) {
    lastServoMove = now;
    posA += dirA * SERVO_STEP;
    if (posA >= 160) { posA = 160; dirA = -1; }
    if (posA <= 20)  { posA = 20;  dirA = 1; }
    servoA.write(posA);

    posB += dirB * SERVO_STEP;
    if (posB >= 160) { posB = 160; dirB = -1; }
    if (posB <= 20)  { posB = 20;  dirB = 1; }
    servoB.write(posB);
  }

  // ultrasonic measurements
  if (now - lastMeasureTime >= MEASURE_INTERVAL) {
    lastMeasureTime = now;

    if (toggleMeasure) {
      int d2 = measureDistanceSharedTrig(ECHO2);
      Serial.print("Sensor 2: "); Serial.println(d2);
      if (d2 != 999 && d2 <= MAX_DETECT_CM) softBeep(SPEAKER2, d2);
    } else {
      int d1 = measureDistanceSharedTrig(ECHO1);
      Serial.print("Sensor 1: "); Serial.println(d1);
      if (d1 != 999 && d1 <= MAX_DETECT_CM) softBeep(SPEAKER1, d1);
    }
    toggleMeasure = !toggleMeasure;
  }

  // optional periodic battery check every ~10s
  static unsigned long lastBattCheck = 0;
  if (ENABLE_BATT_MON && now - lastBattCheck >= 10000UL) {
    lastBattCheck = now;
    float vb = readBatteryVoltage();
    Serial.print("Battery voltage: "); Serial.println(vb, 2);
    if (vb < LOW_BATT_THRESHOLD_V) {
      Serial.println("â ï¸ Battery low!");
    }
  }
}

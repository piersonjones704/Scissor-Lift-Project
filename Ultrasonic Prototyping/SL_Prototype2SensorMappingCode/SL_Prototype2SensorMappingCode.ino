#include "arduino_secrets.h"

// Two-channel ultrasonic + servos + non-blocking speaker beeps + serial telemetry
// Shared TRIG = 10
// Sensor1: ECHO1 = 11, SERVO1 = 12, SPEAKER1 = 13
// Sensor2: ECHO2 = 3,  SERVO2 = 4,  SPEAKER2 = 5

#include <Servo.h>

// pins
const int TRIG_SHARED = 10;
const int ECHO1 = 11;
const int ECHO2 = 3;
const int SERVO1 = 12;
const int SERVO2 = 4;
const int SPEAKER1 = 13;
const int SPEAKER2 = 5;

// behavior / timing
const int MAX_DETECT_CM = 30;
const unsigned long MEASURE_INTERVAL = 120; // ms
const unsigned long SERVO_INTERVAL = 40;    // ms
const int SERVO_STEP = 2;
const int A_MIN = 20, A_MAX = 160;

// servo state
Servo servoA, servoB;
int posA = 90, posB = 90;
int dirA = 1, dirB = 1;
unsigned long lastServoMove = 0;
unsigned long lastMeasure = 0;
bool toggleMeasure = false;

// beep state (non-blocking)
struct BeepState {
  bool on;                 // currently sounding
  unsigned long lastStart; // when current ON started
  unsigned long lastTrig;  // when interval last started
  unsigned long onMs;      // how long to hold current ON
  unsigned long interval;  // how long between beep starts
  int freq;                // tone freq for current distance
};
BeepState b1 = {false,0,0,0,0,0};
BeepState b2 = {false,0,0,0,0,0};

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
  delay(200);
  Serial.println("READY");
}

// measure single echo pin using the shared TRIG (returns cm, 999 = out)
int measureDistanceSharedTrig(int echoPin) {
  digitalWrite(TRIG_SHARED, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_SHARED, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_SHARED, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout
  if (duration == 0) return 999;
  return (int)((duration / 58.0) + 0.5);
}

// map distance to beep parameters (closer => faster, shorter pulses & higher freq)
void updateBeepParams(BeepState &b, int distCm) {
  const int minD = 2;
  const int maxD = MAX_DETECT_CM;
  distCm = constrain(distCm, minD, maxD);
  b.freq = map(distCm, minD, maxD, 1200, 600);      // closer => higher freq
  b.onMs = (unsigned long)map(distCm, minD, maxD, 20, 120);   // closer => short beep
  b.interval = (unsigned long)map(distCm, minD, maxD, 60, 400); // closer => faster repeats
}

// non-blocking beep control: call frequently in loop()
void handleBeep(BeepState &b, int speakerPin, unsigned long now) {
  if (!b.on) {
    // not sounding: check if it's time to start next beep
    if (now - b.lastTrig >= b.interval) {
      // start beep
      tone(speakerPin, b.freq);
      b.on = true;
      b.lastStart = now;
      b.lastTrig = now; // count interval from start
    }
  } else {
    // currently sounding: stop if on-duration elapsed
    if (now - b.lastStart >= b.onMs) {
      noTone(speakerPin);
      b.on = false;
      // lastTrig already set at start; next start will wait interval
    }
  }
}

void loop() {
  unsigned long now = millis();

  // servo sweep (non-blocking)
  if (now - lastServoMove >= SERVO_INTERVAL) {
    lastServoMove = now;
    posA += dirA * SERVO_STEP;
    if (posA >= A_MAX) { posA = A_MAX; dirA = -1; }
    if (posA <= A_MIN) { posA = A_MIN; dirA = 1; }
    servoA.write(posA);

    posB += dirB * SERVO_STEP;
    if (posB >= A_MAX) { posB = A_MAX; dirB = -1; }
    if (posB <= A_MIN) { posB = A_MIN; dirB = 1; }
    servoB.write(posB);
  }

  // measurement + update beep params + CSV output
  if (now - lastMeasure >= MEASURE_INTERVAL) {
    lastMeasure = now;

    // measure sensor 1, then sensor 2 (sequential to avoid cross-talk)
    int d1 = measureDistanceSharedTrig(ECHO1);
    delay(30); // small pause so echoes die away before next sensor
    int d2 = measureDistanceSharedTrig(ECHO2);

    // update beep params only when in range; else treat as out-of-range
    if (d1 != 999 && d1 <= MAX_DETECT_CM) {
      updateBeepParams(b1, d1);
      // ensure we can start soon if needed: if never triggered, set lastTrig to allow immediate start
      if (b1.interval == 0) b1.lastTrig = now - b1.interval;
    } else {
      // out of range -> stop any ongoing beep immediately
      if (b1.on) { noTone(SPEAKER1); b1.on = false; }
      // set interval to a large value so it doesn't try to beep
      b1.interval = 1000000UL;
      b1.lastTrig = now;
    }

    if (d2 != 999 && d2 <= MAX_DETECT_CM) {
      updateBeepParams(b2, d2);
      if (b2.interval == 0) b2.lastTrig = now - b2.interval;
    } else {
      if (b2.on) { noTone(SPEAKER2); b2.on = false; }
      b2.interval = 1000000UL;
      b2.lastTrig = now;
    }

    // print telemetry CSV: d1,d2,a1,a2
    Serial.print(d1); Serial.print(',');
    Serial.print(d2); Serial.print(',');
    Serial.print(posA); Serial.print(',');
    Serial.println(posB);
  }

  // handle beeps non-blocking
  handleBeep(b1, SPEAKER1, now);
  handleBeep(b2, SPEAKER2, now);

  // loop continues quickly (pulseIn blocks briefly during measurements only)
}

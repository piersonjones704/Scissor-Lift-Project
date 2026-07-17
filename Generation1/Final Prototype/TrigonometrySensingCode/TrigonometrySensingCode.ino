// Two-channel ultrasonic, servo, speaker — closest-sensor beeper with hysteresis
#include <Servo.h>

// --- pins & hardware ---
const int TRIG1 = 11;
const int ECHO1 = 12;
const int TRIG2 = 9;
const int ECHO2 = 10;
const int SERVO_PIN = 8;
const int SPEAKER = 7;

// servo pulse limits (set conservatively; based on servo datasheet)
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const float US_PER_DEG = float(SERVO_MAX_US - SERVO_MIN_US) / 270.0f;

// servo rotation constants
const int POS_MIN = 105;
const int POS_MAX = 270;
int pos = (POS_MIN + POS_MAX) / 2;  // starts at 135°
int dir = 1;

// --- geometry for distance correction ---
// 1930ES dimensions: 30" wide (76cm), 73.5" long (187cm)
const float SENSOR_OFFSET_CM = 118.0;  // design offset for trig calculations
const float LIFT_SHORT_SIDE_CM = 38.0; // half-width of lift (76cm / 2)
const float LIFT_LONG_SIDE_CM = 93.5;  // half-length of lift (187cm / 2)

// --- power monitor (optional) ---
#define ENABLE_BATT_MON true
const int BATTERY_SENSE_PIN = A0;
const float VDIV_FACTOR = 47.0 / 147.0;
const float LOW_BATT_THRESHOLD_V = 4.5;

// --- behavior params ---
const int MAX_DETECT_CM = 30;           // detection threshold for detaching servo
const unsigned long MEASURE_INTERVAL = 120; // ms between measurement cycles
const unsigned long SERVO_INTERVAL   = 10;  // ms between servo steps
const int SERVO_STEP = 2;               // degrees per step

const unsigned long TRIG_GAP_MS = 40;   // gap between trig pulses (reduce crosstalk)
const int HYSTERESIS_COUNT = 3;         // require this many consecutive readings to switch active sensor
const int CONTINUOUS_TONE_THRESHOLD = 5; // cm — closer than this = continuous tone

// --- state ---
Servo servo;
bool servoDetached = false;
unsigned long lastServoMove = 0;
unsigned long lastMeasureTime = 0;
int lastD1 = 999;
int lastD2 = 999;

// beep state (non-blocking)
bool beepOn = false;
unsigned long beepNextToggleMs = 0;
int beepFreq = 0;
int beepOnMs = 0;
int beepOffMs = 0;

// fade-out control
bool fadingOut = false;
int fadeStep = 0;
unsigned long fadeNextMs = 0;
const int FADE_STEPS = 4;          // number of fade beeps
const unsigned long FADE_STEP_MS = 100; // delay between fade beeps

// hysteresis state for selecting active sensor
int currentClosest = 0;     // 0 = none, 1 = sensor1, 2 = sensor2
int candidateClosest = 0;
int candidateCount = 0;

//////////////// helpers /////////////////////
float readBatteryVoltage() {
  int adc = analogRead(BATTERY_SENSE_PIN);
  float v = (adc * (5.0 / 1023.0)) / VDIV_FACTOR;
  return v;
}

int measureDistanceWithTrig(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout
  if (duration == 0) return 999;
  return (int)((duration / 58.0) + 0.5);
}

// Calculate actual distance from scissor lift body based on sensor geometry
// Sensors mounted at FRONT RIGHT corner
// At 135°: servo points diagonally forward-right
// At 90°: servo points to the right (along short side)
// At 180°: servo points straight forward (along long axis)
// sensor: 1 = SSR (covers right/short side), 2 = LSL (covers front/long side)
// angle: servo angle in degrees (135° = center, diagonal)
// sensorDist: raw distance measured by sensor
int calculateLiftDistance(int sensor, int angle, int sensorDist) {
  // Handle invalid sensor readings
  if (sensorDist >= 999) return 999;
  
  float angleRad = angle * DEG_TO_RAD;
  float liftDistance;
  
  // SSR (Sensor 1): Short Side Right, Long Side Left
  // Covers angles pointing toward the RIGHT SIDE (short side)
  if (sensor == 1) {
    if (angle >= 25 && angle < 75) {
      // 25-75°: angled toward right side/corner, add offset
      liftDistance = LIFT_SHORT_SIDE_CM + SENSOR_OFFSET_CM * cos((90 - angle) * DEG_TO_RAD) + sensorDist;
    }
    else if (angle >= 75 && angle < 135) {
      // 75-135°: transitioning from right side to diagonal front-right
      liftDistance = LIFT_SHORT_SIDE_CM + SENSOR_OFFSET_CM * sin((angle - 45) * DEG_TO_RAD) + sensorDist;
    }
    else if (angle >= 135 && angle <= 225) {
      // 135-225°: pointing forward/left/back, use raw distance (away from right side)
      liftDistance = sensorDist;
    }
    else {
      // 225-245°: pointing back toward right rear
      liftDistance = LIFT_SHORT_SIDE_CM + sensorDist;
    }
  }
  // LSL (Sensor 2): Long Side Left, Short Side Right  
  // Covers angles pointing toward the FRONT/LONG SIDE
  else {
    if (angle >= 25 && angle < 75) {
      // 25-75°: angled toward right, away from front
      liftDistance = sensorDist;
    }
    else if (angle >= 75 && angle < 135) {
      // 75-135°: transitioning toward front
      liftDistance = LIFT_LONG_SIDE_CM + SENSOR_OFFSET_CM * cos((angle - 45) * DEG_TO_RAD) + sensorDist;
    }
    else if (angle >= 135 && angle <= 225) {
      // 135-225°: pointing forward to left to back, measure from front/rear edge
      if (angle <= 180) {
        liftDistance = sensorDist;  // front side
      } else {
        liftDistance = sensorDist;  // rear side
      }
    }
    else {
      // 225-245°: pointing toward right rear
      liftDistance = SENSOR_OFFSET_CM + sensorDist;
    }
  }
  
  return (int)(liftDistance + 0.5);  // round to nearest integer
}

// map distance to beep parameters
void distanceToBeepParams(int distCm, int &freq, int &onMs, int &offMs) {
  const int minD = 2;
  const int maxD = MAX_DETECT_CM;
  distCm = constrain(distCm, minD, maxD);
  freq = map(distCm, minD, maxD, 1400, 600);   // closer => higher pitch
  onMs = map(distCm, minD, maxD, 30, 140);     // closer => shorter on
  offMs = map(distCm, minD, maxD, 40, 260);    // closer => shorter off
}
//////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(SPEAKER, OUTPUT);
  noTone(SPEAKER);

  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);

  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  int pw = int(SERVO_MIN_US + pos * US_PER_DEG);
  pw = constrain(pw, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(pw);

  Serial.println("Closest-sensor beeper with geometry correction ready");
  Serial.println("1930ES Scissor Lift - Front Right Corner Mount");
  Serial.println("Sweep: 25° to 245° (220° range, centered at 135°)");
  Serial.print("Sensor offset: "); Serial.print(SENSOR_OFFSET_CM); Serial.println(" cm");
  Serial.print("Lift short side: "); Serial.print(LIFT_SHORT_SIDE_CM); Serial.println(" cm");
  Serial.print("Lift long side: "); Serial.print(LIFT_LONG_SIDE_CM); Serial.println(" cm");
  
  if (ENABLE_BATT_MON) {
    float vb = readBatteryVoltage();
    Serial.print("Battery V at startup: ");
    Serial.println(vb, 2);
    if (vb < LOW_BATT_THRESHOLD_V) Serial.println("⚠ Battery low");
  }
}

void loop() {
  unsigned long now = millis();

  // ============================================================
  // MEASUREMENT AND DECISION LOGIC (every MEASURE_INTERVAL)
  // ============================================================
  if (now - lastMeasureTime >= MEASURE_INTERVAL) {
    lastMeasureTime = now;

    // Actually measure both sensors with staggered triggers
    int rawD1 = measureDistanceWithTrig(TRIG1, ECHO1);
    delay(TRIG_GAP_MS);  // prevent crosstalk
    int rawD2 = measureDistanceWithTrig(TRIG2, ECHO2);

    // Apply geometry correction to get distance from lift body
    lastD1 = calculateLiftDistance(1, pos, rawD1);
    lastD2 = calculateLiftDistance(2, pos, rawD2);

    Serial.print("Angle: "); Serial.print(pos); Serial.print("° | ");
    Serial.print("Raw D1: "); Serial.print(rawD1); Serial.print(" → Lift D1: "); Serial.print(lastD1);
    Serial.print(" cm | Raw D2: "); Serial.print(rawD2); Serial.print(" → Lift D2: "); Serial.print(lastD2);
    Serial.println(" cm");

    // Compute the "closest" but only if <= MAX_DETECT_CM
    int closest = 999;
    if (lastD1 != 999 && lastD1 <= MAX_DETECT_CM) closest = lastD1;
    if (lastD2 != 999 && lastD2 <= MAX_DETECT_CM && lastD2 < closest) closest = lastD2;

    // If nothing valid --> immediate silence
    if (closest == 999) {
      Serial.println("DEBUG: No object detected, silencing");
      
      // Stop any tone immediately and reset ALL beep state
      noTone(SPEAKER);
      beepOn = false;
      beepFreq = 0;
      beepOnMs = 0;
      beepOffMs = 0;
      beepNextToggleMs = 0;

      // DON'T start fade-out, just stay silent
      fadingOut = false;
      fadeStep = 0;
      
      // Reset hysteresis state
      candidateClosest = 0;
      currentClosest = 0;
      candidateCount = 0;
    }
    else {
      // We have a valid closest reading <= MAX_DETECT_CM
      if (fadingOut) {
        fadingOut = false;
        fadeStep = 0;
        Serial.println("Object returned — cancel fade-out");
      }

      // Determine which sensor is the active candidate (1 or 2)
      if (lastD1 != 999 && lastD2 != 999) {
        candidateClosest = (lastD1 <= lastD2) ? 1 : 2;
      } else if (lastD1 != 999) {
        candidateClosest = 1;
      } else {
        candidateClosest = 2;
      }

      // Hysteresis commit
      if (candidateClosest != currentClosest) {
        candidateCount++;
        if (candidateCount >= HYSTERESIS_COUNT) {
          currentClosest = candidateClosest;
          candidateCount = 0;
          Serial.print("Switched to sensor "); Serial.println(currentClosest);
        }
      } else {
        candidateCount = 0;
      }

      // After hysteresis, compute beep params from active sensor
      if (currentClosest > 0) {
        int activeDist = (currentClosest == 1) ? lastD1 : lastD2;
        int desiredFreq, desiredOn, desiredOff;
        distanceToBeepParams(activeDist, desiredFreq, desiredOn, desiredOff);

        // Continuous tone if very close
        if (activeDist <= CONTINUOUS_TONE_THRESHOLD) {
          if (beepNextToggleMs != 0 || desiredFreq != beepFreq) {
            noTone(SPEAKER);
            tone(SPEAKER, desiredFreq);
            beepFreq = desiredFreq;
            beepOn = true;
            beepNextToggleMs = 0;  // Disable toggle
            Serial.print("Steady tone @ "); Serial.println(desiredFreq);
          }
        } else {
          // Normal beeper: only update params, don't restart cycle unless params changed
          if (desiredFreq != beepFreq || desiredOn != beepOnMs || desiredOff != beepOffMs) {
            beepFreq = desiredFreq;
            beepOnMs = desiredOn;
            beepOffMs = desiredOff;
            
            // Only restart if we weren't already beeping, or if we were in continuous mode
            if (beepNextToggleMs == 0) {
              tone(SPEAKER, beepFreq);
              beepOn = true;
              beepNextToggleMs = millis() + beepOnMs;
              Serial.println("Starting beep cycle");
            }
            // Otherwise let the existing cycle continue with new params
          }
        }
      }
    }

    // Detach/reattach servo based on any close object
    bool objectClose = ((lastD1 != 999 && lastD1 <= MAX_DETECT_CM) ||
                        (lastD2 != 999 && lastD2 <= MAX_DETECT_CM));
    if (objectClose && !servoDetached) {
      Serial.println("Detaching servo (object close)");
      servo.detach();
      servoDetached = true;
    } else if (!objectClose && servoDetached) {
      Serial.println("Reattaching servo (clear)");
      servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
      int pw = int(SERVO_MIN_US + pos * US_PER_DEG);
      pw = constrain(pw, SERVO_MIN_US, SERVO_MAX_US);
      servo.writeMicroseconds(pw);
      servoDetached = false;
    }
  }

  // ============================================================
  // CONTINUOUS TASKS (run every loop)
  // ============================================================

  // Non-blocking beep toggle (only when not fading and beepNextToggleMs is set)
  if (!fadingOut && beepFreq > 0 && beepNextToggleMs > 0 && now >= beepNextToggleMs) {
    if (beepOn) {
      noTone(SPEAKER);
      beepOn = false;
      beepNextToggleMs = now + beepOffMs;
    } else {
      tone(SPEAKER, beepFreq);
      beepOn = true;
      beepNextToggleMs = now + beepOnMs;
    }
  }

  // Fade-out handler
  if (fadingOut && now >= fadeNextMs) {
    if (fadeStep < FADE_STEPS) {
      int fadeFreq = 1200 - fadeStep * 200;
      tone(SPEAKER, fadeFreq, FADE_STEP_MS - 20);
      fadeNextMs = now + FADE_STEP_MS;
      fadeStep++;
    } else {
      noTone(SPEAKER);
      beepOn = false;
      fadingOut = false;
      fadeStep = 0;
      Serial.println("Fade-out complete → silent");
    }
  }

  // Servo sweep (only when attached)
  if (!servoDetached && (now - lastServoMove >= SERVO_INTERVAL)) {
    lastServoMove = now;
    pos += dir * SERVO_STEP;
    if (pos >= POS_MAX) { pos = POS_MAX; dir = -1; }
    if (pos <= POS_MIN) { pos = POS_MIN; dir = 1; }
    int pw = int(SERVO_MIN_US + pos * US_PER_DEG);
    pw = constrain(pw, SERVO_MIN_US, SERVO_MAX_US);
    servo.writeMicroseconds(pw);
  }

  // Battery monitoring
  static unsigned long lastBattCheck = 0;
  if (ENABLE_BATT_MON && (now - lastBattCheck >= 10000UL)) {
    lastBattCheck = now;
    float vb = readBatteryVoltage();
    Serial.print("Battery V: ");
    Serial.println(vb, 2);
    if (vb < LOW_BATT_THRESHOLD_V) Serial.println("⚠ Battery low");
  }
}
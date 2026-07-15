// Dual-channel ultrasonic radar with TABLE-EDGE detection - OPTIMIZED
// Designed for corner-mount on rectangular table (~118cm x 60cm)
#include <Servo.h>

// --- Hardware pins ---
const int TRIG1 = 10;
const int ECHO1 = 11;
const int TRIG2 = 2;
const int ECHO2 = 3;
const int SERVO_PIN = 12;
const int SPEAKER = 13;

// --- Servo configuration ---
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const float US_PER_DEG = float(SERVO_MAX_US - SERVO_MIN_US) / 270.0f;

// Servo sweep range (degrees) - adjusted for table coverage
const int POS_MIN = 30;   // Start at 30° for table edge
const int POS_MAX = 240;  // End at 240° for table edge
int pos = POS_MIN;
int dir = 1;  // 1 = forward, -1 = backward

// --- Behavior parameters ---
const int SERVO_STEP_DELAY = 8;   // REALLY FAST: 8ms (was 12ms, originally 30ms)
const int TRIG_GAP_MS = 15;       // REDUCED: 15ms between sensors (was 30ms)
const int HYSTERESIS_COUNT = 2;   
const int CONTINUOUS_TONE_THRESHOLD = 5;

// --- Power monitoring (optional) ---
#define ENABLE_BATT_MON false
const int BATTERY_SENSE_PIN = A0;
const float VDIV_FACTOR = 47.0 / 147.0;
const float LOW_BATT_THRESHOLD_V = 4.5;

// --- Table geometry constants ---
const float TABLE_LONG_SIDE = 118.0;  // cm
const float TABLE_SHORT_SIDE = 60.0;  // cm

// --- State variables ---
Servo servo;
int lastD1 = 999;
int lastD2 = 999;

// Beep state (non-blocking)
bool beepOn = false;
unsigned long beepNextToggleMs = 0;
int beepFreq = 0;
int beepOnMs = 0;
int beepOffMs = 0;

// Hysteresis for sensor selection
int currentClosest = 0;
int candidateClosest = 0;
int candidateCount = 0;

bool fadingOut = false;

//////////////// Helper Functions /////////////////////

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
  
  unsigned long duration = pulseIn(echoPin, HIGH, 20000UL); // 20ms timeout (faster)
  if (duration == 0) return 999;
  
  int distance = (int)((duration * 0.034) / 2.0 + 0.5);
  return distance;
}

// Calculate table-edge detection threshold based on servo angle
float getTableEdgeThreshold(int angle) {
  float threshold;
  
  if (angle >= 30 && angle <= 90) {
    // Zone 1: 30° to 90° - scanning along 118cm side
    // Formula: 118 * sin(θ)
    float radians = angle * PI / 180.0;
    threshold = TABLE_LONG_SIDE * sin(radians);
  }
  else if (angle > 90 && angle <= 180) {
    // Zone 2: 90° to 180° - scanning along 60cm perpendicular side
    threshold = TABLE_SHORT_SIDE;
  }
  else if (angle > 180 && angle <= 240) {
    // Zone 3: 180° to 240° - scanning along 118cm side (mirrored)
    int mirrorAngle = 180 - (angle - 180);
    float radians = mirrorAngle * PI / 180.0;
    threshold = TABLE_LONG_SIDE * sin(radians);
  }
  else {
    threshold = TABLE_SHORT_SIDE;
  }
  
  return threshold;
}

// Beeping based on DISTANCE FROM TABLE EDGE (not raw distance)
void distanceToBeepParams(float distFromEdge, int &freq, int &onMs, int &offMs) {
  // distFromEdge: how far the object is from falling off (smaller = more urgent)
  
  const int minD = 2;
  const int maxD = 60;
  int effectiveDist = constrain((int)distFromEdge, minD, maxD);
  
  // CONSTANT FREQUENCY - always loud and clear
  freq = 2000;
  
  // ORIGINAL AGGRESSIVE TIMING - based on distance from edge
  // Closer to edge = faster beeping
  onMs = map(effectiveDist, minD, maxD, 30, 140);
  offMs = map(effectiveDist, minD, maxD, 40, 260);
}

// Handle beeping logic
void updateBeeper(int closestDist, int currentAngle) {
  unsigned long now = millis();
  
  float tableEdge = getTableEdgeThreshold(currentAngle);
  
  // Check if object is within table boundary
  if (closestDist == 999 || closestDist >= tableEdge) {
    // No object or object is at/beyond table edge - silence
    noTone(SPEAKER);
    beepOn = false;
    beepFreq = 0;
    beepOnMs = 0;
    beepOffMs = 0;
    beepNextToggleMs = 0;
    fadingOut = false;
    return;
  }
  
  // Calculate how far object is from table edge
  float distFromEdge = tableEdge - closestDist;
  
  // Object detected within table boundary
  int desiredFreq, desiredOn, desiredOff;
  distanceToBeepParams(distFromEdge, desiredFreq, desiredOn, desiredOff);
  
  if (desiredFreq == 0) {
    noTone(SPEAKER);
    beepOn = false;
    beepNextToggleMs = 0;
    return;
  }
  
  // Continuous tone when very close to edge
  if (distFromEdge <= CONTINUOUS_TONE_THRESHOLD) {
    if (beepNextToggleMs != 0 || desiredFreq != beepFreq) {
      noTone(SPEAKER);
      tone(SPEAKER, desiredFreq);
      beepFreq = desiredFreq;
      beepOn = true;
      beepNextToggleMs = 0;
    }
  } else {
    // Normal beeping pattern
    if (desiredFreq != beepFreq || desiredOn != beepOnMs || desiredOff != beepOffMs) {
      beepFreq = desiredFreq;
      beepOnMs = desiredOn;
      beepOffMs = desiredOff;
      
      if (beepNextToggleMs == 0) {
        tone(SPEAKER, beepFreq);
        beepOn = true;
        beepNextToggleMs = now + beepOnMs;
      }
    }
    
    // Non-blocking beep toggle
    if (beepNextToggleMs > 0 && now >= beepNextToggleMs) {
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
  }
}

// Update hysteresis logic for sensor selection
void updateHysteresis() {
  if (lastD1 != 999 && lastD2 != 999) {
    candidateClosest = (lastD1 <= lastD2) ? 1 : 2;
  } else if (lastD1 != 999) {
    candidateClosest = 1;
  } else if (lastD2 != 999) {
    candidateClosest = 2;
  } else {
    candidateClosest = 0;
  }
  
  if (candidateClosest != currentClosest) {
    candidateCount++;
    if (candidateCount >= HYSTERESIS_COUNT) {
      currentClosest = candidateClosest;
      candidateCount = 0;
    }
  } else {
    candidateCount = 0;
  }
}

////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(SPEAKER, OUTPUT);
  
  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);
  noTone(SPEAKER);
  
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  int pw = int(SERVO_MIN_US + pos * US_PER_DEG);
  pw = constrain(pw, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(pw);
  
  delay(200);
}

void loop() {
  // Move servo one step
  pos += dir * 1;
  
  // Reverse direction at limits
  if (pos >= POS_MAX) {
    pos = POS_MAX;
    dir = -1;
  }
  if (pos <= POS_MIN) {
    pos = POS_MIN;
    dir = 1;
  }
  
  // Update servo position
  int pw = int(SERVO_MIN_US + pos * US_PER_DEG);
  pw = constrain(pw, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(pw);
  
  delay(SERVO_STEP_DELAY);  // 8ms (FAST!)
  
  // Measure both sensors with staggered triggers
  lastD1 = measureDistanceWithTrig(TRIG1, ECHO1);
  delay(TRIG_GAP_MS);  // 15ms gap
  lastD2 = measureDistanceWithTrig(TRIG2, ECHO2);
  
  // Calculate table edge threshold for current angle
  float tableEdge = getTableEdgeThreshold(pos);
  
  // Send data to Processing: angle,distance1,distance2,tableEdgeThreshold.
  Serial.print(pos);
  Serial.print(",");
  Serial.print(lastD1);
  Serial.print(",");
  Serial.print(lastD2);
  Serial.print(",");
  Serial.print((int)tableEdge);
  Serial.print(".");
  
  // Calculate closest distance within table boundary
  int closest = 999;
  if (lastD1 != 999 && lastD1 < tableEdge) closest = lastD1;
  if (lastD2 != 999 && lastD2 < tableEdge && lastD2 < closest) closest = lastD2;
  
  // Update hysteresis
  updateHysteresis();
  
  // Get active sensor distance
  int activeDist = 999;
  if (currentClosest == 1) activeDist = lastD1;
  else if (currentClosest == 2) activeDist = lastD2;
  else activeDist = closest;
  
  // Update beeper
  updateBeeper(activeDist, pos);
}

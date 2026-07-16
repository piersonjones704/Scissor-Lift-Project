// ===== SCISSOR LIFT SAFETY SENSOR (Self-mask + Adaptive Sweep) =====
// VL53L0X + 270° Servo + Proximity Buzzer
// Serial out: angle_deg,mm,x_mm,y_mm   (visualizer accepts 2 or 4 columns)

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X tof;
Servo scanServo;

// ---------------- USER CONFIG ----------------
// Pins
const int SERVO_PIN   = 7;
const int BUZZER_PIN  = 8;

// Servo calibration (270° servo)
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Sweep geometry
const float TOTAL_FOV_DEG = 270.0;
const float TOF_FOV_DEG   = 25.0;
const float CENTER_DEG    = 135.0;   // 0° right, 90° forward

// FAST vs SLOW profile
const float DEG_STEP_FAST       = 2.0;
const uint16_t SETTLE_MS_FAST   = 10;

const float DEG_STEP_SLOW       = 0.8;   // finer stepping when near obstacle
const uint16_t SETTLE_MS_SLOW   = 28;    // let servo damp and ToF stabilize

// How long we keep slow mode after a near hit (in "points")
const uint16_t SLOW_HOLD_POINTS = 20;   // ~ one short arc worth

// ToF configuration
const uint16_t TOF_TIMEOUT_MS     = 30;
const uint32_t TIMING_BUDGET_US   = 20000;           // 20 ms
const uint16_t IM_INTERVAL_MS     = TIMING_BUDGET_US / 1000;

// Safety distances
const int ALERT_DISTANCE_MM = 200;   // 20 cm
const int MIN_DISTANCE_MM   = 30;    // clamp for pitch mapping

// Lift rectangular danger zone (mm), matches Python overlay
const float RECT_X_MIN = -200;
const float RECT_X_MAX =  800;
const float RECT_Y_MIN = -100;
const float RECT_Y_MAX =  600;

// ---- Self-mask (ignore the lift corner where the sensor sits) ----
// Treat any return inside the interior quadrant (x>=0,y>=0) and within this
// radius as a self-hit and ignore it (no beep, no slow, send -1 to host).
const int SELF_MASK_RADIUS_MM = 350;     // tune ~ 250–400 depending on mount
// ---------------------------------------------------------------

// Derived sweep bounds (keep ToF beam inside 270°)
const float HALF_TOTAL   = TOTAL_FOV_DEG * 0.5f;
const float HALF_TOF_FOV = TOF_FOV_DEG * 0.5f;
const float SWEEP_START_DEG = (CENTER_DEG - HALF_TOTAL) + HALF_TOF_FOV;
const float SWEEP_END_DEG   = (CENTER_DEG + HALF_TOTAL) - HALF_TOF_FOV;

// ---------------- INTERNAL STATE ----------------
float currentAngle = SWEEP_START_DEG;
int   sweepDir     = +1;       // +1 forward, -1 backward
uint16_t slowHold  = 0;        // countdown for slow mode

// ---------------------------------------------------------------
// Helpers
int angle270ToMicros(float angleDeg) {
  angleDeg = constrain(angleDeg, 0, 270);
  float t = angleDeg / 270.0f;
  return SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US);
}
inline void gotoAngle(float ang) { scanServo.writeMicroseconds(angle270ToMicros(ang)); }

inline bool inRect(float x, float y) {
  return (x >= RECT_X_MIN && x <= RECT_X_MAX &&
          y >= RECT_Y_MIN && y <= RECT_Y_MAX);
}
inline bool isSelfHit(float x, float y, uint16_t mm) {
  return (x >= 0 && y >= 0 && mm <= SELF_MASK_RADIUS_MM);
}

uint16_t readToFmm() { return tof.readRangeContinuousMillimeters(); }

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(BUZZER_PIN, OUTPUT);
  scanServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  if (!tof.init()) {
    Serial.println("ERR:init");
    while (1);
  }
  tof.setTimeout(TOF_TIMEOUT_MS);
  tof.setSignalRateLimit(0.25f);
  tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
  tof.startContinuous(IM_INTERVAL_MS);

  gotoAngle(currentAngle);
  delay(200);
  Serial.println("=== Scissor Lift Safety System Active (Self-mask + Adaptive Sweep) ===");
}

// ---------------------------------------------------------------
void loop() {
  // Choose motion profile based on slowHold
  const bool slowMode = (slowHold > 0);
  const float step    = slowMode ? DEG_STEP_SLOW     : DEG_STEP_FAST;
  const uint16_t wait = slowMode ? SETTLE_MS_SLOW    : SETTLE_MS_FAST;

  // Move
  gotoAngle(currentAngle);
  delay(wait);

  // Read
  uint16_t mm = readToFmm();
  if (!tof.timeoutOccurred() && mm > 0 && mm < 4000) {
    // Polar -> Cartesian (mm)
    float rad = radians(currentAngle);
    float x = mm * cos(rad);
    float y = mm * sin(rad);

    // Self-mask: ignore anything that is our own corner within the radius.
    if (isSelfHit(x, y, mm)) {
      // Tell visualizer to clear the bin at this angle
      Serial.print(currentAngle, 1);
      Serial.println(",-1");
      noTone(BUZZER_PIN);
    } else {
      // Normal publish (angle,mm,x,y)
      Serial.print(currentAngle, 1); Serial.print(",");
      Serial.print(mm);              Serial.print(",");
      Serial.print(x, 0);            Serial.print(",");
      Serial.println(y, 0);

      // If inside rectangular danger zone and closer than threshold → beep + slow
      if (inRect(x, y) && mm <= ALERT_DISTANCE_MM) {
        proximityBeep(mm);
        slowHold = SLOW_HOLD_POINTS;             // enter/refresh slow mode
      } else {
        noTone(BUZZER_PIN);
        if (slowHold > 0) slowHold--;            // decay slow mode when clear
      }
    }
  } else {
    // bad read: clear dot at this angle and decay slowHold
    Serial.print(currentAngle, 1);
    Serial.println(",-1");
    noTone(BUZZER_PIN);
    if (slowHold > 0) slowHold--;
  }

  // Advance angle with current step & direction
  currentAngle += sweepDir * step;
  if (currentAngle >= SWEEP_END_DEG)  { currentAngle = SWEEP_END_DEG;  sweepDir = -1; }
  if (currentAngle <= SWEEP_START_DEG){ currentAngle = SWEEP_START_DEG; sweepDir = +1; }
}

// ---------------------------------------------------------------
void proximityBeep(uint16_t distance) {
  // Pitch: closer = higher
  int freq = map(distance, ALERT_DISTANCE_MM, MIN_DISTANCE_MM, 450, 2500);
  freq = constrain(freq, 450, 2500);
  // Ping cadence: closer = faster
  int pingMs = map(distance, ALERT_DISTANCE_MM, MIN_DISTANCE_MM, 280, 60);
  pingMs = constrain(pingMs, 60, 300);
  tone(BUZZER_PIN, freq, 40);
  delay(pingMs);
}

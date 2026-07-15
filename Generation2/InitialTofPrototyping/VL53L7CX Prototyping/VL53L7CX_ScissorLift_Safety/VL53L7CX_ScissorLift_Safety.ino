// ===== SCISSOR LIFT SAFETY SENSOR - VL53L7CX (8x8 Multi-Zone) =====
// ESP32 + VL53L7CX ToF + 270° Servo + Piezo Buzzer
// Corner module with adaptive sweep and multi-zone vertical detection
// Serial output: angle_deg,zone_id,mm,x_mm,y_mm,d_eff

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <ESP32Servo.h>

SparkFun_VL53L5CX sensor;
Servo scanServo;

// ===================== USER CONFIG =====================

// --- Hardware Pins (ESP32) ---
const int SERVO_PIN   = 25;
const int BUZZER_PIN  = 26;
const int SDA_PIN     = 21;
const int SCL_PIN     = 22;

// --- Servo Calibration (270° servo) ---
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_FREQ_HZ = 50;

// --- Sweep Geometry (Corner Module: 0° to 270°) ---
const float TOTAL_FOV_DEG = 270.0;
const float CENTER_DEG    = 135.0;     // Mechanical center
const float SWEEP_START_DEG = 0.0;     // Right side
const float SWEEP_END_DEG   = 270.0;   // Left side

// --- Adaptive Sweep Profiles ---
// FAST: Normal scanning when clear
const float DEG_STEP_FAST     = 3.0;   // Larger steps = faster sweep
const uint16_t SETTLE_MS_FAST = 15;    // Minimal settle time

// SLOW: High-resolution scanning when threat detected
const float DEG_STEP_SLOW     = 1.0;   // Fine angular resolution
const uint16_t SETTLE_MS_SLOW = 35;    // Allow servo + sensor to stabilize

// How long to maintain slow mode after threat clears (in measurement points)
const uint16_t SLOW_HOLD_POINTS = 25;  // ~1 arc worth of points

// --- VL53L7CX Configuration ---
const uint16_t TOF_TIMEOUT_MS = 50;
const uint8_t RANGING_FREQ_HZ = 15;    // 15Hz = ~67ms per reading
const uint8_t RESOLUTION = 64;         // 8x8 zones (64 total)

// --- Safety Thresholds ---
const float BASE_THRESHOLD_MM = 200.0;  // 20cm base threshold for scissor lift
const float MIN_DISTANCE_MM   = 50.0;   // Minimum for mapping (5cm)
const float MAX_RANGE_MM      = 3500.0; // Sensor max range

// --- Trigonometric Scaling Parameters ---
const float SIN_CLAMP_MIN = 0.25;      // Prevent explosive thresholds near 0°
const float DEADBAND_DEG  = 5.0;       // No scaling in ±5° deadband for crisp straight-ahead

// --- Multi-Zone Vertical Bands (8x8 grid) ---
// Zones are numbered 0-63, where:
// Row 0 = bottom, Row 7 = top
// Col 0 = left, Col 7 = right (from sensor perspective)

// Define which rows to monitor for different threat levels
const uint8_t GENERAL_ROWS_START = 1;  // Rows 1-6 for general detection
const uint8_t GENERAL_ROWS_END   = 6;
const uint8_t OVERHEAD_ROWS_START = 5; // Rows 5-7 for overhead detection
const uint8_t OVERHEAD_ROWS_END   = 7;
const uint8_t LOW_ROWS_START = 0;      // Rows 0-2 for low objects
const uint8_t LOW_ROWS_END   = 2;

// --- Self-Masking (Ignore sensor's own mount structure) ---
const float SELF_MASK_X_MIN = 0.0;     // Positive X (sensor's right)
const float SELF_MASK_X_MAX = 300.0;   // 30cm to the right
const float SELF_MASK_Y_MIN = 0.0;     // Positive Y (sensor's forward)
const float SELF_MASK_Y_MAX = 300.0;   // 30cm forward
const float SELF_MASK_RADIUS_MM = 350.0; // Additional radial mask

// --- Buzzer Configuration ---
const int BUZZER_FREQ_MIN = 800;       // Minimum frequency (Hz) at 20cm
const int BUZZER_FREQ_MAX = 3500;      // Maximum frequency (Hz) at 0cm
const int BUZZER_DURATION_MS = 50;     // Duration of each beep
const int BUZZER_CADENCE_MIN_MS = 50;  // Fastest beep rate (very close)
const int BUZZER_CADENCE_MAX_MS = 300; // Slowest beep rate (at threshold)

// ===================== INTERNAL STATE =====================

float currentAngle = SWEEP_START_DEG;
int sweepDir = +1;                     // +1 = increasing angle, -1 = decreasing
uint16_t slowHold = 0;                 // Counter for maintaining slow mode

VL53L5CX_ResultsData measurementData;
bool sensorReady = false;

unsigned long lastBeepTime = 0;
int currentBeepCadence = BUZZER_CADENCE_MAX_MS;

// ===================== HELPER FUNCTIONS =====================

// Convert 270° angle to servo microseconds
int angle270ToMicros(float angleDeg) {
  angleDeg = constrain(angleDeg, 0.0, 270.0);
  float t = angleDeg / 270.0;
  return SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US);
}

// Move servo to specified angle
inline void gotoAngle(float ang) {
  scanServo.writeMicroseconds(angle270ToMicros(ang));
}

// Check if point is within self-mask zone (sensor's own structure)
bool isSelfMasked(float x, float y, float distance) {
  // Rectangular mask
  if (x >= SELF_MASK_X_MIN && x <= SELF_MASK_X_MAX &&
      y >= SELF_MASK_Y_MIN && y <= SELF_MASK_Y_MAX) {
    return true;
  }
  
  // Radial mask (for corner mounting structure)
  if (x >= 0 && y >= 0 && distance <= SELF_MASK_RADIUS_MM) {
    return true;
  }
  
  return false;
}

// Calculate effective distance with trigonometric scaling (CORNER MODULE)
// This implements the piecewise function for 270° corner coverage
float calculateEffectiveDistance(float angleDeg, float measuredDist) {
  float theta = angleDeg;
  float dEff = measuredDist;
  
  // Normalize angle to 0-270 range
  while (theta < 0) theta += 360.0;
  while (theta >= 360.0) theta -= 360.0;
  
  if (theta >= 0 && theta <= 90) {
    // First quadrant (0° to 90°): scale by |sin(θ)|
    // Looking along right edge - scale the threshold
    float sinTheta = abs(sin(radians(theta)));
    
    // Apply deadband to avoid extreme scaling near 0°
    if (abs(theta) < DEADBAND_DEG) {
      dEff = measuredDist;  // No scaling in deadband
    } else {
      // Clamp sin to prevent division issues
      sinTheta = max(sinTheta, SIN_CLAMP_MIN);
      dEff = measuredDist * sinTheta;
    }
    
  } else if (theta > 90 && theta <= 180) {
    // Second quadrant (90° to 180°): no scaling
    // Looking straight out from corner
    dEff = measuredDist;
    
  } else if (theta > 180 && theta <= 270) {
    // Third quadrant (180° to 270°): scale by |sin(θ)|
    // Looking along left edge - scale the threshold
    float sinTheta = abs(sin(radians(theta)));
    
    // Apply deadband near 180° (straight back)
    float thetaFrom180 = abs(theta - 180.0);
    if (thetaFrom180 < DEADBAND_DEG) {
      dEff = measuredDist;  // No scaling in deadband
    } else {
      sinTheta = max(sinTheta, SIN_CLAMP_MIN);
      dEff = measuredDist * sinTheta;
    }
  }
  
  return dEff;
}

// Check if any zone in specified vertical band has threat
bool checkVerticalBand(uint8_t rowStart, uint8_t rowEnd, float angleDeg, 
                       float& closestDist, uint8_t& closestZone) {
  bool threatDetected = false;
  closestDist = MAX_RANGE_MM;
  closestZone = 255;
  
  for (uint8_t row = rowStart; row <= rowEnd; row++) {
    for (uint8_t col = 0; col < 8; col++) {
      uint8_t zoneIdx = row * 8 + col;
      
      int16_t distMm = measurementData.distance_mm[zoneIdx];
      uint8_t status = measurementData.target_status[zoneIdx];
      
      // Valid reading check (status 5 or 9 are good)
      if (status != 5 && status != 9) continue;
      if (distMm <= 0 || distMm > MAX_RANGE_MM) continue;
      
      // Convert to Cartesian coordinates
      float rad = radians(currentAngle);
      float x = distMm * cos(rad);
      float y = distMm * sin(rad);
      
      // Check self-masking
      if (isSelfMasked(x, y, distMm)) continue;
      
      // Calculate effective distance with trigonometric scaling
      float dEff = calculateEffectiveDistance(angleDeg, distMm);
      
      // Check against threshold
      if (dEff <= BASE_THRESHOLD_MM) {
        threatDetected = true;
        if (distMm < closestDist) {
          closestDist = distMm;
          closestZone = zoneIdx;
        }
      }
    }
  }
  
  return threatDetected;
}

// Generate proximity alert with dynamic frequency and cadence
void proximityAlert(float distance) {
  unsigned long now = millis();
  
  // Calculate frequency: closer = higher pitch
  int freq = map(distance, BASE_THRESHOLD_MM, MIN_DISTANCE_MM, 
                 BUZZER_FREQ_MIN, BUZZER_FREQ_MAX);
  freq = constrain(freq, BUZZER_FREQ_MIN, BUZZER_FREQ_MAX);
  
  // Calculate cadence: closer = faster beeps
  int cadence = map(distance, BASE_THRESHOLD_MM, MIN_DISTANCE_MM,
                    BUZZER_CADENCE_MAX_MS, BUZZER_CADENCE_MIN_MS);
  cadence = constrain(cadence, BUZZER_CADENCE_MIN_MS, BUZZER_CADENCE_MAX_MS);
  currentBeepCadence = cadence;
  
  // Time-based beeping
  if (now - lastBeepTime >= cadence) {
    tone(BUZZER_PIN, freq, BUZZER_DURATION_MS);
    lastBeepTime = now;
  }
}

// Stop buzzer
inline void silenceBuzzer() {
  noTone(BUZZER_PIN);
}

// Get zone row and column from index
inline uint8_t getRow(uint8_t zoneIdx) { return zoneIdx / 8; }
inline uint8_t getCol(uint8_t zoneIdx) { return zoneIdx % 8; }

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== VL53L7CX Scissor Lift Safety System ===");
  Serial.println("Corner Module | 270° Coverage | 8x8 Multi-Zone");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz Fast Mode
  
  // Initialize Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  silenceBuzzer();
  
  // Initialize Servo
  ESP32PWM::allocateTimer(0);
  scanServo.setPeriodHertz(SERVO_FREQ_HZ);
  scanServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  
  // Initialize VL53L7CX sensor
  Serial.println("Initializing VL53L7CX...");
  
  if (!sensor.begin()) {
    Serial.println("ERROR: VL53L7CX not detected!");
    Serial.println("Check wiring:");
    Serial.println("  VIN -> 3.3V");
    Serial.println("  GND -> GND");
    Serial.println("  SCL -> GPIO 22");
    Serial.println("  SDA -> GPIO 21");
    while (1) {
      tone(BUZZER_PIN, 1000, 100);
      delay(500);
    }
  }
  
  Serial.println("VL53L7CX detected!");
  
  // Configure sensor
  sensor.setResolution(RESOLUTION);  // 8x8 mode
  sensor.setRangingFrequency(RANGING_FREQ_HZ);
  sensor.setIntegrationTime(20);  // ms per zone
  
  // Start ranging
  sensor.startRanging();
  sensorReady = true;
  
  // Move to start position
  gotoAngle(currentAngle);
  delay(500);
  
  // Startup beep sequence
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1500, 100);
    delay(200);
  }
  
  Serial.println("System Active!");
  Serial.println("Format: angle,zone,dist_mm,x,y,d_eff,status");
  Serial.println("===========================================");
}

// ===================== MAIN LOOP =====================

void loop() {
  // Check if sensor has new data ready
  if (!sensorReady || !sensor.isDataReady()) {
    delay(5);
    return;
  }
  
  // Read all 64 zones
  if (!sensor.getRangingData(&measurementData)) {
    Serial.println("ERROR: Failed to get ranging data");
    delay(10);
    return;
  }
  
  // Determine sweep profile based on threat state
  bool slowMode = (slowHold > 0);
  float step = slowMode ? DEG_STEP_SLOW : DEG_STEP_FAST;
  uint16_t settle = slowMode ? SETTLE_MS_SLOW : SETTLE_MS_FAST;
  
  // Move servo to current angle
  gotoAngle(currentAngle);
  delay(settle);
  
  // Check vertical bands for threats
  float closestDist = MAX_RANGE_MM;
  uint8_t closestZone = 255;
  bool threatDetected = false;
  
  // Check general band (rows 1-6)
  bool generalThreat = checkVerticalBand(GENERAL_ROWS_START, GENERAL_ROWS_END, 
                                         currentAngle, closestDist, closestZone);
  
  // Check overhead band (rows 5-7)
  float overheadDist;
  uint8_t overheadZone;
  bool overheadThreat = checkVerticalBand(OVERHEAD_ROWS_START, OVERHEAD_ROWS_END,
                                          currentAngle, overheadDist, overheadZone);
  
  // Check low band (rows 0-2)
  float lowDist;
  uint8_t lowZone;
  bool lowThreat = checkVerticalBand(LOW_ROWS_START, LOW_ROWS_END,
                                     currentAngle, lowDist, lowZone);
  
  // Combine threat detection (OR logic)
  threatDetected = generalThreat || overheadThreat || lowThreat;
  
  // Update closest distance from all bands
  if (overheadThreat && overheadDist < closestDist) {
    closestDist = overheadDist;
    closestZone = overheadZone;
  }
  if (lowThreat && lowDist < closestDist) {
    closestDist = lowDist;
    closestZone = lowZone;
  }
  
  // Output data for closest detection
  if (threatDetected && closestZone != 255) {
    int16_t distMm = measurementData.distance_mm[closestZone];
    float rad = radians(currentAngle);
    float x = distMm * cos(rad);
    float y = distMm * sin(rad);
    float dEff = calculateEffectiveDistance(currentAngle, distMm);
    uint8_t status = measurementData.target_status[closestZone];
    
    // Send to serial for visualization
    Serial.print(currentAngle, 1);  Serial.print(",");
    Serial.print(closestZone);      Serial.print(",");
    Serial.print(distMm);           Serial.print(",");
    Serial.print(x, 0);             Serial.print(",");
    Serial.print(y, 0);             Serial.print(",");
    Serial.print(dEff, 1);          Serial.print(",");
    Serial.println(status);
    
    // Activate alarm
    proximityAlert(closestDist);
    
    // Enter/refresh slow mode
    slowHold = SLOW_HOLD_POINTS;
    
  } else {
    // No threat detected - clear visualization
    Serial.print(currentAngle, 1);
    Serial.println(",-1,0,0,0,0,0");
    
    silenceBuzzer();
    
    // Decay slow mode
    if (slowHold > 0) slowHold--;
  }
  
  // Advance sweep angle
  currentAngle += sweepDir * step;
  
  // Check sweep boundaries and reverse direction
  if (currentAngle >= SWEEP_END_DEG) {
    currentAngle = SWEEP_END_DEG;
    sweepDir = -1;
  }
  if (currentAngle <= SWEEP_START_DEG) {
    currentAngle = SWEEP_START_DEG;
    sweepDir = +1;
  }
}

# VL53L7CX Safety System - Configuration & Testing Guide

## Quick Start Configuration Profiles

### Profile 1: Conservative (High Safety Margin)
**Use case**: Indoor operation, high traffic areas, narrow aisles

```cpp
// Safety Thresholds
const float BASE_THRESHOLD_MM = 300.0;  // 30cm trigger distance
const float SIN_CLAMP_MIN = 0.30;       // More aggressive edge scaling
const float DEADBAND_DEG = 3.0;         // Tighter deadband

// Sweep Profiles
const float DEG_STEP_FAST = 2.5;        // Finer fast sweep
const uint16_t SETTLE_MS_FAST = 20;
const float DEG_STEP_SLOW = 0.8;        // Very fine slow sweep
const uint16_t SETTLE_MS_SLOW = 40;

// Buzzer
const int BUZZER_FREQ_MIN = 1000;       // Higher base frequency
const int BUZZER_CADENCE_MIN_MS = 40;   // Faster alarm
```

**Characteristics:**
- Earlier warnings
- More sensitive to edges
- Finer angular resolution
- More urgent alarm tones

---

### Profile 2: Balanced (Standard Operation)
**Use case**: General warehouse, moderate traffic, standard clearances

```cpp
// Safety Thresholds (DEFAULT)
const float BASE_THRESHOLD_MM = 200.0;  // 20cm trigger distance
const float SIN_CLAMP_MIN = 0.25;
const float DEADBAND_DEG = 5.0;

// Sweep Profiles
const float DEG_STEP_FAST = 3.0;
const uint16_t SETTLE_MS_FAST = 15;
const float DEG_STEP_SLOW = 1.0;
const uint16_t SETTLE_MS_SLOW = 35;

// Buzzer
const int BUZZER_FREQ_MIN = 800;
const int BUZZER_CADENCE_MIN_MS = 50;
```

**Characteristics:**
- Good balance of speed and safety
- Reasonable alarm urgency
- Suitable for most applications

---

### Profile 3: Performance (Fast Sweep)
**Use case**: Outdoor operation, open areas, experienced operators

```cpp
// Safety Thresholds
const float BASE_THRESHOLD_MM = 150.0;  // 15cm trigger distance
const float SIN_CLAMP_MIN = 0.20;       // Less aggressive edge scaling
const float DEADBAND_DEG = 7.0;         // Wider deadband

// Sweep Profiles
const float DEG_STEP_FAST = 4.0;        // Faster sweep
const uint16_t SETTLE_MS_FAST = 12;
const float DEG_STEP_SLOW = 1.5;        // Coarser slow mode
const uint16_t SETTLE_MS_SLOW = 30;

// Buzzer
const int BUZZER_FREQ_MIN = 600;        // Lower base frequency
const int BUZZER_CADENCE_MIN_MS = 60;   // Less aggressive
```

**Characteristics:**
- Faster sweep for better situational awareness
- Closer trigger threshold
- Less alarming (for experienced operators)

---

## Self-Masking Calibration Procedure

### Step 1: Initial Setup
1. Upload code with self-masking DISABLED:
   ```cpp
   // Temporarily comment out masking
   // if (isSelfMasked(x, y, distMm)) continue;
   ```

2. Mount sensor on scissor lift corner
3. Open Serial Monitor (115200 baud)

### Step 2: Data Collection
1. Run system and observe serial output
2. Note X,Y coordinates of false positives from lift structure
3. Record pattern over full 270° sweep

**Example false positive data:**
```
Angle 15°:  x=280, y=75  (corner post)
Angle 30°:  x=320, y=160 (edge rail)
Angle 45°:  x=350, y=350 (diagonal corner)
```

### Step 3: Calculate Mask Dimensions

**For rectangular mask:**
```
X_MAX = max(x values) + 50mm margin
Y_MAX = max(y values) + 50mm margin
X_MIN = 0 (sensor is at origin)
Y_MIN = 0
```

**For radial mask:**
```
RADIUS = max(√(x² + y²)) + 50mm margin
```

**Example calculation:**
```
Max X = 350mm → X_MAX = 400mm
Max Y = 350mm → Y_MAX = 400mm
Max radius = √(350² + 350²) = 495mm → RADIUS = 545mm
```

### Step 4: Apply and Test
1. Update mask values in code:
   ```cpp
   const float SELF_MASK_X_MAX = 400.0;
   const float SELF_MASK_Y_MAX = 400.0;
   const float SELF_MASK_RADIUS_MM = 545.0;
   ```

2. Re-enable masking code
3. Upload and verify false positives are gone

### Step 5: Fine Tuning
- If still getting false alarms: increase mask dimensions by 50mm
- If missing real obstacles: decrease mask dimensions by 30mm
- Test from multiple angles and distances

---

## Vertical Band Configuration Examples

### Example 1: Flat Warehouse Floor
**Scenario**: Lift operating on smooth concrete, main concern is people/equipment at waist height

```cpp
// Focus on middle zones
const uint8_t GENERAL_ROWS_START = 2;
const uint8_t GENERAL_ROWS_END = 5;

// Disable overhead (ceiling is high)
const uint8_t OVERHEAD_ROWS_START = 7;  // Empty range
const uint8_t OVERHEAD_ROWS_END = 6;    // (start > end = disabled)

// Minimal low detection
const uint8_t LOW_ROWS_START = 0;
const uint8_t LOW_ROWS_END = 1;
```

---

### Example 2: Outdoor Construction Site
**Scenario**: Uneven ground, overhead hazards (beams, scaffolding), varied obstacles

```cpp
// Full vertical coverage
const uint8_t GENERAL_ROWS_START = 1;
const uint8_t GENERAL_ROWS_END = 6;

// Strong overhead detection
const uint8_t OVERHEAD_ROWS_START = 4;
const uint8_t OVERHEAD_ROWS_END = 7;

// Full ground detection
const uint8_t LOW_ROWS_START = 0;
const uint8_t LOW_ROWS_END = 2;
```

---

### Example 3: Indoor with Low Clearance
**Scenario**: Low ceilings, need to avoid overhead pipes, lighting, etc.

```cpp
// Standard middle detection
const uint8_t GENERAL_ROWS_START = 2;
const uint8_t GENERAL_ROWS_END = 5;

// Aggressive overhead detection
const uint8_t OVERHEAD_ROWS_START = 3;  // Start lower
const uint8_t OVERHEAD_ROWS_END = 7;

// Minimal low
const uint8_t LOW_ROWS_START = 0;
const uint8_t LOW_ROWS_END = 1;
```

---

## Testing Procedures

### Test 1: Self-Mask Verification
**Goal**: Confirm lift structure doesn't trigger false alarms

**Procedure:**
1. Power up system on mounted lift
2. Observe serial output over full 270° sweep
3. Listen for buzzer - should be SILENT when no external objects present
4. Repeat at different lift heights (extended vs compressed)

**Pass Criteria:**
- No buzzer activation during sweep
- Serial shows "-1" (no detection) for most angles
- Only real obstacles trigger alarm

---

### Test 2: Edge Detection Accuracy
**Goal**: Verify trigonometric scaling works correctly

**Setup:**
- Place test object (box, cone) at known distance from lift edge
- Test at multiple angles: 0°, 30°, 60°, 90°, 120°, 150°, 180°, 210°, 240°, 270°

**Test positions:**
```
Position A: 100mm perpendicular to lift edge, 500mm along edge
Position B: 200mm perpendicular to lift edge, 500mm along edge  
Position C: 300mm perpendicular to lift edge, 500mm along edge
```

**Expected behavior:**
| Angle | Position A | Position B | Position C |
|-------|------------|------------|------------|
| 10°   | ALARM      | NO ALARM   | NO ALARM   |
| 45°   | ALARM      | ALARM      | NO ALARM   |
| 90°   | ALARM      | ALARM      | NO ALARM   |
| 180°  | ALARM      | ALARM      | NO ALARM   |

**Pass Criteria:**
- Object at Position A (10cm from edge) ALWAYS triggers
- Object at Position C (30cm from edge) NEVER triggers
- Smooth transition, no discontinuities

---

### Test 3: Vertical Band Detection
**Goal**: Confirm multi-zone detection at different heights

**Setup:**
- Test objects at different vertical positions:
  - Low: 0-30cm from ground
  - Mid: 30-120cm (waist height)
  - High: 120-180cm (overhead)

**Procedure:**
1. Place object at 150mm distance, LOW position
2. Verify ALARM activates
3. Move same object to MID position
4. Verify ALARM activates
5. Move same object to HIGH position
6. Verify ALARM activates

**Pass Criteria:**
- All three heights detect at same horizontal distance
- No vertical blind spots

---

### Test 4: Adaptive Sweep Performance
**Goal**: Verify fast/slow mode transitions

**Procedure:**
1. Start with clear area - observe FAST sweep
2. Measure time for full 270° sweep (should be ~1.2 sec)
3. Place object within threshold
4. Observe transition to SLOW mode (finer steps)
5. Remove object
6. Verify return to FAST mode after SLOW_HOLD_POINTS

**Pass Criteria:**
- Clear area: completes sweep in 1-2 seconds
- Threat detected: slows to 1° steps
- Hysteresis: stays in SLOW mode for ~20-30 points after threat clears
- Smooth transitions, no oscillation

---

### Test 5: Buzzer Alarm Characteristics
**Goal**: Verify alarm is noticeable and increases with proximity

**Procedure:**
1. Place object at 200mm (threshold)
2. Note buzzer frequency and cadence
3. Move object closer in 50mm increments
4. Record frequency and cadence at each position

**Expected response curve:**
```
Distance   Frequency   Cadence Period
────────   ─────────   ──────────────
200mm      800 Hz      300ms
150mm      1650 Hz     175ms
100mm      2500 Hz     113ms
50mm       3350 Hz     75ms
0mm        3500 Hz     50ms
```

**Pass Criteria:**
- Frequency increases monotonically with proximity
- Cadence speeds up monotonically
- Alarm is clearly audible at 3 meters distance
- Distinct urgency difference between 200mm and 50mm

---

## Troubleshooting Decision Tree

```
┌─────────────────────────────┐
│  Problem: No sensor detect  │
└──────────┬──────────────────┘
           │
           ├─→ Check I2C scanner
           │   └─→ Found 0x29? ─→ Sensor OK, check code
           │   └─→ Not found? ─→ Check wiring
           │
           └─→ Check power supply
               └─→ 3.3V present? ─→ OK
               └─→ No voltage? ─→ Fix power

┌─────────────────────────────┐
│ Problem: False alarms       │
└──────────┬──────────────────┘
           │
           ├─→ From lift structure?
           │   └─→ YES: Expand self-mask dimensions
           │   └─→ NO: Continue
           │
           ├─→ From distant objects?
           │   └─→ YES: Increase SIN_CLAMP_MIN
           │   └─→ NO: Continue
           │
           └─→ At specific angles?
               └─→ YES: Widen DEADBAND_DEG
               └─→ NO: Check sensor calibration

┌─────────────────────────────┐
│ Problem: Missing obstacles  │
└──────────┬──────────────────┘
           │
           ├─→ At edge angles?
           │   └─→ YES: Decrease SIN_CLAMP_MIN
           │   └─→ NO: Continue
           │
           ├─→ Overhead objects?
           │   └─→ YES: Lower OVERHEAD_ROWS_START
           │   └─→ NO: Continue
           │
           └─→ All angles?
               └─→ YES: Increase BASE_THRESHOLD_MM
               └─→ NO: Check sensor range

┌─────────────────────────────┐
│ Problem: Weak buzzer        │
└──────────┬──────────────────┘
           │
           ├─→ Use active buzzer (not passive)
           ├─→ Add transistor amplifier
           ├─→ Increase BUZZER_FREQ_MAX
           └─→ Check buzzer current rating
```

---

## Performance Benchmarking

### Benchmark 1: Sweep Speed
**Measure**: Time for complete 270° sweep

**Method:**
```cpp
// Add to code:
unsigned long sweepStart = 0;
bool timing = false;

// In loop, when angle hits 0:
if (currentAngle < 1.0 && !timing) {
  sweepStart = millis();
  timing = true;
}

// When angle hits 270:
if (currentAngle > 269.0 && timing) {
  Serial.print("Sweep time: ");
  Serial.println(millis() - sweepStart);
  timing = false;
}
```

**Target values:**
- FAST mode: 1000-1500ms
- SLOW mode: 8000-10000ms

---

### Benchmark 2: Detection Latency
**Measure**: Time from object entry to alarm activation

**Method:**
1. Set up high-speed camera or oscilloscope
2. Trigger object into detection zone
3. Measure time to first buzzer tone

**Target value:** <100ms

---

### Benchmark 3: CPU Usage
**Measure**: Loop execution time

**Method:**
```cpp
// Add to loop:
unsigned long loopStart = micros();

// ... existing loop code ...

unsigned long loopTime = micros() - loopStart;
Serial.print("Loop time: ");
Serial.print(loopTime);
Serial.println(" us");
```

**Target value:** <50ms per loop (allows 20Hz update rate)

---

## Common Configuration Mistakes

### Mistake 1: Servo pulse width wrong
**Symptom**: Servo doesn't reach full 270° range

**Fix:**
```cpp
// Test different values:
const int SERVO_MIN_US = 500;   // Try: 400, 500, 600
const int SERVO_MAX_US = 2500;  // Try: 2300, 2500, 2700

// Verify with test code:
void setup() {
  scanServo.writeMicroseconds(500);
  delay(2000);
  scanServo.writeMicroseconds(2500);
}
```

---

### Mistake 2: Self-mask too small
**Symptom**: Constant false alarms from lift structure

**Fix:** Increase all mask dimensions by 50mm at a time until false alarms stop

---

### Mistake 3: Buzzer inverted
**Symptom**: Buzzer beeps when CLEAR, silent when THREAT

**Fix:**
```cpp
// Change from:
proximityAlert(closestDist);

// To:
if (!threatDetected) {
  silenceBuzzer();
} else {
  proximityAlert(closestDist);
}
```

---

### Mistake 4: Wrong I2C pins
**Symptom**: Sensor not detected

**Fix:** ESP32 variants have different default I2C pins:
```cpp
// ESP32 DevKit: GPIO 21 (SDA), 22 (SCL) ✓
// ESP32-S3: GPIO 8 (SDA), 9 (SCL)
// ESP32-C3: GPIO 5 (SDA), 6 (SCL)

// Explicitly set in code:
Wire.begin(YOUR_SDA_PIN, YOUR_SCL_PIN);
```

---

## Integration with Other Systems

### Add Wi-Fi Telemetry
```cpp
#include <WiFi.h>
#include <AsyncTCP.h>

// In setup():
WiFi.begin("SSID", "password");

// In loop when threat detected:
if (threatDetected) {
  String msg = "ALERT: " + String(closestDist) + "mm at " + String(currentAngle) + "°";
  // Send via HTTP, MQTT, WebSocket, etc.
}
```

---

### Log to SD Card
```cpp
#include <SD.h>

File logFile = SD.open("/safety_log.csv", FILE_APPEND);
logFile.print(millis()); logFile.print(",");
logFile.print(currentAngle); logFile.print(",");
logFile.println(closestDist);
logFile.close();
```

---

### Add LED Indicators
```cpp
const int LED_CLEAR = 13;
const int LED_WARN = 14;
const int LED_DANGER = 15;

if (closestDist > 200) {
  digitalWrite(LED_CLEAR, HIGH);
} else if (closestDist > 100) {
  digitalWrite(LED_WARN, HIGH);
} else {
  digitalWrite(LED_DANGER, HIGH);
}
```

---

## Maintenance Schedule

### Daily (Before Each Use)
- [ ] Visual inspection of sensor lens (clean if dirty)
- [ ] Verify servo moves through full 270° range
- [ ] Test buzzer with hand wave test
- [ ] Check all wire connections

### Weekly
- [ ] Run full test suite (Tests 1-5)
- [ ] Verify self-mask is still effective
- [ ] Check buzzer volume (batteries wear)
- [ ] Inspect mount for looseness

### Monthly
- [ ] Recalibrate self-mask if lift modified
- [ ] Verify threshold distances with ruler test
- [ ] Check for firmware updates
- [ ] Backup configuration settings

---

**Document Version**: 1.0  
**Last Updated**: November 2025  
**For System Version**: 2.0

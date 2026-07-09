/*
 * VL53L7CX Distance Mapping with Proximity Buzzer
 * ESP32-C3 Super Mini + VL53L7CX Sensor + SFM-27 Active Buzzer
 * 
 * Hardware Connections:
 * VL53L7CX -> ESP32-C3 Super Mini
 * SDA      -> GPIO4
 * SCL      -> GPIO3
 * VCC      -> 3.3V
 * GND      -> GND
 * LPn      -> GPIO10
 * 
 * BUZZER (SFM-27 Active Buzzer):
 * ESP32 GPIO5 -> 1k resistor -> 2N3904 Base
 * 2N3904 Collector -> Buzzer (-)
 * 2N3904 Emitter -> GND
 * Buzzer (+) -> 9V (+)
 * 9V (-) -> GND (common with ESP32)
 */

#include <Wire.h>
#include <vl53l7cx_class.h>

// I2C pins for ESP32-C3
#define I2C_SDA 4
#define I2C_SCL 3
#define LPN_PIN 10

// BUZZER PIN - GPIO5
#define BUZZER_PIN 5

// Create sensor object
VL53L7CX sensor_vl53l7cx(&Wire, LPN_PIN);

// Smoothing variables
int lastDistance = 999;
int smoothedDistance = 999;
const float SMOOTHING_FACTOR = 0.3;  // Lower = smoother (0.1-0.5)

// Configuration
const uint8_t RESOLUTION = VL53L7CX_RESOLUTION_8X8;
const uint8_t RANGING_FREQ = 15;

// Buzzer behavior settings
const int MAX_DETECT_CM = 100;
const int MIN_DETECT_CM = 5;  // Changed from 2 to 5
const int CONTINUOUS_TONE_THRESHOLD = 10;

// Beep state
bool beepOn = false;
unsigned long beepNextToggleMs = 0;
int beepOnMs = 0;
int beepOffMs = 0;

// Fade-out control
bool fadingOut = false;
int fadeStep = 0;
unsigned long fadeNextMs = 0;
const int FADE_STEPS = 3;  // Changed from 4 to 3
const unsigned long FADE_STEP_MS = 80;  // Changed from 100 to 80

bool isRunning = false;

// Simple ON/OFF for active buzzer
void buzzerOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
}

// Convert distance to beep parameters
void distanceToBeepParams(int distCm, int &onMs, int &offMs) {
  distCm = constrain(distCm, MIN_DETECT_CM, MAX_DETECT_CM);
  
  // Exponential curve for smoother transitions
  float normalized = (float)(distCm - MIN_DETECT_CM) / (float)(MAX_DETECT_CM - MIN_DETECT_CM);
  float curve = normalized * normalized;
  onMs = (int)(50 + curve * 150);
  offMs = (int)(50 + curve * 450);
}

// Get closest valid distance - checking ALL zones now
int getClosestDistance(VL53L7CX_ResultsData *Results) {
  int minDist = 9999;
  int validCount = 0;
  int gridSize = (RESOLUTION == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;
  
  // Check ALL zones (not just center) for 100cm range
  for (int y = 0; y < gridSize; y++) {  // Changed from y=2 to y=0
    for (int x = 0; x < gridSize; x++) {  // Changed from x=2 to x=0
      int index = y * gridSize + x;
      int16_t distance = Results->distance_mm[index] / 10;
      uint8_t status = Results->target_status[index];
      
      if ((status == 5 || status == 9) && distance > 0 && distance <= MAX_DETECT_CM) {
        if (distance < minDist) {
          minDist = distance;
        }
        validCount++;
      }
    }
  }
  
  return (validCount >= 3) ? minDist : 999;
}

// Smoothing function - SEPARATE, NOT INSIDE getClosestDistance!
int smoothDistance(int newDistance) {
  if (smoothedDistance == 999) {
    // First reading - initialize
    smoothedDistance = newDistance;
  } else {
    // Smooth using exponential moving average
    smoothedDistance = (int)(SMOOTHING_FACTOR * newDistance + (1.0 - SMOOTHING_FACTOR) * smoothedDistance);
  }
  return smoothedDistance;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\nVL53L7CX Proximity Buzzer System");
  Serial.println("SFM-27 Active Buzzer on GPIO5");
  Serial.println("================================");
  
  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.print("Buzzer pin: GPIO");
  Serial.println(BUZZER_PIN);
  Serial.print("Detection range: ");
  Serial.print(MAX_DETECT_CM);
  Serial.println(" cm\n");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, HIGH);
  delay(100);
  
  Serial.println("Initializing sensor...");
  
  sensor_vl53l7cx.begin();
  sensor_vl53l7cx.vl53l7cx_off();
  
  if (sensor_vl53l7cx.init_sensor() != VL53L7CX_STATUS_OK) {
    Serial.println("✗ Sensor initialization failed!");
    Serial.println("Check wiring and I2C connection.");
    
    // Error beep pattern
    for (int i = 0; i < 5; i++) {
      buzzerOn();
      delay(100);
      buzzerOff();
      delay(100);
    }
    
    while (1) delay(100);
  }
  
  Serial.println("✓ Sensor initialized!");
  
  sensor_vl53l7cx.vl53l7cx_set_resolution(RESOLUTION);
  sensor_vl53l7cx.vl53l7cx_set_ranging_frequency_hz(RANGING_FREQ);
  
  if (sensor_vl53l7cx.vl53l7cx_start_ranging() == VL53L7CX_STATUS_OK) {
    isRunning = true;
    Serial.println("✓ Ranging started!");
    Serial.println("\nBuzzer active - move object close to sensor\n");
    
    // Success beep
    buzzerOn();
    delay(200);
    buzzerOff();
  } else {
    Serial.println("✗ Failed to start ranging!");
    while (1) delay(100);
  }
  
  delay(500);
}

void loop() {
  unsigned long now = millis();
  
  if (isRunning) {
    VL53L7CX_ResultsData Results;
    uint8_t NewDataReady = 0;
    
    sensor_vl53l7cx.vl53l7cx_check_data_ready(&NewDataReady);
    
    if (NewDataReady) {
      sensor_vl53l7cx.vl53l7cx_get_ranging_data(&Results);
      
      int rawDistance = getClosestDistance(&Results);

      // Apply smoothing
      int closestCm;
      if (rawDistance == 999) {
        closestCm = 999;
        smoothedDistance = 999;  // Reset smoothing when object lost
      } else {
        closestCm = smoothDistance(rawDistance);
      }
      
      // Print debug info
      Serial.print("Raw: ");
      Serial.print(rawDistance);
      Serial.print(" cm | Smooth: ");
      if (closestCm == 999) {
        Serial.println("No object");
      } else {
        Serial.print(closestCm);
        Serial.println(" cm");
      }
      
      // ========== BUZZER LOGIC ==========
      
      if (closestCm == 999) {
        // No object - start fade or ensure silence
        if ((beepOnMs > 0 || beepOn) && !fadingOut) {
          fadingOut = true;
          fadeStep = 0;
          fadeNextMs = now + 20;
          beepNextToggleMs = 0;
        } else if (!fadingOut) {
          buzzerOff();
          beepOn = false;
          beepOnMs = 0;
          beepOffMs = 0;
          beepNextToggleMs = 0;
        }
      }
      else {
        // Object detected!
        if (fadingOut) {
          fadingOut = false;
          fadeStep = 0;
        }
        
        int desiredOn, desiredOff;
        distanceToBeepParams(closestCm, desiredOn, desiredOff);
        
        // Very close = continuous tone
        if (closestCm <= CONTINUOUS_TONE_THRESHOLD) {
          if (beepNextToggleMs != 0) {
            buzzerOn();
            beepOn = true;
            beepNextToggleMs = 0;  // Disable toggle for continuous
          }
        }
        else {
          // Normal beeping - increased threshold for less choppiness
          if (abs(desiredOn - beepOnMs) > 20 || abs(desiredOff - beepOffMs) > 40) {
            beepOnMs = desiredOn;
            beepOffMs = desiredOff;
            
            if (beepNextToggleMs == 0) {
              buzzerOn();
              beepOn = true;
              beepNextToggleMs = now + beepOnMs;
            }
          }
        }
      }
    }
  }
  
  // ========== BEEP TOGGLE ==========
  if (!fadingOut && beepOnMs > 0 && beepNextToggleMs > 0 && now >= beepNextToggleMs) {
    if (beepOn) {
      buzzerOff();
      beepOn = false;
      beepNextToggleMs = now + beepOffMs;
    } else {
      buzzerOn();
      beepOn = true;
      beepNextToggleMs = now + beepOnMs;
    }
  }
  
  // ========== FADE-OUT ==========
  if (fadingOut && now >= fadeNextMs) {
    if (fadeStep < FADE_STEPS) {
      buzzerOn();
      delay(FADE_STEP_MS - 20);
      buzzerOff();
      fadeNextMs = now + FADE_STEP_MS;
      fadeStep++;
    } else {
      buzzerOff();
      beepOn = false;
      fadingOut = false;
      fadeStep = 0;
      beepOnMs = 0;
      beepOffMs = 0;
      beepNextToggleMs = 0;
    }
  }
  
  delay(10);
}
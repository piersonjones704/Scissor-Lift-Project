/*
 * VL53L7CX Distance Mapping with ESP32-C3 Super Mini
 * Using ST's Official VL53L7CX ULD API Library
 * 
 * Hardware Connections:
 * VL53L7CX -> ESP32-C3 Super Mini
 * SDA      -> GPIO4 (default I2C SDA)
 * SCL      -> GPIO3 (default I2C SCL)
 * VCC      -> 3.3V
 * GND      -> GND
 * LPn      -> GPIO10 (optional, for power control)
 * 
 * Library Installation:
 * 1. Download from: https://github.com/stm32duino/VL53L7CX
 * 2. Arduino IDE: Sketch -> Include Library -> Add .ZIP Library
 * 3. Select the downloaded ZIP file
 * 
 * OR search "VL53L7CX" by STMicroelectronics in Library Manager
 */

#include <Wire.h>
#include <vl53l7cx_class.h>

// I2C pins for ESP32-C3
#define I2C_SDA 4
#define I2C_SCL 3
#define LPN_PIN 10  // Optional reset pin

// Create sensor object
VL53L7CX sensor_vl53l7cx(&Wire, LPN_PIN);

// Configuration
const uint8_t RESOLUTION = VL53L7CX_RESOLUTION_8X8;  // 8x8 or VL53L7CX_RESOLUTION_4X4 for 4x4
const uint8_t RANGING_FREQ = 15;  // Hz (1-60)

bool isRunning = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("VL53L7CX Distance Mapper");
  Serial.println("Using ST Official Library");
  Serial.println("========================");
  
  // Initialize I2C with ESP32-C3 pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz I2C
  
  // Configure LPN pin if used
  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, HIGH);
  delay(100);
  
  Serial.println("Initializing sensor...");
  
  // Initialize sensor
  sensor_vl53l7cx.begin();
  
  // Turn off sensor
  sensor_vl53l7cx.vl53l7cx_off();
  
  // Initialize sensor
  if (sensor_vl53l7cx.init_sensor() != VL53L7CX_STATUS_OK) {
    Serial.println("Sensor initialization failed!");
    Serial.println("Check wiring and I2C connection.");
    while (1) {
      delay(100);
    }
  }
  
  Serial.println("Sensor found and initialized!");
  
  // Configure resolution
  sensor_vl53l7cx.vl53l7cx_set_resolution(RESOLUTION);
  
  // Configure ranging frequency
  sensor_vl53l7cx.vl53l7cx_set_ranging_frequency_hz(RANGING_FREQ);
  
  // Start ranging
  if (sensor_vl53l7cx.vl53l7cx_start_ranging() == VL53L7CX_STATUS_OK) {
    isRunning = true;
    Serial.println("Ranging started!");
    Serial.println("\nOutputting JSON data...\n");
  } else {
    Serial.println("Failed to start ranging!");
    while (1) {
      delay(100);
    }
  }
  
  delay(500);
}

void loop() {
  if (isRunning) {
    VL53L7CX_ResultsData Results;
    uint8_t NewDataReady = 0;
    
    // Check if new data is ready
    sensor_vl53l7cx.vl53l7cx_check_data_ready(&NewDataReady);
    
    if (NewDataReady) {
      // Get ranging data
      sensor_vl53l7cx.vl53l7cx_get_ranging_data(&Results);
      
      // Output as JSON
      Serial.println("{");
      Serial.print("  \"timestamp\": ");
      Serial.print(millis());
      Serial.println(",");
      Serial.println("  \"grid\": [");
      
      // Results are stored in linear array, convert to 8x8 grid
      int gridSize = (RESOLUTION == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;
      
      for (int y = 0; y < gridSize; y++) {
        Serial.print("    [");
        for (int x = 0; x < gridSize; x++) {
          int index = y * gridSize + x;
          int16_t distance = Results.distance_mm[index];
          uint8_t status = Results.target_status[index];
          
          // Status 5 and 9 indicate valid measurements
          if (status == 5 || status == 9) {
            Serial.print(distance);
          } else {
            Serial.print("null");
          }
          
          if (x < gridSize - 1) Serial.print(", ");
        }
        Serial.print("]");
        if (y < gridSize - 1) Serial.println(",");
        else Serial.println();
      }
      
      Serial.println("  ]");
      Serial.println("}");
      Serial.println();
    }
  }
  
  delay(10);
}

// Alternative output formats below - uncomment to use

/*
// CSV Output Format
void outputCSV(VL53L7CX_ResultsData *Results) {
  Serial.print(millis());
  Serial.print(",");
  
  int gridSize = (RESOLUTION == VL53L7CX_RESOLUTION_8X8) ? 64 : 16;
  
  for (int i = 0; i < gridSize; i++) {
    int16_t distance = Results->distance_mm[i];
    uint8_t status = Results->target_status[i];
    
    if (status == 5 || status == 9) {
      Serial.print(distance);
    } else {
      Serial.print("0");
    }
    
    if (i < gridSize - 1) Serial.print(",");
  }
  Serial.println();
}
*/

/*
// ASCII Heatmap Output
void outputHeatmap(VL53L7CX_ResultsData *Results) {
  Serial.println("\n========== Distance Heatmap ==========");
  
  int gridSize = (RESOLUTION == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;
  
  for (int y = 0; y < gridSize; y++) {
    for (int x = 0; x < gridSize; x++) {
      int index = y * gridSize + x;
      int16_t distance = Results->distance_mm[index];
      uint8_t status = Results->target_status[index];
      
      if (status == 5 || status == 9) {
        char symbol;
        if (distance < 200) symbol = '#';
        else if (distance < 500) symbol = '@';
        else if (distance < 1000) symbol = '*';
        else if (distance < 2000) symbol = '+';
        else if (distance < 3000) symbol = '.';
        else symbol = ' ';
        
        Serial.print(symbol);
        Serial.print(" ");
      } else {
        Serial.print("? ");
      }
    }
    Serial.println();
  }
  
  Serial.println("======================================\n");
}
*/

#include <Wire.h>

#define SDA_PIN  8
#define SCL_PIN  9
#define TCAADDR  0x70

void tcaDisableAll() {
  Wire.beginTransmission(TCAADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

void tcaSelect(uint8_t ch) {
  tcaDisableAll();
  delay(10);
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delay(20);
}

void scanBus(const char* label) {
  Serial.print(label);
  Serial.println(" — scanning...");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == 0x70) Serial.print("  <-- MUX");
      if (addr == 0x29) Serial.print("  <-- VL53L8CX !!!");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("  (nothing found)");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(10000);

  scanBus("Main bus");

  for (uint8_t ch = 0; ch < 6; ch++) {
    tcaSelect(ch);
    char label[20];
    sprintf(label, "Channel %d", ch);
    scanBus(label);
    tcaDisableAll();
    delay(50);
  }

  Serial.println("Done.");
}

void loop() {}
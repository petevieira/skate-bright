#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(2000);          // give USB serial time
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("I2C scan...");
}

void loop() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  Serial.print("Done. Found ");
  Serial.print(count);
  Serial.println(" device(s).");
  delay(2000);
}

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SERIAL_BAUD_RATE 115200
#define BNO08X_RESET     -1
#define BNO_RST_PIN 8

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

static const sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
static const uint32_t reportIntervalUs = 10000; // 100 Hz (start here)
static uint32_t lastBeat = 0;

bool enableReports() {
  const uint32_t intervalUs = 10000; // 100 Hz
  bool ok = true;

  ok &= bno08x.enableReport(SH2_ARVR_STABILIZED_RV, intervalUs);

  Serial.print("enableReports: ");
  Serial.println(ok ? "OK" : "FAILED");
  return ok;
}


void pulseBnoReset() {
  pinMode(BNO_RST_PIN, OUTPUT);
  digitalWrite(BNO_RST_PIN, LOW);
  delay(10);
  digitalWrite(BNO_RST_PIN, HIGH);
  delay(500);
}

bool beginWithRetry(uint8_t addr, TwoWire* theWire) {
  for (int attempt = 0; attempt < 30; attempt++) {   // ~3 seconds total
    pulseBnoReset();
    if (bno08x.begin_I2C(addr, theWire)) return true;
    delay(100);
  }
  return false;
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(2000);

  while (!Serial) delay(10);

  Wire.begin();
  Wire.setClock(100000); // try 400k; if unstable, drop to 100k

  // Give BNO085 time to boot on cold power-up
  delay(1000);

  Serial.println("Starting BNO08x over I2C...");

  if (!beginWithRetry(0x4A, &Wire)) {
    Serial.println("BNO08x begin_I2C failed (even though 0x4A ACKs).");
    while (1) delay(10);
  }

  Serial.println("BNO08x found!");
  enableReports();
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("BNO08x was reset; re-enabling reports");
    enableReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      Serial.println("Got data!");
            float qr = sensorValue.un.accelerometer.x;
      float qi = sensorValue.un.accelerometer.y;
      float qj = sensorValue.un.accelerometer.z;
      // const float qr = sensorValue.un.arvrStabilizedRV.real;
      // const float qi = sensorValue.un.arvrStabilizedRV.i;
      // const float qj = sensorValue.un.arvrStabilizedRV.j;
      // const float qk = sensorValue.un.arvrStabilizedRV.k;

      Serial.print("Quat: ");
      Serial.print(qr, 3); Serial.print(", ");
      Serial.print(qi, 3); Serial.print(", ");
      Serial.print(qj, 3); Serial.print(", ");
      // Serial.println(qk, 6);
    }
    return;
  }

    // Heartbeat if no events
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 1000) {
    Serial.println("...waiting for events");
    lastBeat = millis();
  }
}

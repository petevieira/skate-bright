/**
 * Datasheet:
 *   - https://www.oakchina.cn/wp-content/uploads/2022/05/BNO080_085-Datasheet.pdf
 */

#include "debug.h"
#include "skate_params.h"
#include "skate_imu.h"

void SkateImu::initialize() {
  DEBUG_PRINTLN("Initting IMU");

#if __PROTOCOL__ == PROTOCOL_I2C
  initI2c();
#elif __PROTOCOL__ == PROTOCOL_UART
  initUart();
#endif

  DEBUG_PRINTLN("BNO08x Found!");

  // Enable the report we want
  // interval_us is microseconds between reports (e.g., 20000 us = 50 Hz)
  if (!imu.enableReport(reportType, IMU_REPORT_RATE_US)) {
    DEBUG_PRINTLN("Failed to enable report!");
    return;
  }

  DEBUG_PRINTLN("IMU reports enabled");
}

void SkateImu::initI2c() {
  // Wire.begin();
  // Wire.setClock(100000);

  if (!imu.begin_I2C()) {
    DEBUG_PRINTLN("IMU Failed I2C init");
    while (1) {
      delay(10);
    }
  }
  DEBUG_PRINTLN("IMU I2C initted");
}

void SkateImu::initUart() {
  DEBUG_PRINTLN("Beginning UART");

  // BNO08x default UART baud rate is 3000000 (3 Mbps)
  Serial1.begin(3000000);
  while (!Serial1) {
    delay(10);
  }
  DEBUG_PRINTLN("Serial1 ready");

  // Hardware reset before init
  reset();

  if (!imu.begin_UART(&Serial1)) {
    DEBUG_PRINTLN("IMU Failed UART init");
    while (1) {
      delay(10);
    }
  }
  DEBUG_PRINTLN("IMU UART initted");
}

void SkateImu::reset() {
  pinMode(IMU_RESET_PIN, OUTPUT);

  // Assert reset (active-low)
  digitalWrite(IMU_RESET_PIN, LOW);
  delay(20);

  // Deassert reset
  digitalWrite(IMU_RESET_PIN, HIGH);

  // Give it time to boot
  delay(500);
}

bool SkateImu::enableReports() {
  if (!imu.enableReport(reportType, IMU_REPORT_RATE_US)) {
    DEBUG_PRINTLN("Failed to enable ARVR stabilized RV report");
    return false;
  }
  DEBUG_PRINTLN("Enabled IMU reports");
  return true;
}

void SkateImu::setZero() {

}

bool SkateImu::readImu() {
  if (imu.getSensorEvent(&sensorValue)) {
    // Only print if the event is the type we enabled
    if (sensorValue.sensorId == reportType) {
      float qr = sensorValue.un.arvrStabilizedRV.real;
      float qi = sensorValue.un.arvrStabilizedRV.i;
      float qj = sensorValue.un.arvrStabilizedRV.j;
      float qk = sensorValue.un.arvrStabilizedRV.k;

      DEBUG_PRINT("Quat: R=");
      DEBUG_PRINTF(qr, 6);
      DEBUG_PRINT(" X=");
      DEBUG_PRINTF(qi, 6);
      DEBUG_PRINT(" Y=");
      DEBUG_PRINTF(qj, 6);
      DEBUG_PRINT(" Z=");
      DEBUG_PRINTF(qk, 6);

      return true;
    }
  }

  return false;
}

bool SkateImu::process() {
  readImu();
  // void enableReports();
  // bool newEventsExist = imu.getSensorEvent(&sensorValue);
  // return newEventsExist;
}

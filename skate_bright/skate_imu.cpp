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

  enableReports();
}

void SkateImu::initI2c() {
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

  if (!imu.begin_UART(&Serial1)) {
    DEBUG_PRINTLN("IMU Failed UART init");
    while (1) {
      delay(10);
    }
  }
  DEBUG_PRINTLN("IMU UART initted");
}

bool SkateImu::enableReports() {
  // Enable the reports we want
  if (!imu.enableReport(OrientationSensor, IMU_REPORT_RATE_US)) {
    DEBUG_PRINTLN("Failed to enable orientation report!");
    return;
  }
  if (!imu.enableReport(AccelerationSensor, IMU_REPORT_RATE_US)) {
    DEBUG_PRINTLN("Failed to enable acceleration report!");
    return;
  }
  if (!imu.enableReport(AngularVelocitySensor, IMU_REPORT_RATE_US)) {
    DEBUG_PRINTLN("Failed to enable angular velocity report!");
    return;
  }

  DEBUG_PRINTLN("IMU reports (orientation|acceleration|angular-velocity) enabled");

  return true;
}

void SkateImu::setZero() {

}

bool SkateImu::readImu() {
  if (imu.getSensorEvent(&sensorValue)) {
    // Only print if the event is the type we enabled
    if (sensorValue.sensorId == OrientationSensor) {
      orientation = sensorValue.un.arvrStabilizedRV;
    } else if (sensorValue.sensorId == AccelerationSensor) {
      acceleration = sensorValue.un.accelerometer;
    } else if (sensorValue.sensorId == AngularVelocitySensor) {
      angularVelocity = sensorValue.un.gyroscope;
    }

    return true;
  }

  return false;
}

bool SkateImu::process() {
  bool newDataReceived = readImu();
  return newDataReceived;
}

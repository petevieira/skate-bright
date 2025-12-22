/**
 * Datasheet:
 *   - https://www.oakchina.cn/wp-content/uploads/2022/05/BNO080_085-Datasheet.pdf
 */

#include "debug.h"
#include "skate_params.h"
#include "skate_imu.h"

#define UART_BAUD_RATE 3000000
#define REPORT_RATE_HZ 50
#define REPORT_RATE_US (1 / REPORT_RATE_HZ * 1000 * 1000)

void SkateImu::initialize() {
  DEBUG_PRINTLN("Initting IMU");
  Serial1.begin(UART_BAUD_RATE); // BNO08x default UART speed

  DEBUG_PRINTLN("Beginning UART");
  if (!imu.begin_UART(&Serial1)) {
    DEBUG_PRINTLN("BNO08x Failed UART init");
    while (1) { delay(10); }
  }
  DEBUG_PRINTLN("BNO08x Found!");

  // Enable the report we want
  // interval_us is microseconds between reports (e.g., 20000 us = 50 Hz)
  if (!imu.enableReport(report_type, REPORT_RATE_US)) {
    DEBUG_PRINTLN("Failed to enable report!");
  }
}

bool SkateImu::enableReports() {
  if (!imu.enableReport(report_type, REPORT_RATE_US)) {
    Serial.println("Failed to enable ARVR stabilized RV report");
    return false;
  }
  return true;
}

bool SkateImu::readImu() {
  if (imu.getSensorEvent(&sensor_value)) {
    // Only print if the event is the type we enabled
    if (sensor_value.sensorId == report_type) {
      float qr = sensor_value.un.arvrStabilizedRV.real;
      float qi = sensor_value.un.arvrStabilizedRV.i;
      float qj = sensor_value.un.arvrStabilizedRV.j;
      float qk = sensor_value.un.arvrStabilizedRV.k;

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
  void enableReports();
  bool new_events_exist = imu.getSensorEvent(&sensor_value);
}

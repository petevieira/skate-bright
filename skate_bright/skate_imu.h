#ifndef SKATE_IMU_H
#define SKATE_IMU_H

#include <Adafruit_BNO08x.h>

/**
 * @class SkateImu
 * @brief Class that encapsulates the IMU functionality for the inline skates.
 */
class SkateImu {
public:
  Adafruit_BNO08x imu = Adafruit_BNO08x();
  // Buffer to receive sensor events
  sh2_SensorValue_t sensor_value;
  // Report type we want to enable
  static const sh2_SensorId_t report_type = SH2_ARVR_STABILIZED_RV;

  // static const uint32_t report_interval_us = 10000; // 100 Hz (start here)

  bool enableReports();

  void initialize();

  bool readImu();

  bool process();
};

#endif
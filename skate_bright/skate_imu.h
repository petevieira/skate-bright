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
  sh2_SensorValue_t sensorValue;
  // Report type we want to enable (stabilized rotation vector)
  static const sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;

  bool enableReports();

  void initialize();

  void initI2c();

  void initUart();

  void reset();

  void setZero();

  bool readImu();

  bool process();
};

#endif
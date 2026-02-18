#ifndef SKATE_IMU_H
#define SKATE_IMU_H

#include <Adafruit_BNO08x.h>

/**
 * @class SkateImu
 * @brief Class that encapsulates the IMU functionality for the inline skates.
 * Data:
 *   Acceleration
 *     - Calibrated Acceleration (m/s2). Acceleration of the device including gravity (accelerometer)
 *     - Linear acceleration (m/s2). Acceleration of the device with gravity removed (linearAcceleration)
 *   Angular Velocity
 *     - Calibrated gyroscope (rad/s). The angular velocity of the device (gyroscope)
 *   Orientation
 *     - AR/VR Stabilized Rotation Vector (arvrStabilizedRV)
 *
 *   See https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf for more info.
 */
class SkateImu {
public:
  // data storage
  sh2_Accelerometer_t acceleration; // .x, .y, .z (m/s^2)
  sh2_RotationVectorWAcc_t orientation; // .i, .j, .k, .real (rad)
  sh2_Gyroscope_t angularVelocity; // .x, .y, .z (rad/s)

  void initialize();

  bool process();

private:
  // main object for the BNO08x board
  Adafruit_BNO08x imu = Adafruit_BNO08x();

  // Buffer to receive sensor events
  sh2_SensorValue_t sensorValue;

  // Report type we want to enable (stabilized rotation vector)
  static const sh2_SensorId_t OrientationSensor = SH2_ARVR_STABILIZED_RV; // gravity removed
  static const sh2_SensorId_t AccelerationSensor = SH2_LINEAR_ACCELERATION; // gravity removed
  static const sh2_SensorId_t AngularVelocitySensor = SH2_GYROSCOPE_CALIBRATED;

  bool enableReports();

  void initI2c();

  void initUart();

  void setZero();

  bool readImu();
};

#endif // SKATE_IMU_H
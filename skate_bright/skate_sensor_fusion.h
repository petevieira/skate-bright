#ifndef SKATE_SENSOR_FUSION_H
#define SKATE_SENSOR_FUSION_H

#include <Adafruit_BNO08x.h>
#include "skate_imu.h"
#include "skate_pressure.h"

enum class Direction : uint8_t { Stopped, Forward, Backward };
enum class Lean: uint8_t { Upright, Left, Right };
enum class Contact: unit8_t { OnGround, Airborne, Unknown };
enum class SpeedMode: uint8_t { Constant, Accelerating, Braking, Stopped, Unknown };

enum SkateFlag : uint32_t {
  FLAG_HEEL_BRAKE           = 1u << 0,
  FLAG_TOE_ONLY             = 1u << 1,
  FLAG_HEEL_ONLY            = 1u << 2,
}

inline bool has(uint32_t flags, SkateFlag f) {
  return (flags & f) != 0;
}

class SkateSensors {
  sh2_Accelerometer_t acceleration; // .x, .y, .z
  sh2_RotationVector_t orientation; // .i, .j, .k, .real
  sh2_Gyroscope_t angularVelocity; // .x, .y, .z
  float pressure;
}

class SkateState {
  // continuous estimates (for thresholds/hysteresis/debug)
  float velocity;    // signed: + forward, - backward (m/s)
  float acceleration;   // signed along travel axis (m/s^2)
  float roll;   // left/right
  float pitch;  // toe/heel
  float yaw;
  float contactProb; // 0..1

  // discrete dimensions
  Direction direction;
  Lean lean;
  Contact contact;
  SpeedMode speedMode;

  // multi-true flags
  uint32_t flags;

  // bookkeeping
  uint32_t lastUpdateMs;
};

class SkateSensorFusion {
public:

  SkateState skate;

  void update(SkateImu imu, SkatePressure pressure);

private:

  SkateSensors sensors;
  SkateSensors filteredSensorsPrev;
  SkateSensors filteredSensors;

  void storeNewSensorData(SkateImu imu, SkatePressure pressure);

  void computeSkateDynamics();

  void computeOrientationConjugate(sh2_RotationVector_t q);

  void filterMeasurement(float measurement, float filteredValPrev);

  void filterSensorAcceleration();

  void computeSkateAcceleration();

  void applyDeadband(sh2_Accelerometer_t &a, float deadband);
  
  void computeSkateVelocity();

  void computeSkateDirection();

  void computeSkateSpeedMode();

  void computeSkateRpy();

  void computeSkateLean();

  void computeSkatePressure();

  void computeContact();

  void quaternionToRpy(
    float qx, float qy, float qz, float qw,
    float *roll, float *pitch, float *yaw
  );

  inline float radToDeg(float rad) { return rad * 180 / M_PI; }
};

#endif // SKATE_SENSOR_FUSION_H
#ifndef SKATE_SENSOR_FUSION_H
#define SKATE_SENSOR_FUSION_H

#include <Adafruit_BNO08x.h>
#include "skate_imu.h"
#include "skate_pressure.h"

enum class Direction { Stationary, Forward, Backward };
enum class Lean { Upright, Left, Right };
enum class Contact { OnGround, Airborne, Unknown };
enum class SpeedMode { Constant, Accelerating, Braking, Stopped, Unknown };

inline const char* toString(Direction d) {
  switch (d) {
    case Direction::Stationary: return "Stationary";
    case Direction::Forward:    return "Forward";
    case Direction::Backward:   return "Backward";
    default:                    return "Unknown";
  }
}

inline const char* toString(Lean l) {
  switch (l) {
    case Lean::Upright: return "Upright";
    case Lean::Left:    return "Left";
    case Lean::Right:   return "Right";
    default:            return "Unknown";
  }
}

inline const char* toString(Contact c) {
  switch (c) {
    case Contact::OnGround: return "OnGround";
    case Contact::Airborne: return "Airborne";
    case Contact::Unknown:  return "Unknown";
    default:                return "Unknown";
  }
}

inline const char* toString(SpeedMode s) {
  switch (s) {
    case SpeedMode::Constant:     return "Constant";
    case SpeedMode::Accelerating: return "Accelerating";
    case SpeedMode::Braking:      return "Braking";
    case SpeedMode::Stopped:      return "Stopped";
    case SpeedMode::Unknown:      return "Unknown";
    default:                      return "Unknown";
  }
}

enum SkateFlag : uint32_t {
  FLAG_HEEL_BRAKE           = 1u << 0,
  FLAG_TOE_ONLY             = 1u << 1,
  FLAG_HEEL_ONLY            = 1u << 2,
};

inline bool has(uint32_t flags, SkateFlag f) {
  return (flags & f) != 0;
};

class SkateSensors {
public:
  Vec3f accMps2;
  Vec3f rpyRad;
  Vec3f velDegps;
  float pressure;
};

class SkateState {
public:
  // continuous estimates (for thresholds/hysteresis/debug)
  Vec3f velMps;    // signed: + forward, - backward (m/s)
  Vec3f accMps2;   // signed along travel axis (m/s^2)
  Vec3f rpyRad;   // left/right
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

  SkateSensors sensors;
  SkateSensors filteredSensorsPrev;
  SkateSensors filteredSensors;
  SkateState skate;

  void initialize();

  void update(SkateImu imu, SkatePressure pressure);

private:

  void storeNewSensorData(SkateImu imu, SkatePressure pressure);

  void computeSkateDynamics();

  void computeSkateBehavior();

  sh2_Accelerometer_t rotateByQuat(sh2_RotationVectorWAcc_t q, const sh2_Accelerometer_t &v);

  float filterMeasurement(float measurement, float filteredValPrev, float timeConstant);

  void filterSensorData();

  void filterSensorAcceleration();

  void filterSensorPressure();

  void filterSensorOrientation();

  void filterSensorAngularVelocity();

  void computeSkateAcceleration();

  void applyDeadband(float &val, float deadband);

  void computeSkateVelocity();

  void computeSkateDirection();

  void computeSkateSpeedMode();

  void computeSkateRpy();

  void computeSkateLean();

  void computeSkatePressure();

  void computeContact();

  void quaternionToRpy(
    float qx, float qy, float qz, float qw,
    float &roll, float &pitch, float &yaw
  );

  inline float radToDeg(float rad) { return rad * 180 / M_PI; }
};

#endif // SKATE_SENSOR_FUSION_H
#include <Arduino.h>
#include "skate_sensor_fusion.h"
#include "skate_params.h"

void SkateSensorFusion::initialize() {
  skate.direction = Direction::Stationary;
  skate.lean = Lean::Upright;
  skate.contact = Contact::Unknown;
  skate.speedMode = SpeedMode::Unknown;
}

void SkateSensorFusion::update(SkateImu imu, SkatePressure pressure) {
  storeNewSensorData(imu, pressure);
  filterSensorData();
  computeSkateDynamics();
  computeSkateBehavior();
}

void SkateSensorFusion::storeNewSensorData(SkateImu imu, SkatePressure pressure) {
  sensors.acceleration = imu.acceleration;
  sensors.orientation = imu.orientation;
  sensors.angularVelocity = imu.angularVelocity;
  sensors.pressure = pressure.value;
}

void SkateSensorFusion::filterSensorData() {
  filteredSensorsPrev = filteredSensors;
  filterSensorAcceleration();
  filterSensorPressure();
  filterSensorOrientation();
  filterSensorAngularVelocity();
}

void SkateSensorFusion::computeSkateDynamics() {
  // operate on sensor data
  computeSkateAcceleration();
  computeSkateVelocity();
  computeSkateRpy();
  computeSkatePressure();
}

void SkateSensorFusion::computeSkateBehavior() {
  // compute skate high-level info
  computeSkateLean();
  computeSkateDirection();
  computeSkateSpeedMode();
}

float SkateSensorFusion::computeOrientationConjugate(sh2_RotationVectorWAcc_t q) {
  return q.real - q.i - q.j - q.k;
}

float SkateSensorFusion::filterMeasurement(float measurement, float filteredValPrev, float timeConstant) {
  float dt = LOOP_DELAY_S;
  // time constant for filter
  float rc = timeConstant;
  // gain (how fast the value is allowed to change 0..1)
  float alpha = dt / (rc + dt);

  // exponentially weighted moving average (exponential smoothing)
  // y[n] = y[n-1] + α ( x[n] - y[n-1] ) (new value = old value + (fraction) × (error))
  return filteredValPrev + alpha * (measurement - filteredValPrev);
}

void SkateSensorFusion::filterSensorAcceleration() {
  filteredSensors.acceleration.x = filterMeasurement(sensors.acceleration.x, filteredSensorsPrev.acceleration.x, ACCELERATION_FILTER_TIME_CONSTANT);
  filteredSensors.acceleration.y = filterMeasurement(sensors.acceleration.y, filteredSensorsPrev.acceleration.y, ACCELERATION_FILTER_TIME_CONSTANT);
  filteredSensors.acceleration.z = filterMeasurement(sensors.acceleration.z, filteredSensorsPrev.acceleration.z, ACCELERATION_FILTER_TIME_CONSTANT);
}

void SkateSensorFusion::filterSensorPressure() {
  // exponentially weighted moving average (exponential smoothing)
  filteredSensors.pressure = filterMeasurement(sensors.pressure, filteredSensorsPrev.pressure, PRESSURE_FILTER_TIME_CONSTANT);
}

void SkateSensorFusion::filterSensorOrientation() {
  filteredSensors.orientation = sensors.orientation;
}

void SkateSensorFusion::filterSensorAngularVelocity() {
  filteredSensors.angularVelocity = sensors.angularVelocity;
}

void SkateSensorFusion::computeSkateAcceleration() {
  sh2_RotationVectorWAcc_t q = filteredSensors.orientation;
  float qConjugate = computeOrientationConjugate(filteredSensors.orientation);

  // put acceleration in skate frame (a_skateframe = orientation_quaternion * acc_sensorframe * orientation_quaternion_conjugate)
  float a = q * filteredSensors.acceleration * qConjugate;

  // zero out values inside deadband
  applyDeadband(&a.x, ACCERELATION_DEADBAND_MPS2);
  applyDeadband(&a.y, ACCERELATION_DEADBAND_MPS2);
  applyDeadband(&a.z, ACCERELATION_DEADBAND_MPS2);
  skate.acceleration = a;
}

void SkateSensorFusion::applyDeadband(float &val, float deadband) {
  if (val < deadband && val > -deadband) {
    val = 0;
  }
}

void SkateSensorFusion::computeSkateVelocity() {
  // discrete integration
  float vel = skate.acceleration.x * LOOP_DELAY_S;
  // apply deadband so anything below a certain velocity is considered stopped
  // to avoid flip-flopping
  applyDeadband(vel, VELOCITY_DEADBAND_MPS);
  // store computed velocity
  skate.velocity = vel;
}

void SkateSensorFusion::computeSkateDirection() {
  float vel = skate.velocity;
  if (vel == 0) {
    skate.direction = Stationary;
  } else if (vel > 0) {
    skate.direction = Forward;
  } else {
    skate.direction = Backward;
  }
}

void SkateSensorFusion::computeSkateSpeedMode() {
  acc = skate.acceleration.x;  

  if (skate.direction == Direction::Stationary) {
    skate.speedMode = SpeedMode::Stopped;
  } else if (skate.direction == Direction::Forward) {
    if (acc = 0) {
      skate.speedMode = SpeedMode::Constant;
    } else if (acc < 0) {
      skate.speedMode = SpeedMode::Braking;
    } else {
      skate.speedMode = SpeedMode::Accelerating;
    }
  } else if (skate.direction == Direction::Backward) {
    if (acc = 0) {
      skate.speedMode = SpeedMode::Constant;
    } else if (acc > 0) {
      skate.speedMode = SpeedMode::Braking;
    } else {
      skate.speedMode = SpeedMode::Accelerating;
    }
  } else if (skate.direction == Direction::Forward) {
    if (acc = 0) {
      skate.speedMode = SpeedMode::Constant;
    } else if (acc < 0) {
      skate.speedMode = SpeedMode::Braking;
    } else {
      skate.speedMode = SpeedMode::Accelerating;
    }
  }
}

void SkateSensorFusion::computeSkateRpy() {
  float r, p, y;
  quaternionToRpy(
    filteredSensors.orientation.i, filteredSensors.orientation.j, filteredSensors.orientation.k, filteredSensors.orientation.real,
    r, p, y
  );

  applyDeadband(&r, ANGLE_DEADBAND_RAD);
  applyDeadband(&p, ANGLE_DEADBAND_RAD);
  applyDeadband(&y, ANGLE_DEADBAND_RAD);

  skate.roll = r;
  skate.pitch = p;
  skate.yaw = y;
}

void SkateSensorFusion::computeSkateLean() {
  if (skate.roll < 0) {
    skate.lean = Lean::Right;
  } else if skate.roll > 0) {
    skate.lean = Lean::Left;
  } else {
    skate.lean = Lean::Upright;
  }
}

void SkateSensorFusion::computeContact() {
  if (filteredSensors.pressure < MIN_PRESSURE) {
    skate.contact = Contact::Airborne;
  } else if (filteredSensors.pressure >= MIN_PRESSURE) {
    skate.contact = Contact::OnGround;
  } else {
    skate.contact = Contact::Unknown;
  }
}

void SkateSensorFusion::quaternionToRpy(
  float qx, float qy, float qz, float qw,
  float *roll, float *pitch, float *yaw
) {
  float r = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx*qx + qy*qy));
  float p = -PI * 0.5 + 2 * atan2(sqrt(1 + 2 * (qw * qy - qx * qz)), sqrt(1 - 2 * qw * qy - qx * qz));
  float y = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz));

  *roll = r;
  *pitch = p;
  *yaw = y;
}




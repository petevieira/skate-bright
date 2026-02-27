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
  sensors.velDegps = imu.gyroDegps;
  sensors.accMps2 = imu.accMps2;
  sensors.rpyRad = imu.rpyRad;
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

sh2_Accelerometer_t SkateSensorFusion::rotateByQuat(sh2_RotationVectorWAcc_t q, const sh2_Accelerometer_t &v) {
  const float x = q.i;
  const float y = q.j;
  const float z = q.k;
  const float w = q.real;

  // Rotation matrix from quaternion
  const float xx = x * x, yy = y * y, zz = z * z;
  const float xy = x * y, xz = x * z, yz = y * z;
  const float wx = w * x, wy = w * y, wz = w * z;

  sh2_Accelerometer_t out;
  out.x = (1.0f - 2.0f * (yy + zz)) * v.x + (2.0f * (xy - wz)) * v.y + (2.0f * (xz + wy)) * v.z;
  out.y = (2.0f * (xy + wz)) * v.x + (1.0f - 2.0f * (xx + zz)) * v.y + (2.0f * (yz - wx)) * v.z;
  out.z = (2.0f * (xz - wy)) * v.x + (2.0f * (yz + wx)) * v.y + (1.0f - 2.0f * (xx + yy)) * v.z;

  return out;
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
  filteredSensors.accMps2.x = filterMeasurement(sensors.accMps2.x, filteredSensorsPrev.accMps2.x, ACCELERATION_FILTER_TIME_CONSTANT);
  filteredSensors.accMps2.y = filterMeasurement(sensors.accMps2.y, filteredSensorsPrev.accMps2.y, ACCELERATION_FILTER_TIME_CONSTANT);
  filteredSensors.accMps2.z = filterMeasurement(sensors.accMps2.z, filteredSensorsPrev.accMps2.z, ACCELERATION_FILTER_TIME_CONSTANT);
}

void SkateSensorFusion::filterSensorPressure() {
  // exponentially weighted moving average (exponential smoothing)
  filteredSensors.pressure = filterMeasurement(sensors.pressure, filteredSensorsPrev.pressure, PRESSURE_FILTER_TIME_CONSTANT);
}

void SkateSensorFusion::filterSensorOrientation() {
  filteredSensors.rpyRad = sensors.rpyRad;
}

void SkateSensorFusion::filterSensorAngularVelocity() {
  filteredSensors.velDegps = sensors.velDegps;
}

void SkateSensorFusion::computeSkateAcceleration() {
  // const sh2_RotationVectorWAcc_t &q = filteredSensors.orientation;

  // put acceleration in skate frame (a_skateframe = orientation_quaternion * acc_sensorframe * orientation_quaternion_conjugate)
  // sh2_Accelerometer_t accSensor{ filteredSensors.accMps2.x,
  //                                filteredSensors.accMps2.y,
  //                                filteredSensors.accMps2.z };

  // sh2_Accelerometer_t accSkate = rotateByQuat(q, accSensor);
  Vec3f accSkateMps2 = filteredSensors.accMps2;

  // zero out values inside deadband
  applyDeadband(accSkateMps2.x, ACCERELATION_DEADBAND_MPS2);
  applyDeadband(accSkateMps2.y, ACCERELATION_DEADBAND_MPS2);
  applyDeadband(accSkateMps2.z, ACCERELATION_DEADBAND_MPS2);

  skate.accMps2.x = accSkateMps2.x;
  skate.accMps2.y = accSkateMps2.y;
  skate.accMps2.z = accSkateMps2.z;
}

void SkateSensorFusion::computeSkatePressure() {
  
}

void SkateSensorFusion::applyDeadband(float &val, float deadband) {
  if (val < deadband && val > -deadband) {
    val = 0;
  }
}

void SkateSensorFusion::computeSkateVelocity() {
  // discrete integration
  float velX = skate.accMps2.x * LOOP_DELAY_S;
  float velY = skate.accMps2.y * LOOP_DELAY_S;
  float velZ = skate.accMps2.z * LOOP_DELAY_S;

  // apply deadband so anything below a certain velocity is considered stopped
  // to avoid flip-flopping
  applyDeadband(velX, VELOCITY_DEADBAND_MPS);
  applyDeadband(velY, VELOCITY_DEADBAND_MPS);
  applyDeadband(velZ, VELOCITY_DEADBAND_MPS);

  // store computed velocity
  skate.velMps = Vec3f(velX, velY, velZ);
}

void SkateSensorFusion::computeSkateDirection() {
  float velX = skate.velMps.x;
  if (velX == 0) {
    skate.direction = Direction::Stationary;
  } else if (velX > 0) {
    skate.direction = Direction::Forward;
  } else {
    skate.direction = Direction::Backward;
  }
}

void SkateSensorFusion::computeSkateSpeedMode() {
  float accX = skate.accMps2.x;

  if (skate.direction == Direction::Stationary) {
    skate.speedMode = SpeedMode::Stopped;
  } else if (skate.direction == Direction::Forward) {
    if (accX = 0) {
      skate.speedMode = SpeedMode::Constant;
    } else if (accX < 0) {
      skate.speedMode = SpeedMode::Braking;
    } else {
      skate.speedMode = SpeedMode::Accelerating;
    }
  } else if (skate.direction == Direction::Backward) {
    if (accX = 0) {
      skate.speedMode = SpeedMode::Constant;
    } else if (accX > 0) {
      skate.speedMode = SpeedMode::Braking;
    } else {
      skate.speedMode = SpeedMode::Accelerating;
    }
  } else if (skate.direction == Direction::Forward) {
    if (accX = 0) {
      skate.speedMode = SpeedMode::Constant;
    } else if (accX < 0) {
      skate.speedMode = SpeedMode::Braking;
    } else {
      skate.speedMode = SpeedMode::Accelerating;
    }
  }
}

void SkateSensorFusion::computeSkateRpy() {
  float r, p, y;
  // quaternionToRpy(
  //   filteredSensors.orientation.i, filteredSensors.orientation.j, filteredSensors.orientation.k, filteredSensors.orientation.real,
  //   r, p, y
  // );

  applyDeadband(r, ANGLE_DEADBAND_RAD);
  applyDeadband(p, ANGLE_DEADBAND_RAD);
  applyDeadband(y, ANGLE_DEADBAND_RAD);

  skate.rpyRad.x = r;
  skate.rpyRad.y = p;
}

void SkateSensorFusion::computeSkateLean() {
  if (skate.rpyRad.x < 0) {
    skate.lean = Lean::Right;
  } else if (skate.rpyRad.x > 0) {
    skate.lean = Lean::Left;
  } else {
    skate.lean = Lean::Upright;
  }
}

void SkateSensorFusion::computeContact() {
  if (filteredSensors.pressure < MIN_PRESSURE_N) {
    skate.contact = Contact::Airborne;
  } else if (filteredSensors.pressure >= MIN_PRESSURE_N) {
    skate.contact = Contact::OnGround;
  } else {
    skate.contact = Contact::Unknown;
  }
}

void SkateSensorFusion::quaternionToRpy(
  float qx, float qy, float qz, float qw,
  float &roll, float &pitch, float &yaw
) {
  float r = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx*qx + qy*qy));
  float p = -PI * 0.5 + 2 * atan2(sqrt(1 + 2 * (qw * qy - qx * qz)), sqrt(1 - 2 * qw * qy - qx * qz));
  float y = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz));

  roll = r;
  pitch = p;
  yaw = y;
}


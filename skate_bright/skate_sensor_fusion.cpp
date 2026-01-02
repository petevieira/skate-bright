#include <cmath.h>
#include "skate_sensor_fusion.h"

void SkateSensorFusion::update(imu, pressure) {
  filteredSensorsPrev = filteredSensors;
  storeNewSensorData(imu, pressure);
  computeSkateDynamics();

  return;
}

void SkateSensorFusion::storeNewSensorData(imu, pressure) {
  sensors.acceleration = imu.acceleration;
  sensors.orientation = imu.orientation;
  sensors.angularVelocity = imu.angularVelocity;
  sensors.pressure = pressure.value;
}

void SkateSensorFusion::computeSkateDynamics() {
  // operate on sensor data
  filterSensorAcceleration();
  computeSkateAcceleration();
  computeSkateVelocity();
  computeSkateRpy();
  computeSkatePressure();

  // compute skate high-level info
  computeSkateLean();
  computeSkateDirection();
  computeSkateSpeedMode();

}

void SkateSensorFusion::computeOrientationConjugate(sh2_RotationVector_t q) {
  return q - q.i - q.j - q.k;
}

void SkateSensorFusion::filterMeasurement(float &measurement, float &filteredValPrev) {
  float dt = LOOP_DELAY_S;
  // time constant for filter
  float rc = ACCELERATION_FILTER_TIME_CONSTANT;
  // gain (how fast the value is allowed to change 0..1)
  float alpha = dt / (rc + dt);

  // y[n] = y[n-1] + α ( x[n] - y[n-1] ) (new value = old value + (fraction) × (error))
  return filteredValPrev + alpha * (measurement - filteredValPrev);
}

void SkateSensorFusion::filterSensorAcceleration() {
  float dt = LOOP_DELAY_S;
  // time constant for filter
  float rc = ACCELERATION_FILTER_TIME_CONSTANT;
  // gain (how fast the value is allowed to change 0..1)
  float alpha = dt / (rc + dt);

  // update previous filtered values
  filteredSensorsPrev.acceleration.x = filteredSensors.acceleration.x;
  filteredSensorsPrev.acceleration.y = filteredSensors.acceleration.y;
  filteredSensorsPrev.acceleration.z = filteredSensors.acceleration.z;

  // y[n] = y[n-1] + α ( x[n] - y[n-1] ) (new value = old value + (fraction) × (error))
  filteredSensors.acceleration.x = filterMeasurement(sensors.acceleration.x, filteredSensorsPrev.acceleration.x);
  filteredSensors.acceleration.y = filterMeasurement(sensors.acceleration.y, filteredSensorsPrev.acceleration.y);
  filteredSensors.acceleration.z = filterMeasurement(sensors.acceleration.z, filteredSensorsPrev.acceleration.z);
}

void SkateSensorFusion::computeSkateAcceleration() {
  float q = sensors.orientation;
  float qConjugate = computeOrientationConjugate(sensors.orientation);

  // put acceleration in skate frame (a_skateframe = orientation_quaternion * acc_sensorframe * orientation_quaternion_conjugate)
  float a = q * sensors.acceleration * qConjugate;

  // zero out values inside deadband
  applyDeadband(&a, ACCELERATION_DEADBAND);
  skate.acceleration = a;
}

void SkateSensorFusion::applyDeadband(sh2_Accelerometer_t &a, float deadband) {
  if (a.x < deadband && a > -deadband) {
    a.x = 0;
  }
  if (a.y < deadband && a > -deadband) {
    a.y = 0;
  }
  if (a.z < deadband && a > -deadband) {
    a.z = 0;
  }
}

void SkateSensorFusion::computeSkateVelocity() {
  // discrete integration
  float vel = skate.acceleration.x * LOOP_DELAY_S;
  applyDeadband(vel);
  skate.velocity = vel;
}

void SkateSensorFusion::computeSkateDirection() {
  if (vel == 0) {
    skate.direction = Stopped;
  } else if (vel > 0) {
    skate.direction = Forward;
  } else {
    skate.direction = Backward;
  }

void SkateSensorFusion::computeSkateSpeedMode() {
  acc = skate.acceleration.x;  

  if (skate.direction == Stopped) {
    skate.speedMode = Stopped;
  } else if (skate.direction == Forward) {
    if (acc = 0) {
      skate.speedMode = Constant;
    } else if (acc < 0) {
      skate.speedMode = Braking;
    } else {
      skate.speedMode = Accelerating;
    }
  } else if (skate.direction == Backward) {
    if (acc = 0) {
      skate.speedMode = Constant;
    } else if (acc < 0) {
      skate.speedMode = Braking;
    } else {
      skate.speedMode = Accelerating;
    }
  }

}

void SkateSensorFusion::computeSkateSpeedMode() {
  if (skate.velocity)
}

float SkateSensorFusion::radToDeg(float rad) {
  return rad * 180 / M_PI;
}

void SkateSensorFusion::computeSkateRpy() {
  float r, p, y;
  quaternionToRpy(
    sensors.orientation.i, sensors.orientation.j, sensors.orientation.k, sensors.orientation.real,
    r, p, y
  );

  r = radToDeg(r);
  p = radToDeg(p);
  y = radToDeg(y);

  applyDeadband(&r, &p, &y);

  skate.roll = r;
  skate.pitch = p;
  skate.yaw = y;
}

void SkateSensorFusion::computeSkateLean() {
  if (skate.roll < 0) {
    skate.lean = RIGHT;
  } else if skate.roll > 0) {
    skate.lean = LEFT;
  } else {
    skate.lean = UPRIGHT;
  }
}

void SkateSensorFusion::computeSkatePressure() {
    float dt = LOOP_DELAY_S;
  // time constant for filter
  float rc = PRESSURE_FILTER_TIME_CONSTANT;
  // gain (how fast the value is allowed to change 0..1)
  float alpha = dt / (rc + dt);

  // update previous filtered values
  filteredSensorsPrev.pressure = filteredSensors.pressure;

  // y[n] = y[n-1] + α ( x[n] - y[n-1] ) (new value = old value + (fraction) × (error))
  filteredSensors.pressure = filterMeasurement(sensors.pressure, filteredSensorsPrev.pressure);
}

void SkateSensorFusion::computeContact() {
  if (filteredSensors.pressure < MIN_PRESSURE) {
    skate.contact = Airborne;
  } else if (filteredSensors.pressure >= MIN_PRESSURE) {
    skate.contact = OnGround;
  } else {
    skate.contact = Unknown;
  }
}

void SkateSensorFusion::quaternionToRpy(
  float qx, float qy, float qz, float qw,
  float *roll, float *pitch, float *yaw
) {
  float r = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx*qx + qy*qy));
  float p = -M_PI * 0.5 + 2 * atan2(sqrt(1 + 2 * (qw * qy - qx * qz)), sqrt(1 - 2 * qw * qy - qx * qz));
  float y = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*qy + qz*qz));

  *roll = r;
  *pitch = p;
  *yaw = y;
}




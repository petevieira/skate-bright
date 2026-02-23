/**
 * Datasheet:
 *   - https://www.oakchina.cn/wp-content/uploads/2022/05/BNO080_085-Datasheet.pdf
 */

#include "debug.h"
#include "skate_params.h"
#include "skate_imu.h"
#include <Wire.h>

void SkateImu::initialize() {

  DEBUG_PRINTLN("Initting IMU with I^2C");
  initI2c();
}

void SkateImu::initI2c() {
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, IMU_I2C_ADDR, -1)) {
    DEBUG_PRINTLN("IMU Failed I2C init");
    while (1) {
      delay(10);
    }
  }
  DEBUG_PRINTLN("IMU initted");
}

void SkateImu::setAccelerometerRange() {
  write8(0x41, IMU_ACCELEROMETER_RANGE);
}

void SkateImu::setZero() {

}

bool SkateImu::readImu() {
  BMI160.readGyro(gyroCounts.x, gyroCounts.y, gyroCounts.z);
  BMI160.readAccelerometer(accCounts.x, accCounts.y, accCounts.z);

  return true;
}

float SkateImu::convertAccCountsToMps2() {
  accMps2.x = (float)accCounts.x * GRAVITY_MPS2 / IMU_ACC_SENSITIVITY_COUNTS_PER_G;
  accMps2.y = (float)accCounts.y * GRAVITY_MPS2 / IMU_ACC_SENSITIVITY_COUNTS_PER_G;
  accMps2.z = (float)accCounts.z * GRAVITY_MPS2 / IMU_ACC_SENSITIVITY_COUNTS_PER_G;
}

float SkateImu::convertGyroCountsToDegps() {
  gyroDegps.x = (float)gyroCounts.x * GRAVITY_MPS2 / IMU_GYRO_SENSITIVITY_COUNTS_PER_DEG_PER_S;
  gyroDegps.y = (float)gyroCounts.y * GRAVITY_MPS2 / IMU_GYRO_SENSITIVITY_COUNTS_PER_DEG_PER_S;
  gyroDegps.z = (float)gyroCounts.z * GRAVITY_MPS2 / IMU_GYRO_SENSITIVITY_COUNTS_PER_DEG_PER_S;
}

void SkateImu::convertAccToRpyRad() {
  rpyRad.x = atan2(accMps2.y, accMps2.z);
  rpyRad.y = atan2(-accMps2.x, sqrt(accMps2.y * accMps2.y + accMps2.z * accMps2.z));
}

bool SkateImu::process() {
  readImu();
  convertAccCountsToMps2();
  convertGyroCountsToDegps();
  convertAccToRpyRad();
}

uint8_t SkateImu::read8(uint8_t reg) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_I2C_ADDR, (uint8_t)1);
  return Wire.read();
}

void SkateImu::write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

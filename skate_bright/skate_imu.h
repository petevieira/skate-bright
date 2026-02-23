#ifndef SKATE_IMU_H
#define SKATE_IMU_H

#include <BMI160Gen.h>
#include "skate_params.h"

/**
 * @class SkateImu
 * @brief Class that encapsulates the IMU functionality for the inline skates.
 */
class SkateImu {
public:
  // data storage
  Vec3i gyroCounts;
  Vec3f gyroDegps;

  Vec3i accCounts;
  Vec3f accMps2;

  Vec3f rpyRad;

  void initialize();

  bool process();

private:
  void initI2c();

  void setZero();

  bool readImu();

  void setAccelerometerRange();

  void convertAccToRpyRad();

  float convertAccCountsToMps2();

  float convertGyroCountsToDegps();

  uint8_t read8(uint8_t reg);

  void write8(uint8_t reg, uint8_t val);

};

#endif // SKATE_IMU_H
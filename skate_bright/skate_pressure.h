#ifndef SKATE_PRESSURE_H
#define SKATE_PRESSURE_H

// Heel Pressure Sensor Constants
// #define PRESSURE_SENSOR_PIN  A9;
// const float ADC_RES_BITS = 10.0;

class SkatePressure {
public:
  void initialize();
  bool process();

  float value;
};

#endif
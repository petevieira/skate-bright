/**
 * This sketch is for NeoPixel LED light strips whose lights change
 * depending on two sensor:
 *   1) an IMU on the outside of the skate
 *   2) a velostat pressure in the heel of the skate
 * We need to read an analog input for the pressure sensor
 *
 * Skate lifted: blue lights
 * Stationary: multi-colored slow pulse
 * Forward (nominal): green in front fading towards back
 * Forward (leaned): same as forward nominal but rotated to direction of lean, relative to lean amount
 * Heel braking: red in back, fading towards front
 * Lunge/power Stops: red everywhere
 * Backward (nominal): green in back fading towards front
 * Backward (leaned): same as backward nominal but rotated to direction of lean, relative to lean amount
 * Toe only: Purple or something in front fading to back
 * Heel only: Yellow or something in back fading to front
 */

#include "debug.h"
#include "skate_params.h"
#include "skate_imu.h"
#include "skate_pressure.h"
#include "skate_leds.h"

constexpr uint32_t PERIOD_US = 10000; // 100 Hz
uint32_t nextTickUs;

class SkateState {
  double roll_deg;
  double pitch_deg;
  double yaw_deg;
  double forward_vel_m_s;
  double forward_acc_m_s_s;
  double down_pressure_n;
};

// sensors
SkateImu skate_imu = SkateImu();
SkatePressure skate_pressure = SkatePressure();
// signals
SkateLeds skate_leds = SkateLeds();

void setup() {
  initializeSerial();
  initializeSensors();
  initializeSignals();
  nextTickUs = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextTickUs) >= 0) {
    nextTickUs += PERIOD_US;
    loopTask();
  }
}

void loopTask() {
  readSensors();
  fuseSensorData();
  sendCommands();
}

void initializeSerial() {
  DEBUG_BEGIN(SERIAL_BAUD_RATE);
  while (!Serial) { delay(10); }
  DEBUG_PRINTLN("Serial ready");
}

void initializeSensors() {
  skate_imu.initialize();
}

void initializeSignals() {
  skate_leds.initialize();
}

void readSensors() {
  skate_imu.process();
  skate_pressure.process();
}

void fuseSensorData() {

}

void sendCommands() {

}


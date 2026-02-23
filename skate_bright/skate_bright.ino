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
#include "skate_sensor_fusion.h"
#include "skate_leds.h"

uint32_t nextTickUs;

// sensors
SkateImu skateImu = SkateImu();
SkatePressure skatePressure = SkatePressure();
// state
SkateSensorFusion skateSensorFusion = SkateSensorFusion();
// signals
SkateLeds skateLeds = SkateLeds();

/**
 * Setup function. Run once at startup.
 * Initializes state, Serial port, sensors, and signals.
 */
void setup() {
  initializeSerial();
  DEBUG_PRINTLN("[setup]");
  initializeState();
  // initializeSignals();
  initializeSensors();
  nextTickUs = micros();
}

/**
 * Main update loop. Runs continuously, but data and commands are
 * processed based on predefined loop rate.
 */
void loop() {
  DEBUG_PRINTLN("[loop]");
  uint32_t now = micros();
  if ((int32_t)(now - nextTickUs) >= 0) {
    // DEBUG_PRINT("now: "); DEBUG_PRINTLN(now);
    nextTickUs += LOOP_DELAY_US;
    loopTask();
  }
}

/**
 * Function that gets run every update.
 * Reads sensors and sends commands.
 */
void loopTask() {
  bool gotNewData = readSensors();
  fuseSensorData(gotNewData);
  // sendCommands();
  printSensorsState();
}

/**
 * Initializes state.
 */
void initializeState() {
  skateSensorFusion.initialize();
}

void initializeSerial() {
  Serial.begin(115200);
  while (!Serial) delay(10);
}

/**
 * Initializes sensors.
 */
void initializeSensors() {
  skateImu.initialize();
  skatePressure.initialize();
}

/**
 * Initializes signals, like LED strip.
 */
void initializeSignals() {
  skateLeds.initialize();
}

/**
 * Reads all sensors.
 * @returns true if new sensor data acquired, false otherwise.
 */
bool readSensors() {
  bool newImuData = skateImu.process();
  bool newPressureData = skatePressure.process();
  return newImuData || newPressureData;
}

/**
 * Processes sensor data if new data exists.
 */
void fuseSensorData(bool gotNewData) {
  if (!gotNewData) return;

  skateSensorFusion.update(skateImu, skatePressure);
}

void printSensorsState() {
  Serial.println("Sensors:");
  // Orientation
  Serial.print("roll: "); Serial.print(skateSensorFusion.filteredSensors.rpyRad.x);
  Serial.print(", pitch: "); Serial.print(skateSensorFusion.filteredSensors.rpyRad.y);
  Serial.print(", yaw: "); Serial.println(skateSensorFusion.filteredSensors.rpyRad.z);

  // Acceleration
  Serial.print("ax: "); Serial.print(skateSensorFusion.filteredSensors.accMps2.x);
  Serial.print(", ay: "); Serial.print(skateSensorFusion.filteredSensors.accMps2.y);
  Serial.print(", ay: "); Serial.println(skateSensorFusion.filteredSensors.accMps2.y);

  // Pressure
  Serial.print("pressure: "); Serial.println(skateSensorFusion.filteredSensors.pressure);
}

void printSkateState() {

}

/**
 * Sends commands to signals, like LED strip.
 */
void sendCommands() {
  skateLeds.processLeds();
}


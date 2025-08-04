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

#include <Adafruit_BNO08x_RVC.h> // for IMU
#include <Adafruit_NeoPixel.h> // for LED strip

// General Constants
const float V_REF = 3.3;

// IMU Constants
#define SERIAL_BAUD_RATE     115200;

// Heel Pressure Sensor Constants
#define PRESSURE_SENSOR_PIN  A9;
const float V_REF = 3.3;
const float ADC_RES_BITS = 10.0;

// NeoPixel LED Strip Constants
#define NUM_LEDS 32;
#define LED_STRIP_PIN D0;

enum {
  SKATE_STATIONARY = 0,
  SKATE_MOVING,
  

} SkateState_t;

typedef struct {
  BNO8x_RVC_Data heading;
  int heelPressure;
} SensorData_t;

typedef struct {

} CmdData_t;

typedef struct {
  SensorData_t sensors;
  SkateState_t skateState;
  SkateState_t lastSkateState;
} StateData_t;

// Global variables for IMU
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
// Global variables for NeoPixel LEDs
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
StateData_t state;
CmdData_t cmd;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  readSensors();
  computeSkateStateFromSensorData();
  sendCommands();
}

void readSensors() {

}

void computeSkateStateFromSensorData() {
  switch (state.skateState) {
    case SKATE_STATIONARY:
      if (state.sensors.)
      break;
    default:
      Serial.println("Unknown Skate state %d.", state.skateState);
      break;
  }
}

void sendCommands() {

}

void setUpHeelSensor() {
  // Set internal pullup resistor for sensor pin
  pinMode(9, INPUT_PULLUP);
}

void setUpImu() {
  // Wait for serial monitor to open
}

void waitForSerialMonitorToOpen() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Adafruit BNO08x IMU - UART-RVC mode");
}

void readImu() {
  BNO8x_RVC_Data heading;

  if (!rvc.read(&heading)) {
    return;
  }
}

void setSkateState(SkateState_t state) {
  Serial.println("Transitioning from %s to %s.")
  state.lastSkateState = state.skateState;
  state.skateState = state;
}

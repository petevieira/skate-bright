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
#define LED_COUNT 5;
#define LED_STRIP_PIN 7; // digital pin 7
uint32_t magent = strip.Color(200, 0, 200);

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
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
StateData_t state;
CmdData_t cmd;
unsigned long time;
int loopDeltaMs = 500; // 500ms = 0.5s
bool ledsState = false;

void setup() {
  // put your setup code here, to run once:
  initializeLedStrips();
  time = millis();

}

void loop() {
  // readSensors();
  // computeSkateStateFromSensorData();
  // sendCommands();
  unsigned long dt = millis() - time;
  if (dt >= loopDeltaMs) {
    toggleLeds();
    time = millis();
  }
}

void initializeLedStrips() {
  // prepare data pin for NeoPixel strip
  strip.begin();
  // initalize all pixels to 'off', since no colors set yet
  strip.show();
  ledsState = false;
}

void toggleLeds() {
  if (ledsState) {
    turnOfLeds();
  } else {
    turnOnLeds();
  }
}

void turnOnLeds() {
  strip.fill(magenta, 0);
  strip.show();
}

void turnOffLeds() {
  strip.clear();
  strip.show();
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

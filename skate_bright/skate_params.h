#ifndef SKATE_PARAMS_H
#define SKATE_PARAMS_H

#define REAR_LEDS_COUNT     0
#define INSIDE_LEDS_COUNT   0
#define FRONT_LEDS_COUNT    1
#define OUTSIDE_LEDS_COUNT  1
#define SKATE_LED_COUNT     (REAR_LEDS_COUNT + INSIDE_LEDS_COUNT + FRONT_LEDS_COUNT + OUTSIDE_LEDS_COUNT)
#define LED_STRIP_PIN       6
#define HEEL_BRAKE_ON       true
// IMU params
#define IMU_I2C_ADDR        0x69
#define IMU_ACCELEROMETER_RANGE 0b0101
#define IMU_ACC_SENSITIVITY_COUNTS_PER_G     16384
#define GRAVITY_MPS2        9.80665
#define IMU_GYRO_SENSITIVITY_COUNTS_PER_DEG_PER_S  16.4
#define IMU_UART_BAUD_RATE  115200
#define IMU_REPORT_RATE_HZ  50
#define IMU_REPORT_RATE_US  (1000000 / IMU_REPORT_RATE_HZ)
#define IMU_RESET_PIN       4
#define IMU_CALIBRATE_PIN   3
// Sensor fusion params
// Deadband values
#define VELOCITY_DEADBAND_MPS 0.001
#define ACCERELATION_DEADBAND_MPS2 0.001
#define ANGLE_DEADBAND_RAD 2 * M_PI / 180
#define ACCELERATION_FILTER_TIME_CONSTANT 0.1
#define ORIENTATION_DEADBAND_DEG 3
#define PRESSURE_FILTER_TIME_CONSTANT 0.001
#define MIN_PRESSURE_N 5

constexpr uint32_t LOOP_RATE_HZ  = 10;
constexpr float    LOOP_DELAY_S  = 1.0f / LOOP_RATE_HZ;
constexpr float    LOOP_DELAY_MS = 1000.0f / LOOP_RATE_HZ;
constexpr float    LOOP_DELAY_US = 1000000.0f / LOOP_RATE_HZ;

class Vec3i {
public:
  int x, y, z;
};

class Vec3f {
public:
  float x, y, z;

  Vec3f(): x(0), y(0), z(0) {}
  Vec3f(float x, float y, float z): x(x), y(y), z(z) {}
};

#endif

#ifndef SKATE_PARAMS_H
#define SKATE_PARAMS_H

#define PROTOCOL_I2C        1
#define PROTOCOL_UART       2

#define __PROTOCOL__ PROTOCOL_I2C

#ifndef __PROTOCOL__
  #error "__PROTOCOL__ must be defined."
#endif

#define REAR_LEDS_COUNT     1
#define INSIDE_LEDS_COUNT   1
#define FRONT_LEDS_COUNT    1
#define OUTSIDE_LEDS_COUNT  1
#define SKATE_LED_COUNT     (REAR_LEDS_COUNT + INSIDE_LEDS_COUNT + FRONT_LEDS_COUNT + OUTSIDE_LEDS_COUNT)
#define LED_STRIP_PIN       6
#define HEEL_BRAKE_ON       true
// IMU params
#define IMU_UART_BAUD_RATE  115200
#define IMU_REPORT_RATE_HZ  50
#define IMU_REPORT_RATE_US  (1000000 / IMU_REPORT_RATE_HZ)
#define IMU_RESET_PIN       4
#define IMU_CALIBRATE_PIN   3

constexpr uint32_t LOOP_RATE_HZ  = 100;
constexpr uint32_t LOOP_DELAY_MS = 1000 / LOOP_RATE_HZ;
constexpr uint32_t LOOP_DELAY_US = 1000000 / LOOP_RATE_HZ;

#endif

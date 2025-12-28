#ifndef SKATE_LEDS_H
#define SKATE_LEDS_H

#include <Adafruit_NeoPixel.h> // for LED strip

#include "skate_params.h"

class SkateLeds {
public:
  // Argument 1 = Number of pixels in NeoPixel strip
  // Argument 2 = Arduino pin number (most are valid)
  // Argument 3 = Pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
  Adafruit_NeoPixel strip{SKATE_LED_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800};
  bool ledsOn[SKATE_LED_COUNT] = { false };
  bool ledsState = false;
  uint32_t magenta = strip.Color(255, 0, 255);

  void initialize();

  void processLeds();

  void toggleLeds();

  void turnOnLeds();

  void turnOffLeds();

private:
  unsigned long time_prev_ms;
};

#endif
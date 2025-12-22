/**
 */

#include "skate_leds.h"

void SkateLeds::initialize() {
  // prepare data pin for NeoPixel strip
  strip.begin();

  // initalize all pixels to 'off', since no colors set yet
  strip.show();
}

void SkateLeds::processLeds() {
  toggleLeds();
}

void SkateLeds::toggleLeds() {
  if (leds_state) {
    turnOffLeds();
  } else {
    turnOnLeds();
  }
  strip.show();
}

void SkateLeds::turnOnLeds() {
  strip.fill(magenta, 0);
  leds_state = true;
}

void SkateLeds::turnOffLeds() {
  strip.clear();
  leds_state = false;
}
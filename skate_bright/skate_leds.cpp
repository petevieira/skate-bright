/**
 */

#include "debug.h"
#include "skate_leds.h"

SkateLeds::~SkateLeds() {
  turnOffLeds();
  strip.show();
}

void SkateLeds::initialize() {
  DEBUG_PRINTLN("[SkateLeds::initialize]");
  // prepare data pin for NeoPixel strip
  strip.begin();

  // initalize all pixels to 'off', since no colors set yet
  turnOffLeds();
  strip.show();
}

void SkateLeds::processLeds() {
  toggleLeds();
}

void SkateLeds::toggleLeds() {
  DEBUG_PRINTLN("[toggleLeds]");
  if (ledsState) {
    turnOffLeds();
  } else {
    turnOnLeds();
  }
  strip.show();
}

void SkateLeds::turnOnLeds() {
  strip.fill(magenta, 0);
  ledsState = true;
}

void SkateLeds::turnOffLeds() {
  strip.clear();
  ledsState = false;
}
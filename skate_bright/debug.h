#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#define DEBUG_ENABLED 0

#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, f) Serial.print(x, f)
  #define DEBUG_PRINTLNF(x, f) Serial.println(x, f)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, f)
  #define DEBUG_PRINTLNF(x, f)
#endif

#endif
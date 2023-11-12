#ifndef WARNING_DETECTION_H
#define WARNING_DETECTION_H

#include <Arduino.h>

void setupWarningLed();
void warningLed(uint8_t color);
uint8_t getActualWarningColor();
#endif

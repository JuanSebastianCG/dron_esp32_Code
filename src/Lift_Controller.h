#ifndef LIFT_CONTROLLER_H
#define LIFT_CONTROLLER_H
#include <Arduino.h>


extern uint16_t minLimitMotor;
extern uint16_t heightWanted;

void getAltitudeActual(float actualHeight);
void getAltitudeWanted(uint8_t key, int16_t value1, int16_t value2);


#endif

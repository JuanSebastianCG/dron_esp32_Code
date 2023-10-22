#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <PID_v1.h>

void setupMotors();
void controllBalance(float y,float x);
void moveMotors();
void moveMotorsXbox(uint8_t key, int16_t value1, int16_t value2); 


#endif

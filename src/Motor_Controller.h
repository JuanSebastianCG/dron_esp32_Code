#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <PID_v1.h>

extern bool stabilize;
extern uint8_t motorSpeed1;
extern uint8_t motorSpeed2;
extern uint8_t motorSpeed3;
extern uint8_t motorSpeed4;

void moveMotorsXbox(uint8_t key, int16_t value1, int16_t value2); 
void controllBalance(float y,float x);
void setupMotors();
void moveMotors();


#endif


#include <Arduino.h>
#include "Motor_Controller.h"

/* int minLimitMotor = 0;
int maxLimitMotor = 155; */

uint8_t minLimitMotor = 100;
uint8_t maxLimitMotor = 200;
u_int8_t minPowerHeight = 0;

uint16_t heightWanted = 0;
uint16_t heightActual = 0;

uint8_t diferenceMargin = 1;

void getAltitudeWanted(uint8_t key, int16_t value1, int16_t value2)
{
    value2 = map(value2, -32768, 32767, 0, 255);
    if (key == 17 || key == 18)
    {
        if(value2 > 10 && value2 < 255)
        {
            heightWanted = value2;
        }
        else if(value2 < 10 && value2 > 0)
        {
            heightWanted = value2;
        }
        
    }
}

void getAltitudeActual(float zAceleration)
{ 
  

}


#ifndef MPU_6050_CONECTION_H
#define MPU_6050_CONECTION_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

extern Adafruit_MPU6050 mpu;

void connectMPU6050( void (*callback)(sensors_event_t a, sensors_event_t g, sensors_event_t temp));
void readMPU6050() ;

#endif

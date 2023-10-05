#include "Mpu_6050_Conection.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
void (*sensors_eventCallBack)(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void readMPU6050();

void connectMPU6050( void (*callback)(sensors_event_t a, sensors_event_t g, sensors_event_t temp)) {
  
  sensors_eventCallBack = callback;
  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    while (1) {
    Serial.println("Failed to find MPU6050 chip");
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void readMPU6050() {
  if(mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sensors_eventCallBack(a, g, temp);
  }
}

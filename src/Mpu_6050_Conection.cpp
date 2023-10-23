#include "Mpu_6050_Conection.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

int16_t ax, ay, az, gx, gy, gz;
float ypr[3];

Adafruit_MPU6050 mpu;
void (*sensors_eventCallBack)(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void readMPU6050();
void readMpuMotion();
void calculateOrientation();

void readMpuMotion() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ax = static_cast<int16_t>(a.acceleration.x);
  ay = static_cast<int16_t>(a.acceleration.y);
  az = static_cast<int16_t>(a.acceleration.z);

  gx = static_cast<int16_t>(g.gyro.x);
  gy = static_cast<int16_t>(g.gyro.y);
  gz = static_cast<int16_t>(g.gyro.z);
}

void calculateOrientation() {
  float pitch, roll, yaw;
  readMpuMotion();
  pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  roll = atan2(-ax, az) * 180 / PI;
  yaw = atan2(az, sqrt(ax * ax + az * az)) * 180 / PI;
  ypr[0] = yaw;
  ypr[1] = pitch;
  ypr[2] = roll;

  //Print values for plotter
  Serial.print("yaw:");
  Serial.print(yaw);
  Serial.print(", pitch:");
  Serial.print(pitch);
  Serial.print(", roll:");
  Serial.println(roll);
}

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

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);//21Hz

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void readMPU6050() {
  //if(mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sensors_eventCallBack(a, g, temp);
   //if error reading connectMpu Again
  
  //}
}

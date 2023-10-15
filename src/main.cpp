#include <Arduino.h>

#include "Wifi_Connection.h"
#include "Mpu_6050_Conection.h"

// Definición de pines para el motor
#define MOTOR_PIN1 15
#define MOTOR_PIN2 13
#define MOTOR_PIN3 32
#define MOTOR_PIN4 19

const int channel1 = 0;
const int channel2 = 1;
const int channel3 = 2;
const int channel4 = 3;

const int frequency = 5000;
const int resolution = 8;


void processMpu6050(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void procesUdp(uint8_t key, int16_t value1, int16_t value2);
void controllBalance(float y );
void printData(sensors_event_t a, sensors_event_t g, sensors_event_t temp);

const unsigned long period = 500; // 1 segundo
unsigned long current = 0;
unsigned long start = 0;

void setup() {
  Serial.begin(115200);

  ledcSetup(channel1, frequency, resolution);
  ledcAttachPin(MOTOR_PIN1, channel1);

  ledcSetup(channel2, frequency, resolution);
  ledcAttachPin(MOTOR_PIN2, channel2);

  ledcSetup(channel3, frequency, resolution);
  ledcAttachPin(MOTOR_PIN3, channel3);

  ledcSetup(channel4, frequency, resolution);
  ledcAttachPin(MOTOR_PIN4, channel4);

  //conect to wifi
  //wifiConnection("ssid", "password", procesUdp);

  //conect to mpu6050
  connectMPU6050(processMpu6050);


}

void loop() {
  //processUDPData();
  readMPU6050();
  
}

void printData(sensors_event_t a, sensors_event_t g, sensors_event_t temp) {
  Serial.print("Aceleración: ");
  Serial.print(a.acceleration.x);
  Serial.print(" ");
  Serial.print(a.acceleration.y);
  Serial.print(" ");
  Serial.println(a.acceleration.z);

/*   Serial.print("Giro: ");
  Serial.print(g.gyro.x);
  Serial.print(" ");
  Serial.print(g.gyro.y);
  Serial.print(" ");
  Serial.println(g.gyro.z); */
}

void processMpu6050(sensors_event_t a, sensors_event_t g, sensors_event_t temp) {
  controllBalance(a.acceleration.y);
  current = millis();
  if (current - start >= period) {
    start = current;
    printData(a, g, temp);
  }
}

void controllBalance(float y ){
  const int stableVel = 80;

  if (y > 0.3){
    ledcWrite(channel1, stableVel);
    ledcWrite(channel4, 0);
  }else if (y < 0.1){
    ledcWrite(channel1, 0);
    ledcWrite(channel4, stableVel);
  } else {
    ledcWrite(channel1, stableVel);
    ledcWrite(channel4, stableVel);
  }



}
void procesUdp(uint8_t key, int16_t value1, int16_t value2){


}


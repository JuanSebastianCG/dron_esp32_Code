#include <Arduino.h>
#include <PID_v1.h>
#include "Wifi_Connection.h"
#include "Mpu_6050_Conection.h"
#include "Motor_Controller.h"
#include "WarningDetection.h"

const int8_t port = 80;

//=========================Setup=========================

void processMpu6050(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void procesUdp(uint8_t key, int16_t value1, int16_t value2);
void warningLed(uint8_t color);

void setup()
{
  Serial.begin(115200);
  setupWarningLed();

  // --------------connect to Wi-Fi---------------
  warningLed(1);
  delay(400);
  wifiConnection("ANPARO..", "24280650", procesUdp);
  // wifiConnection("Sebas", "Vienna22*", procesUdp);
  // wifiConnection("AndroidAPB943", "vtqz9627", procesUdp);
  // wifiConnection("redjuan", "juan4321", procesUdp);

  // ---------------Connect to MPU6050---------------
  warningLed(2);
  delay(400);
  connectMPU6050(processMpu6050);

  // ---------------Initialize PID controllers------------
  warningLed(3);
  delay(400);
  setupMotors();

  //---------------- start ---------------
}

//================================OPERATION================================
void loop()
{
  // Process UDP data if needed
  processUDPData();

  // Read MPU6050 data
  readMPU6050();

  // mover los motores
  moveMotors();
}

void processMpu6050(sensors_event_t a, sensors_event_t g, sensors_event_t temp)
{
  // Use acceleration data to control balance
  controllBalance(a.acceleration.y, a.acceleration.x);
}

void procesUdp(uint8_t key, int16_t value1, int16_t value2)
{
  // Use UDP data to control balance
  moveMotorsXbox(key, value1, value2);
}

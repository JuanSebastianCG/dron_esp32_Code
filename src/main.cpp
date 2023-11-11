#include <Arduino.h>
#include <PID_v1.h>
#include "Wifi_Connection.h"
#include "Mpu_6050_Conection.h"
#include "Motor_Controller.h"


//led rgb to show the state of the drone
uint8_t LEDR = 4;
uint8_t LEDG = 16;
uint8_t LEDB = 17;
uint8_t LEDW = 2;

const int8_t port = 80; 

//=========================Setup=========================

void processMpu6050(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void procesUdp(uint8_t key, int16_t value1, int16_t value2);

void setup() {
  Serial.begin(115200);

  //initialize the led
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDW, OUTPUT);
  digitalWrite(LEDW, HIGH);

  // --------------connect to Wi-Fi---------------
  //put light to blue 
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  delay(500);

  wifiConnection("ANPARO..", "24280650", procesUdp);
  //wifiConnection("Sebas", "Vienna22*", procesUdp);
  //wifiConnection("AndroidAPB943", "vtqz9627", procesUdp);
  //wifiConnection("redjuan", "juan4321", procesUdp);

  // ---------------Connect to MPU6050---------------
  //put light to green
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);
  delay(500);

  // Connect to MPU6050
  connectMPU6050(processMpu6050);

  // ---------------Initialize PID controllers------------
  //put light to red
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  delay(500);

  // Initialize PID controllers
  setupMotors();

  //---------------= start ---------------
  // put light to blue
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);



}


//================================OPERATION================================
void loop() {
  // Process UDP data if needed
  processUDPData();

  // Read MPU6050 data
  readMPU6050();

  //mover los motores
  moveMotors();

}

void processMpu6050(sensors_event_t a, sensors_event_t g, sensors_event_t temp) {
  // Use acceleration data to control balance
  controllBalance(a.acceleration.y, a.acceleration.x);
}

void procesUdp(uint8_t key, int16_t value1, int16_t value2) {
  // Use UDP data to control balance
  moveMotorsXbox(key, value1, value2);

}

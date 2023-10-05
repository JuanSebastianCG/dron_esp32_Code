#include <Arduino.h>
#include <ESP32Servo.h>

#include "Wifi_Connection.h"
#include "Mpu_6050_Conection.h"

#define SERVO_PIN1 15 // ESP32 pin GPIO26 connected to servo motor
#define SERVO_PIN2 2  // ESP32 pin GPIO26 connected to servo motor
#define SERVO_PIN3 4  // ESP32 pin GPIO26 connected to servo motor

Servo servoMotor1;
Servo servoMotor2;
Servo servoMotor3;

// void processData(uint8_t key, int16_t value1, int16_t value2);}
void readSensor(sensors_event_t a, sensors_event_t g, sensors_event_t temp);

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting...");

    servoMotor1.attach(SERVO_PIN1);
    servoMotor2.attach(SERVO_PIN2);
    servoMotor3.attach(SERVO_PIN3);

    connectMPU6050(readSensor);

    // wifiConnection("ANPARO..", "24280650", processData);
    // Serial.println("WiFi connected.");
}

void loop()
{
    readMPU6050();
    //processUDPData();

}

/* void processData(uint8_t key, int16_t value1, int16_t value2)
{
} */

void readSensor(sensors_event_t a, sensors_event_t g, sensors_event_t temp)
{


    /* int servoPosition = map(a.acceleration.x ,-10, 10, 0, 180);
    servoMotor1.write(servoPosition);

    int servoPositio1 = map(a.acceleration.y, -10, 10, 0, 180);
    servoMotor1.write(servoPositio1);

    int servoPosition2 = map(a.acceleration.z, -10, 10, 0, 180);
    servoMotor1.write(servoPosition2 */

}
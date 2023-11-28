#include <Arduino.h>
#include <PID_v1.h>
#include "Lift_Controller.h"
#include "WarningDetection.h"

void controllBalance(float y, float x);
void moveMotorsXbox(uint8_t key, int16_t value1, int16_t value2);
void moveMotors();
void setupMotors();

//=======================PWM===================

// Define pins for the motors
#define MOTOR_PIN1 15
#define MOTOR_PIN2 13
#define MOTOR_PIN3 32
#define MOTOR_PIN4 19

// Define channels and PWM settings
const uint8_t channel1 = 0;
const uint8_t channel2 = 1;
const uint8_t channel3 = 2;
const uint8_t channel4 = 3;

const uint32_t frequency = 5000;
const uint8_t resolution = 8;

const float Kp = 2.0, Ki = 0.01, Kd = 0.1; 
//====================pidy===================

float stableYSetPoint = 0.4;                 // Set the setpoint to 0

double Setpointy1 = stableYSetPoint, Inputy1, Outputy1;
float Kpy1 = Kp, Kiy1 = Ki, Kdy1 = Kd;
PID pid1(&Inputy1, &Outputy1, &Setpointy1, Kpy1, Kiy1, Kdy1, DIRECT);

double Setpointy2 = stableYSetPoint, Inputy2, Outputy2;
float Kpy2 = Kp, Kiy2 = Ki, Kdy2 = Kd;
PID pid2(&Inputy2, &Outputy2, &Setpointy2, Kpy2, Kiy2, Kdy2, DIRECT);

double Setpointy3 = stableYSetPoint, Inputy3, Outputy3;
float Kpy3 = Kp, Kiy3 = Ki, Kdy3 = Kd;
PID pid3(&Inputy3, &Outputy3, &Setpointy3, Kpy3, Kiy3, Kdy3, DIRECT);

double Setpointy4 = stableYSetPoint, Inputy4, Outputy4;
float Kpy4 = Kp, Kiy4 = Ki, Kdy4 = Kd;
PID pid4(&Inputy4, &Outputy4, &Setpointy4, Kpy4, Kiy4, Kdy4, DIRECT);

//===================pidx========================

float stableXSetPoint = 0.4;                 // Set the setpoint to 0

double Setpointx1 = stableXSetPoint, Inputx1, Outputx1;
float Kpx1 = Kp, Kix1 = Ki, Kdx1 = Kd;
PID pid5(&Inputx1, &Outputx1, &Setpointx1, Kpx1, Kix1, Kdx1, DIRECT);

double Setpointx2 = stableXSetPoint, Inputx2, Outputx2;
float Kpx2 = Kp, Kix2 = Ki, Kdx2 = Kd;
PID pid6(&Inputx2, &Outputx2, &Setpointx2, Kpx2, Kix2, Kdx2, DIRECT);

double Setpointx3 = stableXSetPoint, Inputx3, Outputx3;
float Kpx3 = Kp, Kix3 = Ki, Kdx3 = Kd;
PID pid7(&Inputx3, &Outputx3, &Setpointx3, Kpx3, Kix3, Kdx3, DIRECT);

double Setpointx4 = stableXSetPoint, Inputx4, Outputx4;
float Kpx4 = Kp, Kix4 = Ki, Kdx4 = Kd;
PID pid8(&Inputx4, &Outputx4, &Setpointx4, Kpx4, Kix4, Kdx4, DIRECT);

//============================== const =============================

const short minInputPID = -255;
const short maxInputPID = 255;

uint8_t motorSpeed1 = 0;
uint8_t motorSpeed2 = 0;
uint8_t motorSpeed3 = 0;
uint8_t motorSpeed4 = 0;

bool stabilize = false;
int yEvaluate = 0;

//============================== functions =============================

void setupMotors()
{
  const float responsePID = 0.01;

  // Set up PWM for motor control
  ledcSetup(channel1, frequency, resolution);
  ledcAttachPin(MOTOR_PIN1, channel1);
  ledcSetup(channel2, frequency, resolution);
  ledcAttachPin(MOTOR_PIN2, channel2);
  ledcSetup(channel3, frequency, resolution);
  ledcAttachPin(MOTOR_PIN3, channel3);
  ledcSetup(channel4, frequency, resolution);
  ledcAttachPin(MOTOR_PIN4, channel4);

  // Initialize PID controllers
  pid1.SetMode(AUTOMATIC);
  pid1.SetSampleTime(responsePID);
  pid1.SetOutputLimits(minInputPID, maxInputPID);

  pid2.SetMode(AUTOMATIC);
  pid2.SetSampleTime(responsePID);
  pid2.SetOutputLimits(minInputPID, maxInputPID);

  pid3.SetMode(AUTOMATIC);
  pid3.SetSampleTime(responsePID);
  pid3.SetOutputLimits(minInputPID, maxInputPID);

  pid4.SetMode(AUTOMATIC);
  pid4.SetSampleTime(responsePID);
  pid4.SetOutputLimits(minInputPID, maxInputPID);

  pid5.SetMode(AUTOMATIC);
  pid5.SetSampleTime(responsePID);
  pid5.SetOutputLimits(minInputPID, maxInputPID);

  pid6.SetMode(AUTOMATIC);
  pid6.SetSampleTime(responsePID);
  pid6.SetOutputLimits(minInputPID, maxInputPID);

  pid7.SetMode(AUTOMATIC);
  pid7.SetSampleTime(responsePID);
  pid7.SetOutputLimits(minInputPID, maxInputPID);

  pid8.SetMode(AUTOMATIC);
  pid8.SetSampleTime(responsePID);
  pid8.SetOutputLimits(minInputPID, maxInputPID);
}

void controllBalance(float y, float x)
{
  if (stabilize)
  {
    // Read MPU6050 data
    Inputy1 = y;
    Inputy2 = y;
    Inputy3 = y;
    Inputy4 = y;

    Inputx1 = x;
    Inputx2 = x;
    Inputx3 = x;
    Inputx4 = x;

    pid1.Compute();
    pid2.Compute();
    pid3.Compute();
    pid4.Compute();
  
    pid5.Compute();
    pid6.Compute();
    pid7.Compute();
    pid8.Compute();

    // Ajustar la velocidad del motor 1 proporcionalmente a "Output1"
    
     motorSpeed1 = map(Outputy1+Outputx1*-1+heightWanted, -255, 255, 0, 255);
     motorSpeed2 = map(Outputy2+Outputx2+heightWanted, -255, 255, 0, 255);
      motorSpeed3 = map(Outputy3*-1+Outputx3+heightWanted, -255, 255, 0, 255);
      motorSpeed4 = map(Outputy4*-1+Outputx4*-1+heightWanted, -255, 255, 0, 255);
   

  }
  else
  {
    motorSpeed1 = 0;
    motorSpeed2 = 0;
    motorSpeed3 = 0;
    motorSpeed4 = 0;
  }
   printf("Motor 1: %d, Motor 2: %d, Motor 3: %d, Motor 4: %d\n", motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4);
}

void moveMotors()

{
  ledcWrite(channel1, motorSpeed1);
  ledcWrite(channel2, motorSpeed2);
  ledcWrite(channel3, motorSpeed3);
  ledcWrite(channel4, motorSpeed4);
}

//============================XBox Controller============================
void moveMotorsXbox(uint8_t key, int16_t value1, int16_t value2)
{
  yEvaluate = map(value1, -32768, 32767, -100, 100);
  if (key == 1){
    stabilize = !stabilize;
    Serial.println(stabilize);
  }


  if (key == 16 || key == 15)
  {
    

  }
}

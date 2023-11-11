#include <Arduino.h>
#include <PID_v1.h>

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
const int channel1 = 0;
const int channel2 = 1;
const int channel3 = 2;
const int channel4 = 3;

const int frequency = 5000;
const int resolution = 8;

//====================pid===================

const float Kp = 2.0, Ki = 0.01, Kd = 0.001; // Increase Kp
float stableYSetPoint = 0.4;                 // Set the setpoint to 0

double Setpoint1 = stableYSetPoint, Input1, Output1;
float Kp1 = Kp, Ki1 = Ki, Kd1 = Kd;
PID pid1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

double Setpoint2 = stableYSetPoint, Input2, Output2;
float Kp2 = Kp, Ki2 = Ki, Kd2 = Kd;
PID pid2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

double Setpoint3 = stableYSetPoint, Input3, Output3;
float Kp3 = Kp, Ki3 = Ki, Kd3 = Kd;
PID pid3(&Input3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);

double Setpoint4 = stableYSetPoint, Input4, Output4;
float Kp4 = Kp, Ki4 = Ki, Kd4 = Kd;
PID pid4(&Input4, &Output4, &Setpoint4, Kp4, Ki4, Kd4, DIRECT);

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

const float responsePID = 0.01;
void setupMotors()
{

  // Set up PWM for motor control
  ledcSetup(channel1, frequency, resolution);
  ledcAttachPin(MOTOR_PIN1, channel1);
  ledcSetup(channel2, frequency, resolution);
  ledcAttachPin(MOTOR_PIN4, channel2);
  ledcSetup(channel3, frequency, resolution);
  ledcAttachPin(MOTOR_PIN3, channel3);
  ledcSetup(channel4, frequency, resolution);
  ledcAttachPin(MOTOR_PIN2, channel4);

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
}

void controllBalance(float y, float x)
{
  if (stabilize)
  {
    Input1 = yEvaluate;
    Input2 = yEvaluate;
    Input3 = yEvaluate;
    Input4 = yEvaluate;

    pid1.Compute();
    pid2.Compute();
    pid3.Compute();
    pid4.Compute();

    // Ajustar la velocidad del motor 1 proporcionalmente a "Output1"
    /*  motorSpeed1 = (maxLimitMotor + minLimitMotor) - map(Output1, minInputPID, maxInputPID, minLimitMotor, maxLimitMotor) + 120;
     motorSpeed2 = map(Output2, minInputPID, maxInputPID, minLimitMotor, maxLimitMotor) + 120;
     motorSpeed4 = (maxLimitMotor + minLimitMotor) - map(Output4, minInputPID, maxInputPID, minLimitMotor, maxLimitMotor) + 120;
     motorSpeed3 = map(Output3, minInputPID, maxInputPID, minLimitMotor, maxLimitMotor) + 120; */

    motorSpeed1 = 255;
    motorSpeed2 = 255;
    motorSpeed3 = 255;
    motorSpeed4 = 255;
  }
  else
  {
    motorSpeed1 = 0;
    motorSpeed2 = 0;
    motorSpeed3 = 0;
    motorSpeed4 = 0;
  }
  // printf("Motor 1: %d, Motor 2: %d, y: %f\n", motorSpeed1, motorSpeed2, y);
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
  // Serial.print(key);
  yEvaluate = map(value1, -32768, 32767, -100, 100);
  Serial.println(key);
  if (key == 1)
    stabilize = !stabilize;

  if (key == 16 || key == 15)
  {
  }
}

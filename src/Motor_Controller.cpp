#include <Arduino.h>
#include <PID_v1.h>

// Define pins for the motors
#define MOTOR_PIN1 15
#define MOTOR_PIN2 13
#define MOTOR_PIN3 32
#define MOTOR_PIN4 19

// Define channels and PWM settings
const int channel1 = 0;
const int channel2 = 1;
const int frequency = 5000;
const int resolution = 8;

int minLimitMotor = 0;
int maxLimitMotor = 155;

double minInput = -255;
double maxInput = 255;

const double Kp = 2.0, Ki = 0.01, Kd = 0.001; // Increase Kp
double stableYSetPoint = 0.4;                 // Set the setpoint to 0

// Create PID controllers for each motor
double Setpoint1 = stableYSetPoint, Input1, Output1;
double Kp1 = Kp, Ki1 = Ki, Kd1 = Kd;
PID pid1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

double Setpoint2 = stableYSetPoint, Input2, Output2;
double Kp2 = Kp, Ki2 = Ki, Kd2 = Kd;
PID pid2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

bool stabilize = false;

int motorSpeed1 = 0;
int motorSpeed2 = 0;

void controllBalance(float y, float x);

void setupMotors()
{
  Serial.begin(115200);

  // Set up PWM for motor control
  ledcSetup(channel1, frequency, resolution);
  ledcAttachPin(MOTOR_PIN1, channel1);
  ledcSetup(channel2, frequency, resolution);
  ledcAttachPin(MOTOR_PIN4, channel2);

  // Initialize PID controllers
  pid1.SetMode(AUTOMATIC);
  pid1.SetSampleTime(0.01);
  pid1.SetOutputLimits(minInput, maxInput);

  pid2.SetMode(AUTOMATIC);
  pid2.SetSampleTime(0.01);
  pid2.SetOutputLimits(minInput, maxInput);
}

void controllBalance(float y, float x)
{
  if (stabilize)
  {
    Input1 = y;
    Input2 = y;

    pid1.Compute();
    pid2.Compute();

    // Ajustar la velocidad del motor 1 proporcionalmente a "Output1"
    motorSpeed1 = (maxLimitMotor + minLimitMotor) - map(Output1, minInput, maxInput, minLimitMotor, maxLimitMotor);
    motorSpeed2 = map(Output2, minInput, maxInput, minLimitMotor, maxLimitMotor);
  }
  // printf("Motor 1: %d, Motor 2: %d, y: %f\n", motorSpeed1, motorSpeed2, y);
}

void moveMotors()
{
  ledcWrite(channel1, motorSpeed1);
  ledcWrite(channel2, motorSpeed2);
}

//============================XBox Controller============================
void moveMotorsXbox(uint8_t key, int16_t value1, int16_t value2)
{

  value1 = map(value1, -32768, 32767, -100, 100);
  int errorValueLimit[] = {-15, 15};

  if (key == 16 || key == 15)
  {
    if (value1 > 0)
    {
      motorSpeed2 = value1;
      motorSpeed1 = -value1+100;
    }
    else
    {
      motorSpeed2 = -value1;
      motorSpeed1 = value1 + 100;
    }

  


  }
  
  //printf("Motor 1: %d, Motor 2: %d, y: %f\n", motorSpeed1, motorSpeed2, value1);

  if (key == 1)
  {
    stabilize = true;
  }
}

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

int minInput = -255;
int maxInput = 255;

const double Kp =1.0, Ki = 0.01, Kd = 0.001;  // Increase Kp
double setpoity = 0.4;  // Set the setpoint to 0

// Create PID controllers for each motor
double Setpoint1 = setpoity, Input1, Output1;
double Kp1 = Kp, Ki1 = Ki, Kd1 = Kd;
PID pid1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);

double Setpoint2 = setpoity, Input2, Output2;
double Kp2 = Kp, Ki2 = Ki, Kd2 = Kd;
PID pid2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

// Define the stable velocity variable
const double stableVelocity = 100.0; 
const double stableSetpoint = 0.0;

void controllBalance(float y, float x);

void setupMotors() {
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

void controllBalance(float y, float x) {
  double exponent = 0.1; // Puedes ajustar este exponente según tu necesidad
  Input1 = y ;
  Input2 = y ;
  
  pid1.Compute();
  pid2.Compute();
  
  // Ajustar la velocidad del motor 1 proporcionalmente a "Output1"
  int motorSpeed1 = (maxLimitMotor+minLimitMotor) - map(Output1, minInput, maxInput, minLimitMotor, maxLimitMotor);
  int motorSpeed2 = map(Output2, minInput, maxInput, minLimitMotor, maxLimitMotor);


  printf("Motor 1: %d, Motor 2: %d, y: %f\n", motorSpeed1, motorSpeed2, y);

  // Actualizar el control de los motores
  ledcWrite(channel1, motorSpeed1);
  ledcWrite(channel2, motorSpeed2);
}

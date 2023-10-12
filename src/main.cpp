#include <Arduino.h>

// Definición de pines para el motor
#define MOTOR_PIN1 15
#define MOTOR_PIN2 13
#define MOTOR_PIN3 32
#define MOTOR_PIN4 19

void setup() {
  Serial.begin(115200);
  
  // Configura los pines como salidas
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_PIN3, OUTPUT);
  pinMode(MOTOR_PIN4, OUTPUT);

  // Establece el valor de PWM en 255 para los pines del motor


}
int i = 0;
void loop() {



  analogWrite(MOTOR_PIN1, i);
  analogWrite(MOTOR_PIN2, i);
  Serial.println(i++);
  if (i > 255) {
    i = 0;
  }


  analogWrite(MOTOR_PIN3, 255);
  analogWrite(MOTOR_PIN4, 255);
  delay(100);
  Serial.println(i);

  // El motor se mantendrá funcionando a velocidad máxima (255) de manera continua
}

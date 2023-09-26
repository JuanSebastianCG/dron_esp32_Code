#include <Arduino.h>
#include "Wifi_Connection.h"


const int canal = 0;
const int resolucion = 8;
const int frecuencia = 5000; 
#define SERVO 2

void processData(uint8_t key, int16_t value1, int16_t value2);

void setup() {
    Serial.begin(9600);
    ledcSetup(canal, resolucion, frecuencia);
    ledcAttachPin(SERVO, canal);
    
    // Inicializa la conexión a Internet con los parámetros ssid y password
    // wifiConnection("ANPARO..", "24280650", processData);
     //wifiConnection("Sebas", "Vienna22*", processData);
}

void loop() {
    processUDPData();
}

void processData(uint8_t key, int16_t value1, int16_t value2) {
    Serial.print("Datos recibidos:");
    Serial.print(key);
    Serial.print(", ");
    Serial.print(value1);
    Serial.print(", ");
    Serial.println(value2);
    

}

void convertJoistickTo360(int16_t value){
    int16_t angle = map(value, 0, 1023, 0, 180);
    ledcWrite(canal, angle);
}

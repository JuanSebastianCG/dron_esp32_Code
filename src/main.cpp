#include <Arduino.h>
#include "Wifi_Connection.h"


const int canal = 0;
const int resolucion = 8;
const int frecuencia = 5000; 
#define SERVO 2



void processData(uint8_t key, int16_t value);

void setup() {
    Serial.begin(9600);

    // Inicializa la conexión a Internet con los parámetros ssid y password
    // wifiConnection("ANPARO..", "24280650", processData);
     //wifiConnection("Sebas", "Vienna22*", processData);
     wifiConnection("Sebas", "Vienna22*", processData);
}

void loop() {
    processUDPData();
}

void processData(uint8_t key, int16_t value) {
    Serial.print("Datos recibidos:");
    Serial.print(key);
    Serial.print(", ");
    Serial.println(value);



}

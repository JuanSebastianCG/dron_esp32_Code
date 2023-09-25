#include <Arduino.h>
#include "Wifi_Connection.h"

void processData(String key, String value);

void setup() {
    Serial.begin(9600);

    // Inicializa la conexión a Internet con los parámetros ssid y password
    wifiConnection("ANPARO..", "24280650", processData);
}

void loop() {
    processUDPData();
}

void processData(String key, String value) {
    Serial.println("Datos recibidos:");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(value);
}

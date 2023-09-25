#include <Arduino.h>
#include "Wifi_Connection.h"
#include <ESPAsyncWebServer.h>

// Declarar las funciones
void processData(String key, String value);

void setup() {
    // Resto de tu código de configuración
    Serial.begin(9600);

    // Inicializa la conexión a Internet con los parámetros ssid y password
    wifiConection("ANPARO..", "24280650", processData);
}

void loop() {
    // Tu código principal aquí
}

void processData(String key, String value) {
    Serial.println("Datos recibidos:");
    Serial.println(key);
}

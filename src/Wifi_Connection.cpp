#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "Wifi_Connection.h"

AsyncWebServer server(80);

// Declaración del puntero a función
void (*processDataCallback)(String key, String value);

void wifiConection(const char* ssid, const char* password, void (*callback)(String key, String value)) {
    // Guarda el puntero a la función processData
    processDataCallback = callback;

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Enciende un LED de prueba (conectado al pin 13, por ejemplo)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  

    // Configura las rutas HTTP
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request){
        String key = request->arg("key");
        String value = request->arg("value");
        
        // Llama a la función de callback con la clave y el valor
        processDataCallback(key, value);

        request->send(200, "text/plain", "Datos recibidos");
    });


    // Inicia el servidor web
    server.begin();
}

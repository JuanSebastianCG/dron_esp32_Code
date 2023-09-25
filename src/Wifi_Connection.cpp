#include <Arduino.h>
#include <WiFi.h> // Asegúrate de incluir la biblioteca WiFi

#include "Wifi_Connection.h"

WiFiServer server(80);  // Inicializa server como variable global


/* WiFi connected.
IP address: 
192.168.20.72 */

void wifiConection(const char* ssid, const char* password) {
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Enciende un LED de prueba (conectado al pin 13, por ejemplo)
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // Enciende el LED

    // Inicia el servidor web en el puerto 80
    server.begin();
}

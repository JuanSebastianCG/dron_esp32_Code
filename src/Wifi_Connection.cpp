#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udpServer;

void (*processDataCallback)(String key, String value);

void wifiConnection(const char* ssid, const char* password, void (*callback)(String key, String value)) {
    processDataCallback = callback;
    WiFi.begin(ssid, password);

    // Espera hasta que se establezca la conexión Wi-Fi
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (++attempts > 10) {
            Serial.println("Error: No se pudo conectar a la red Wi-Fi");
            ESP.restart();
        }
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    

    // Configura el servidor UDP en el puerto 12345 (o el que desees)
    udpServer.begin(12345);
}

void processUDPData() {
    int packetSize = udpServer.parsePacket();
    if (packetSize) {
        char packetData[packetSize + 1];
        udpServer.read(packetData, packetSize);
        packetData[packetSize] = '\0';
        
        String dataString(packetData);
        int separatorIndex = dataString.indexOf('=');
        if (separatorIndex != -1) {
            String key = dataString.substring(0, separatorIndex);
            String value = dataString.substring(separatorIndex + 1);
            processDataCallback(key, value);
        }
    }
}

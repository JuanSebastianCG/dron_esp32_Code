#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udpServer;
const int8_t port = 80; 

void (*processDataCallback)(uint8_t key, int16_t value);

void wifiConnection(const char* ssid, const char* password, void (*callback)(uint8_t key, int16_t value)) {
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
    Serial.print(WiFi.localIP());
    Serial.print(" port: ");
    Serial.println(port);
    
    // Configura el servidor UDP en el puerto 12345 (o el que desees)
    udpServer.begin(port);
}


void processUDPData() {
    int packetSize = udpServer.parsePacket();
    if (packetSize >= sizeof(uint8_t) + sizeof(int16_t)) {
       
        uint8_t key;
        int16_t value;
        
        uint8_t buffer[sizeof(uint8_t) + sizeof(int16_t)];
        udpServer.read(buffer, sizeof(buffer));
        // El primer byte es uint8
        key = buffer[0];  
        value = (int16_t)((buffer[1] << 8) | buffer[2]);  // Los siguientes 2 bytes son int16
        
        processDataCallback(key, value);
    }
}


/* void processUDPData() {
    int packetSize = udpServer.parsePacket();
    if (packetSize >= sizeof(uint8_t) + sizeof(int16_t)) {
        
        uint8_t buffer[sizeof(uint8_t) + sizeof(int16_t)];
        udpServer.read(buffer, sizeof(buffer));
        
        processDataCallback(buffer[0], *((buffer + sizeof(int16_t))));
    }
}
 */

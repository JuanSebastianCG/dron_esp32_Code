#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "main.h"

WiFiUDP udpServer;

void (*processDataCallback)(uint8_t key, int16_t value1, int16_t value2);

void wifiConnection(const char *ssid, const char *password, void (*callback)(uint8_t key, int16_t value1, int16_t value2))
{
    processDataCallback = callback;
    WiFi.begin(ssid, password);

    // Espera hasta que se establezca la conexión Wi-Fi
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        if (++attempts > 10)
        {
            Serial.println("Error: No se pudo conectar a la red Wi-Fi");
            ESP.restart();
        }
    }

    // Configura el servidor UDP en el puerto 12345
    udpServer.begin(port);

    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.print(WiFi.localIP());
    Serial.print(" port: ");
    Serial.println(port);
}

void processUDPData()
{
    int packetSize = udpServer.parsePacket();
    if (packetSize >= sizeof(uint8_t) + 2 * sizeof(int16_t))
    {
        uint8_t key;
        int16_t value1, value2;

        uint8_t buffer[sizeof(uint8_t) + 2 * sizeof(int16_t)];
        udpServer.read(buffer, sizeof(buffer));

        key = buffer[0];
        value1 = (int16_t)((buffer[1] << 8) | buffer[2]); // Los siguientes 2 bytes son int16
        value2 = (int16_t)((buffer[3] << 8) | buffer[4]); // Los siguientes 2 bytes son int16

        processDataCallback(key, value1, value2);
    }
}

#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

extern WiFiUDP udpServer;


void wifiConnection(const char* ssid, const char* password, void (*callback)(uint8_t key, int16_t value1, int16_t value2));
void processUDPData();

#endif

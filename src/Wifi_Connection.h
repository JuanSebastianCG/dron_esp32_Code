#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

extern WiFiUDP udpServer;

void wifiConnection(const char* ssid, const char* password, void (*callback)(String key, String value));
void processUDPData();

#endif

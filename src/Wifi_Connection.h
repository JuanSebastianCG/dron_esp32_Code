#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

extern AsyncWebServer server;
extern WiFiClient client;

void wifiConection(const char* ssid, const char* password, void (*callback)(String key, String value));

#endif

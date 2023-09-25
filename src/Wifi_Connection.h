// Wifi_Connection.h
#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

#include <Arduino.h>
#include <WiFi.h>

// Declaración de la variable global server como externa
extern WiFiServer server;

// Declara la función para inicializar la conexión a Internet con parámetros
void wifiConection(const char* ssid, const char* password);

#endif

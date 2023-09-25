#include <Arduino.h>
#include "Wifi_Connection.h"

const char* ssid     = "ANPARO..";
const char* password = "24280650";

// Declarar las funciones
void processData(char data);

WiFiClient client;

void setup() {
    // Resto de tu código de configuración
    Serial.begin(9600);
    
    // Inicializa la conexión a Internet con los parámetros ssid y password
    wifiConection(ssid, password);
}

void loop() {
    // Maneja las solicitudes del servidor web aquí
    if (!client.connected()) {
        client = server.available();
    }

    // Lee y procesa los datos recibidos del cliente
    while (client.connected() && client.available()) {
        char c = client.read();
        processData(c);
        //json=data_Input
        //["key","value"]
         
        
    }

    // Enviar una respuesta HTTP 200 OK válida al cliente
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    // Cierra la conexión después de enviar la respuesta
    client.stop();
}


void processData(char data) {

    Serial.write(data)


}

#include <startup.h>

#include <Arduino.h>
#include <Wifi.h>

#include <eeprom_constants.h>
#include <wireless_constants.h>
#include <globals.h>

WiFiClient client;
uint16_t id;

/**
 * Reads the ID from the EEPROM.
*/
void readId() {
    EEPROM.begin(EEPROM_SIZE);
    uint16_t read_id = EEPROM.readUShort(ID_ADDRESS);

    // Prints warning for invalid ID.
    if (read_id == 0xFFFF) {
        Serial.println("Invalid ID 0xFFFF. Halting.");
        Serial.flush();
        while (1);
    }

    id = read_id;
}

/**
 * Wait for WiFi connection, and, if not connected, reboot
 */
void waitForWiFiConnectOrReboot() {
    uint32_t notConnectedCounter = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.println("Wifi connecting...");
        notConnectedCounter++;
        if(notConnectedCounter > 150) { // Reset board if not connected after 15s
            Serial.printf("Unable to connect to WiFi network: \"%s\".\n", ssid);
            ESP.restart();
        }
    }

    Serial.printf("Connected to WiFi network \"%s\". IP address: %s\n", ssid, WiFi.localIP().toString());
}

/**
 * Connects to WIFI and TCP.
*/
void connectToWifi() {
    WiFi.begin(ssid, password);
    waitForWiFiConnectOrReboot();

	client.connect(host_name, tcp_port);
    uint32_t us = micros();
    while (!client.connected()) {
        if (micros() - us > 10000000) { // 10 Seconds
            Serial.printf("Unabled to connect to %s:%d, restarting.\n", host_name, tcp_port);
            delay(1000);
            ESP.restart();
        }
    }
}

void startup() {
    connectToWifi();
    Serial.printf("Client has been connected to %s on port %d\n", host_name, tcp_port);

    readId();
    Serial.printf("Read ID: %04x.\n", id);
}

#include <Arduino.h>
// #include <WiFi.h>
// #include <AsyncTCP.h>

#include <globals.h>

#include <startup.h>
#include <standard_operation.h>
#include <reset_id.h>
#include <sensor.h>

#define RESET_PAIR 12


LIS3DHH sensor;

void setup() {
    // Initialize Serial
	Serial.begin(115200);
    while (!Serial);

    // Start the ESP by connecting to WIFI andd TCP
    startup();

    // Check reset pins
    pinMode(RESET_PAIR, INPUT_PULLDOWN);
    pinMode(RESET_ID, INPUT_PULLDOWN);
    if (digitalRead(RESET_PAIR)) {
        // TODO: New pair
    }
    if (digitalRead(RESET_ID)) {
        resetId(); // calls ESP.restart();
    }

    // Connect fully to the server
    connect();
    sensor.initalize();

    // Sync to the other ESP
    synchronize();
    // Collect the data
    collectData(sensor);

    // Sleep the ESP.
    // TODO: Move sleep logic into a better sleep system that properly deep sleeps the ESP
    uint8_t data[5] = {0};
    while (client.available() < 5);
    client.readBytes(reinterpret_cast<uint8_t*>(&data), 5);

    if (data[0] != 4) {
        Serial.printf("Invalid packet id for sleep time: %d", data[0]);
        return;
    }

    uint32_t sleep_time = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
    Serial.printf("Found sleep time of %d seconds.\n", sleep_time);
    // client.close(); How to close the client?

    delay(1000 * sleep_time);
    ESP.restart();
}


// No looping functionality
void loop() {}

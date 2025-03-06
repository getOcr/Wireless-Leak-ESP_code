#pragma once

#include <WiFi.h>
#include <SPI.h>
/**
 * The wifi client (TCP) used to communicate with the server.
*/
extern WiFiClient client;

#include <stdint.h>
/**
 * The ID for the device.
*/
extern uint16_t id;

#define ESP_Header 1
#define server_Header 2
#define sensorData_Header 3

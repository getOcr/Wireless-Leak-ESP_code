#pragma once

#include <WiFi.h>
/**
 * The wifi client (TCP) used to communicate with the server.
*/
extern WiFiClient client;

#include <stdint.h>
/**
 * The ID for the device.
*/
extern uint16_t id;
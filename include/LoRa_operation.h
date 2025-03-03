#pragma once

#include <lmic.h>
#include <hal/hal.h>
#include <Arduino.h>

extern const lmic_pinmap lmic_pins;  // lmic_pinmap is defined in <hal/hal.h> other than <lmic.h>

void LoRa_init();

void LoRa_sendData(const uint8_t* data, size_t size);

void LoRa_connectCheck();
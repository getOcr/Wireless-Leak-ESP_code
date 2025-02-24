#pragma once

#include <sensor.h>

/**
 * Connects to the server by sending the ID.
*/
void connect();

/**
 * Synchronizes to the other ESP.
*/
void synchronize();

/**
 * Collects data from the attached sensor.
*/
int16_t* collectData(BaseSensor& sensor);

void goToSleep(uint32_t sleep_time);
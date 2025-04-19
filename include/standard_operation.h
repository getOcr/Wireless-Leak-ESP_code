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
int16_t* collectData_og(BaseSensor& sensor); // collect for 1s but no specific frequency
int16_t* collectDataOnce(BaseSensor& sensor); // for test
int16_t* collectData(BaseSensor& sensor, int& outGroupCount); // normal one

void SensorCollectTask(BaseSensor& sensor);

void goToSleep(uint32_t sleep_time);
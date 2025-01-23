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
void collectData(BaseSensor& sensor);
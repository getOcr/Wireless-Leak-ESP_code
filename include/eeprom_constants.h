#pragma once

#include <EEPROM.h>

/**
 * The total size of the data stored in the EEPROM:
 * 0x00 - 0x02: ID 
*/
#define EEPROM_SIZE 2

/**
 * The address the ID (uint16_t) is stored
*/
#define ID_ADDRESS 0


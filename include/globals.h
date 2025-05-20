#pragma once

#include <WiFi.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHHardwareSPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include <stdint.h>

/**
 * The wifi client (TCP) used to communicate with the server.
*/
extern WiFiClient client;


/**
 * The ID for the device.
*/
extern uint16_t id;
#define ESP_Header 1
#define server_Header 2
#define sensorData_Header 3

/**
 * LoRa
*/
extern RH_RF95 rf95;
extern RHHardwareSPI mySPI;

/**
 * Sensor
*/
extern esp_timer_handle_t sampling_timer;
//extern LIS3DH sensor; // just claimed in task_schedule instead of in global

/**
 * Tasks schedule
*/
extern QueueHandle_t sensorDataQueue;
extern TaskHandle_t LoRaSendTaskHandle;

/**
 * LED indicator
*/
extern volatile bool isSending;
extern float batteryVoltage;
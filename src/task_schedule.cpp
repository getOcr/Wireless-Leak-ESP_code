/*#include <Arduino.h>
#include <LoRa_operation.h>
#include <sensor.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define QUEUE_SIZE 10  // 
QueueHandle_t sensorDataQueue;

LIS3DHH sensor;

//sensor collection task（high prioritized）
void SensorTask(void *pvParameters) {
    int16_t sensorData[3];
    while (1) {
        sensor.read(sensorData);  
        xQueueSend(sensorDataQueue, sensorData, portMAX_DELAY);  // store in queue
        vTaskDelay(pdMS_TO_TICKS(100));  // sampling interval: 100ms
    }
}

// LoRa transmission（less prioritized）
void LoRaTask(void *pvParameters) {
    int16_t receivedData[3];
    while (1) {
        if (xQueueReceive(sensorDataQueue, receivedData, portMAX_DELAY)) {
            uint8_t LoRaPacket[6] = {
                receivedData[0] >> 8, receivedData[0] & 0xFF,
                receivedData[1] >> 8, receivedData[1] & 0xFF,
                receivedData[2] >> 8, receivedData[2] & 0xFF
            };
            LoRa_sendData(LoRaPacket, sizeof(LoRaPacket));  //
        }
    }
}*/
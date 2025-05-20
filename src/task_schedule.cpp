#include "task_schedule.h"
#include "globals.h"
#include "sensor.h"
#include <Arduino.h>
#include <string.h>

extern LIS3DH sensor;

QueueHandle_t sensorDataQueue = NULL;
TaskHandle_t LoRaSendTaskHandle = NULL;
esp_timer_handle_t sampling_timer;
volatile bool isSending = false;
float batteryVoltage = 0.0;

#define LED_PIN 2
#define BAT_ADC_PIN 34

// CRC16
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

void IRAM_ATTR onSampleTimer(void* arg) {
    LIS3DH* sensor = (LIS3DH*) arg;
    int16_t group[4];
    static uint16_t groupIndex = 0;
    group[0] = groupIndex++;
    sensor->read(&group[1]);
    xQueueSendFromISR(sensorDataQueue, &group, NULL); // ISR: Interrupt Service Routine
        // can not use vQueueSend() which is a probably blocking function,
        // any probably blocking function is not allowed in ISR, such as vTaskDelay(), may cause dead lock.
        // The 3rd argu NULL means no "woken" is supported
}

void LoRaSendTask(void *pvParameters) {
    uint8_t packet[255];
    uint8_t packetId = 0;
    int16_t group[4];
    const int GROUP_BYTE_SIZE = 8;
    const int GROUPS_PER_PACKET = 30;
    const int MAX_RETRIES = 3; // max retry times
    const int RETRY_DELAY_MS = 100; // wait ACK for 100ms

    while (true) {
        int groupCount = 0;
        memset(packet, 0, sizeof(packet));
        packetId++;
        packet[0] = packetId;

        while (groupCount < GROUPS_PER_PACKET) {
            if (xQueueReceive(sensorDataQueue, &group, pdMS_TO_TICKS(100))) {
                int offset = groupCount * GROUP_BYTE_SIZE;
                for (int k = 0; k < 4; k++) {
                    packet[1 + offset + 2*k] = (group[k] >> 8) & 0xFF;
                    packet[1 + offset + 2*k + 1] = group[k] & 0xFF;
                }
                groupCount++;
            } else {
                break;
            }
        }

        if (groupCount > 0) {
            // calculate CRC
            uint16_t crc = calculateCRC16(packet, 1 + groupCount * GROUP_BYTE_SIZE);
            packet[1 + groupCount * GROUP_BYTE_SIZE] = (crc >> 8) & 0xFF;
            packet[1 + groupCount * GROUP_BYTE_SIZE + 1] = crc & 0xFF;

            // send data and wait for ACK
            bool ackReceived = false;
            int retryCount = 0;
            
            while (!ackReceived && retryCount < MAX_RETRIES) {
                isSending = true;
                rf95.send(packet, 1 + groupCount * GROUP_BYTE_SIZE + 2); // +2 for CRC
                rf95.waitPacketSent();
                isSending = false;

                // wait ACK
                uint32_t startTime = millis();
                while (millis() - startTime < RETRY_DELAY_MS) {
                    if (rf95.available()) {
                        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
                        uint8_t len = sizeof(buf);
                        if (rf95.recv(buf, &len)) {
                            if (buf[0] == packetId) { 
                                ackReceived = true;
                                int16_t rssi = rf95.lastRssi();
                                Serial.printf("[LoRa] Packet #%d sent successfully, RSSI = %d dBm\n", packetId, rssi);
                                break;
                            }
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                
                if (!ackReceived) {
                    retryCount++;
                    Serial.printf("[LoRa] Retry %d/%d for packet #%d\n", retryCount, MAX_RETRIES, packetId);
                }
            }

            if (!ackReceived) {
                Serial.printf("[LoRa] Failed to send packet #%d after %d retries\n", packetId, MAX_RETRIES);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void StatusLedTask(void *pvParameters) {
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, isSending ? HIGH : LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

float readBatteryVoltage() {
    int raw = analogRead(BAT_ADC_PIN);
    return (raw / 4095.0f) * 3.3f * 2.0f;
}

void BatteryMonitorTask(void *pvParameters) {
    analogReadResolution(12);
    while (1) {
        batteryVoltage = readBatteryVoltage();
        Serial.printf("[Battery] Voltage: %.2f V\n", batteryVoltage);
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 second, 5000 * tick frequency
    }
}

void startTasks() {
    sensorDataQueue = xQueueCreate(1000, sizeof(int16_t) * 4);
    if (sensorDataQueue == NULL) {
        Serial.println("[Error] Failed to create sensorDataQueue!");
    }

    xTaskCreatePinnedToCore(LoRaSendTask, "LoRaSendTask", 4096, NULL, 3, &LoRaSendTaskHandle, 1); // 4096*32 bytes is the stack size, word size is 32 in esp32 
    xTaskCreatePinnedToCore(StatusLedTask, "StatusLedTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(BatteryMonitorTask, "BatteryMonitorTask", 2048, NULL, 1, NULL, 1);
    
    // use esp harware timer for Sensor data collection
    esp_timer_create_args_t timer_args = {
        .callback = &onSampleTimer,
        .arg = &sensor,
        .name = "sampling_timer"
    };
    if (esp_timer_create(&timer_args, &sampling_timer) != ESP_OK) {
        Serial.println("[Error] Failed to create esp_timer!");
    }
    if (esp_timer_start_periodic(sampling_timer, 200) != ESP_OK) { // register an ISR 
        Serial.println("[Error] Failed to start esp_timer!");
    // 1 group collection per 200 micro second, approximately 20%~30% CPU occupation
    }
}
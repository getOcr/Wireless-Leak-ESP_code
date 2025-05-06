#include "task_schedule.h"
#include "globals.h"
#include "sensor.h"
#include <Arduino.h>
#include <string.h>

extern LIS3DH sensor;

QueueHandle_t sensorDataQueue = NULL;
TaskHandle_t LoRaSendTaskHandle = NULL;
TaskHandle_t LoRaReceiveTaskHandle = NULL;
esp_timer_handle_t sampling_timer;
volatile bool isSending = false;
float batteryVoltage = 0.0;

#define LED_PIN 2
#define BAT_ADC_PIN 34

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

void LoRaSendTask(void *pvParameters) { // pv means pointer to void
    uint8_t packet[255];
    uint8_t packetId = 0;
    int16_t group[4];
    const int GROUP_BYTE_SIZE = 8;
    const int GROUPS_PER_PACKET = 30;

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
            isSending = true;
            rf95.send(packet, 1 + groupCount * GROUP_BYTE_SIZE);
            rf95.waitPacketSent();
            isSending = false;
            Serial.printf("[LoRa] Sent %d groups\n", groupCount);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // suspend sending task and give the CPU runtime to other lower-priority task
    }
}

void LoRaReceiveTask(void *pvParameters) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    while (true) {
        if (rf95.available()) {
            if (rf95.recv(buf, &len)) {
                Serial.print("[LoRa RX] Got reply: ");
                Serial.println((char*)buf);
                Serial.print("[LoRa RX] RSSI: ");
                Serial.println(rf95.lastRssi(), DEC);
            } else {
                Serial.println("[LoRa RX] recv failed");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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

    xTaskCreatePinnedToCore(LoRaSendTask, "LoRaSendTask", 4096, NULL, 3, &LoRaSendTaskHandle, 1);
    xTaskCreatePinnedToCore(LoRaReceiveTask, "LoRaReceiveTask", 2048, NULL, 2, &LoRaReceiveTaskHandle, 1);
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
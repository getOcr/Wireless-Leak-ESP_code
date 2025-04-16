#include <Arduino.h>
#include <globals.h>
#include <startup.h>
#include <standard_operation.h>
#include <reset_id.h>
#include <sensor.h>
#include "esp_sleep.h"
#include <LoRa_operation.h>
#include <BLE_operation.h>

// LIS3DHH sensor;
// //extern bool LoRa_joined;

// #define QUEUE_SIZE 128
// int16_t sensorDataQueue[QUEUE_SIZE][4];
// int queueHead = 0;  // send
// int queueTail = 0;  // collect
// static uint16_t packetCounter = 1;

// // time control
// unsigned long lastSampleTime = 0;
// unsigned long lastSendTime = 0;
// const unsigned long sampleInterval = 10000;     // collect every 10s
// const unsigned long sendInterval = 60000;    // send every 120s

// extern "C" void printSerial(const char* msg) {
//     Serial.println(msg);
// }

void setup() {
    // Initialize Serial
	Serial.begin(115200);
    while (!Serial);

    // BLE
    BLE_init();
    delay(2000);  // wait for connection
    BLE_sendDummyData();

    // // disable WiFi in case NVS
    // WiFi.disconnect(true);  
    // delay(100);
    // WiFi.mode(WIFI_OFF); 
    // Serial.println("WiFi disabled!");

    // SPI.begin(5, 19, 18);
    // //SPI.begin(18, 19, 23, 5); // SCK, MISO, MOSI, CS (NSS)

    // // sensor initialize
    // sensor.initialize();
    // Serial.println("sensor initialized");

    // // LoRa initialize
    // LoRa_init();
    // Serial.println("loRa initialized");

    /*// Collect the data
    int16_t* LoRadata = collectData(sensor);
    Serial.println("data collected");
    uint8_t data[6] = {
        LoRadata[0] >> 8, LoRadata[0] & 0xFF,
        LoRadata[1] >> 8, LoRadata[1] & 0xFF,
        LoRadata[2] >> 8, LoRadata[2] & 0xFF
    };
    Serial.print("Data bytes: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X ", data[i]);  // Print as two-digit hexadecimal
    }
    Serial.println();*/  

    // LoRa connect check
    //LoRa_connectCheck();
    
    // create queue
    //sensorDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(int16_t) * 3);

    // create tasks
    //xTaskCreatePinnedToCore(sensor.SensorTask, "SensorTask", 4096, NULL, 2, NULL, 1);
    //xTaskCreatePinnedToCore(LoRaTask, "LoRaTask", 4096, NULL, 1, NULL, 1);
    //xTaskCreatePinnedToCore(SleepTask, "SleepTask", 2048, NULL, 0, NULL, 1);

    // Test
    //if(LoRa_joined){
    //LoRa_sendData(data, sizeof(data));
    //} 
}

void loop() {
    // os_runloop_once();

    // //uint8_t test_data[4] = { 0x12, 0x34, 0x56, 0x78 };
    // //LoRa_sendData(test_data, sizeof(test_data));

    // //Serial.println("Test data sent. Sleeping for 30 seconds...");
    
    // //LoRa_sendData(test_data, sizeof(test_data));

    // //goToSleep(30); // deep sleep for 30s

    // unsigned long now = millis();

    // // sensor collecting
    // if (now - lastSampleTime >= sampleInterval) {
    //     lastSampleTime = now;

    //     int nextTail = (queueTail + 1) % QUEUE_SIZE;
    //     if (nextTail != queueHead) {  // queue is not full
    //         sensor.read(&sensorDataQueue[queueTail][1]);
    //         sensorDataQueue[queueTail][0] = packetCounter; 
    //         // debug
    //         Serial.printf("Sampled: #%d X=%d Y=%d Z=%d\n",
    //             sensorDataQueue[queueTail][0],
    //             sensorDataQueue[queueTail][1],
    //             sensorDataQueue[queueTail][2],
    //             sensorDataQueue[queueTail][3]);
    //         packetCounter++;
    //         queueTail = nextTail;
    //     } else {
    //         Serial.println("Sensor queue full. Skipping sample.");
    //     }
    // }

    // // LoRa sending
    // if (now - lastSendTime >= sendInterval) {
    //     lastSendTime = now;

    //     if (queueHead != queueTail && !(LMIC.opmode & OP_TXRXPEND)) {

    //         uint8_t data[8] = {
    //             (uint8_t)(sensorDataQueue[queueHead][0] >> 8), (uint8_t)(sensorDataQueue[queueHead][0] & 0xFF),
    //             sensorDataQueue[queueHead][1] >> 8, sensorDataQueue[queueHead][1] & 0xFF,
    //             sensorDataQueue[queueHead][2] >> 8, sensorDataQueue[queueHead][2] & 0xFF,
    //             sensorDataQueue[queueHead][3] >> 8, sensorDataQueue[queueHead][3] & 0xFF,
    //         };

    //         LoRa_sendData(data, sizeof(data));

    //         Serial.printf("LoRa queued: #%d X=%d Y=%d Z=%d\n",
    //             sensorDataQueue[queueHead][0],
    //             sensorDataQueue[queueHead][1],
    //             sensorDataQueue[queueHead][2],
    //             sensorDataQueue[queueHead][3]);
    //         Serial.println();

    //         queueHead = (queueHead + 1) % QUEUE_SIZE;
    //     } else {
    //         Serial.println("LoRa busy or no data to send.");
    //     }
    // }
}

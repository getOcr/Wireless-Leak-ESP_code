#include <Arduino.h>
#include <globals.h>
#include <startup.h>
#include <standard_operation.h>
#include <reset_id.h>
#include <sensor.h>
#include "esp_sleep.h"
#include <LoRa_operation.h>

LIS3DHH sensor;
extern bool LoRa_joined;

extern "C" void printSerial(const char* msg) {
    Serial.println(msg);
}

void setup() {
    // Initialize Serial
	Serial.begin(115200);
    while (!Serial);

    // disable WiFi in case NVS
    WiFi.disconnect(true);  
    delay(100);
    WiFi.mode(WIFI_OFF); 
    Serial.println("WiFi disabled!");

    // sensor initialize
    //sensor.initialize();

    /*// Sync to the other ESP
    synchronize();*/

    //SPI.begin(18, 19, 23, 5); // SCK, MISO, MOSI, CS (NSS)
    //Serial.println("SPI fffffkkkkk");
    
    // LoRa
    //LoRa_joined = LoRa_init();
    LoRa_init();

    // LoRa connect check
    //LoRa_connectCheck();
    
    // create queue
    //sensorDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(int16_t) * 3);

    // create tasks
    //xTaskCreatePinnedToCore(sensor.SensorTask, "SensorTask", 4096, NULL, 2, NULL, 1);
    //xTaskCreatePinnedToCore(LoRaTask, "LoRaTask", 4096, NULL, 1, NULL, 1);
    //xTaskCreatePinnedToCore(SleepTask, "SleepTask", 2048, NULL, 0, NULL, 1);
    
    // Test
    uint8_t test_data[4] = { 0x12, 0x34, 0x56, 0x78 };
    if(LoRa_joined){
        
        LoRa_sendData(test_data, sizeof(test_data));
        Serial.println("Connected and Test data sent.");
    } 
}

void loop() {
    os_runloop_once();
    // Collect the data
    //int16_t* LoRadata = collectData(sensor);

    //Serial.println("Sending sensor data via LoRa...");

    // Send the data via LoRa
    /*uint8_t data[6] = { LoRadata[0] >> 8, LoRadata[0] & 0xFF,
                         LoRadata[1] >> 8, LoRadata[1] & 0xFF,
                         LoRadata[2] >> 8, LoRadata[2] & 0xFF };
    LoRa_sendData(data, sizeof(data));*/

    //uint8_t test_data[4] = { 0x12, 0x34, 0x56, 0x78 };
    //LoRa_sendData(test_data, sizeof(test_data));

    //Serial.println("Test data sent. Sleeping for 30 seconds...");
    
    //LoRa_sendData(test_data, sizeof(test_data));

    //goToSleep(30); // deep sleep for 30s
}

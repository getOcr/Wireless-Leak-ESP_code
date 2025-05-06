#include <RH_RF95.h>
#include <Arduino.h>
#include <globals.h>
#include <standard_operation.h>
#include <sensor.h>
#include "esp_sleep.h"
#include <LoRaWAN_operation.h>
#include "task_schedule.h"
//#include <BLE_operation.h>

// for feather esp32 with RFM95
#define RFM95_CS 33
#define RFM95_INT 13
#define RFM95_RST 32 

// Singleton instance of the radio driver
//RH_RF95 rf95;
RHHardwareSPI mySPI;
RH_RF95 rf95(RFM95_CS, RFM95_INT, mySPI); 

LIS3DH sensor;

void setup() 
{
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to be available
  SPI.begin(5, 19, 18);  // SCK, MISO, MOSI, for both sensor and LoRa module

  pinMode(21, OUTPUT); digitalWrite(21, HIGH); // sensor CS
  pinMode(33, OUTPUT); digitalWrite(33, HIGH); // LoRa CS
  pinMode(32, OUTPUT); digitalWrite(32, HIGH); // LoRa RST

  // disable WiFi in case NVS
  WiFi.disconnect(true);  
  delay(100);
  WiFi.mode(WIFI_OFF); 
  Serial.println("WiFi disabled!");

  // sensor initialize
  sensor.initialize();
  Serial.println("sensor initialized");

  // LoRa p2p initialize
  if (!rf95.init()){
    Serial.println("rf95.init() failed");
  }else{
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // You can change the modulation parameters with eg
  //rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  // We configured that in RH_RF95.cpp
  //rf95.spiWrite(0x26, 0x08);
  //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 2 to 20 dBm:
  //rf95.setTxPower(20, false);
    Serial.println("rf95.init() success");
  }

  uint8_t reg26 = rf95.spiRead(0x26);
  Serial.printf("RegModemConfig3 (0x26) = 0x%02X\n", reg26);

  startTasks();

}

void loop()
{
  // replaced by RTOS
}


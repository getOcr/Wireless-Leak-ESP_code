// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <RH_RF95.h>
#include <Arduino.h>
#include <globals.h>
#include <standard_operation.h>
#include <sensor.h>
#include "esp_sleep.h"
#include <LoRaWAN_operation.h>
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
  while (!Serial) ; // Wait for serial port to be available
  SPI.begin(5, 19, 18);  // SCK, MISO, MOSI, for both sensor and LoRa module

  // disable WiFi in case NVS
  WiFi.disconnect(true);  
  delay(100);
  WiFi.mode(WIFI_OFF); 
  Serial.println("WiFi disabled!");

  // sensor initialize
  sensor.initialize();
  Serial.println("sensor initialized");

  // LoRa p2p
  if (!rf95.init()){
    Serial.println("init() failed");
  }else{
    Serial.println("init() success");
  }
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  rf95.setFrequency(915.0);
  // You can change the modulation parameters with eg
  //rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf2048);
  //rf95.spiWrite(0x26, 0x08);
  //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 2 to 20 dBm:
  //rf95.setTxPower(20, false);

  // Collect the data
  int groupCount = 0;
  int16_t* sampleData = collectData(sensor, groupCount);
    /*int totalAxes = 3;
    int totalSamples = 10;

    for (int i = 0; i < totalSamples; i++) {
        int x = data[i * totalAxes + 0];
        int y = data[i * totalAxes + 1];
        int z = data[i * totalAxes + 2];

        Serial.printf("Sample %d: X=%d Y=%d Z=%d\n", i, x, y, z);
    }*/

  Serial.println("Start sending to rf95_server");
  // Send a message to rf95_server
  const int GROUP_SIZE = 4;               // [ID, X, Y, Z]
  const int GROUP_BYTE_SIZE = 8;          // 4 × int16_t = 8
  const int GROUPS_PER_PACKET = 30;       // 30 sets per sending
  const int PACKET_SIZE = 1 + GROUPS_PER_PACKET * GROUP_BYTE_SIZE;

  uint8_t packet[PACKET_SIZE];
  uint8_t packetId = 0;

  unsigned long start = millis();
  for (int i = 0; i < groupCount; i += GROUPS_PER_PACKET) {
    memset(packet, 0, PACKET_SIZE); 
    packetId++;
    packet[0] = packetId;  // package ID
    int groupThisPacket = min(GROUPS_PER_PACKET, groupCount - i); // consider the last set.

    for (int j = 0; j < groupThisPacket; j++) {
        int16_t* groupPtr = &sampleData[(i + j) * GROUP_SIZE];
        int offset = j * GROUP_BYTE_SIZE;

        for (int k = 0; k < GROUP_SIZE; k++) {
            packet[1 + offset + 2 * k]     = (groupPtr[k] >> 8) & 0xFF;
            packet[1 + offset + 2 * k + 1] = groupPtr[k] & 0xFF;
        }
    }

    rf95.send(packet, 1 + groupThisPacket * GROUP_BYTE_SIZE);
    rf95.waitPacketSent();
    Serial.printf("Sent %d groups (%d bytes)\n", groupThisPacket, groupThisPacket * GROUP_BYTE_SIZE);

    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.waitAvailableTimeout(3000))
    { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
      {
        Serial.print("got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else
      {
        Serial.println("recv failed");
      }
    }
    else
    {
      Serial.println("No reply, is rf95_server running?");
    }
    delay(400);
  }
  unsigned long end = millis();
  Serial.printf("%d sets of data sent, time consumption: %.2f s\n", groupCount, (end - start) / 1000.0);
}

void loop()
{
//   Serial.println("Sending to rf95_server");
//   // Send a message to rf95_server
//   uint8_t data[] = "Hello World!";
//   rf95.send(data, sizeof(data));
  
//   rf95.waitPacketSent();
//   // Now wait for a reply
//   uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//   uint8_t len = sizeof(buf);

//   if (rf95.waitAvailableTimeout(3000))
//   { 
//     // Should be a reply message for us now   
//     if (rf95.recv(buf, &len))
//    {
//       Serial.print("got reply: ");
//       Serial.println((char*)buf);
// //      Serial.print("RSSI: ");
// //      Serial.println(rf95.lastRssi(), DEC);    
//     }
//     else
//     {
//       Serial.println("recv failed");
//     }
//   }
//   else
//   {
//     Serial.println("No reply, is rf95_server running?");
//   }
//   delay(400);
}


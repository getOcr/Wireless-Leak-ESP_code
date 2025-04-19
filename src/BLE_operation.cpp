#include "BLE_operation.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-90ab-abcdefabcdef"

BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Client disconnected");
  }
};

static MyServerCallbacks* serverCallbacks = nullptr;

void BLE_init() {
  BLEDevice::init("ESP32_BLE_Sensor");
  BLEDevice::setMTU(247);  // 支持更大 notify 长度

  BLEServer* pServer = BLEDevice::createServer();
  serverCallbacks = new MyServerCallbacks();
  pServer->setCallbacks(serverCallbacks);

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE service started and advertising");
}

void BLE_sendDummyData() {
  const int totalSamples = 25000;
  const int axes = 3;
  int16_t dummyData[totalSamples * axes];

  for (int i = 0; i < totalSamples * axes; i++) {
    dummyData[i] = i % 1000;
  }

  const int chunkSize = 122; // 244 bytes
  uint8_t buffer[chunkSize * 2];

  while (!deviceConnected) {
    Serial.println("Waiting for BLE client...");
    delay(1000);
  }

  for (int i = 0; i < totalSamples * axes; i += chunkSize) {
    if (!deviceConnected) {
      Serial.println("BLE disconnected during transfer, aborting.");
      break;
    }

    int remaining = totalSamples * axes - i;
    int sendCount = remaining >= chunkSize ? chunkSize : remaining;

    for (int j = 0; j < sendCount; j++) {
      buffer[2 * j]     = dummyData[i + j] >> 8;
      buffer[2 * j + 1] = dummyData[i + j] & 0xFF;
    }

    pCharacteristic->setValue(buffer, sendCount * 2);
    pCharacteristic->notify();
    delay(30); // 稍微大一点防止 buffer 堆积崩溃
  }

  Serial.println("All dummy data sent!");
}
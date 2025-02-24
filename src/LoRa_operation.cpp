#include <LoRa_operation.h>

//We use Over-The-Air-Activation(OTAA), it can be concluded as followed:
//Step1. 
//  End device sends join-request message which consists of following:
//      | 8 bytes | 8 bytes | 2 bytes   |
//      | AppEUI  | DevEUI  | DevNonce |
//Step2.
//  Network Server generates 2 session keys (NwkSKey kept by Network server and AppSKey kept by application server) 
//  and sends join-accept message which consists of following:
//      | 3 Bytes  | 3 Bytes | 4 Bytes | 1 Bytes    |  1 Bytes  | 16 Bytes (optional) |
//      | AppNonce | NetID   | DevAddr | DLSettings |  RX Delay | CFList              |
//Step3.
//  End device uses AppKey and AppNonce to derive NwkSKey and AppSKey.

static bool LoRa_joined = false;

// LoRaWAN device info
// PROGMEM means data is stored in Flash instead of RAM
static const u1_t PROGMEM APPEUI[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x82, 0xAE };  // identifier of TTN or ChirpStack server
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x4F, 0xD1, 0xB3, 0x71, 0x34, 0x56, 0x78 };  // identifier of LoRaWAN end device
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x49, 0x21, 0x34, 0x56, 0x78, 0x90 };  // 

void os_getArtEui (u1_t* buf) { memcpy(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy(buf, APPKEY, 16); }

// LoRa frequency（915 MHz for US，868 MHz for EU）
const lmic_pinmap lmic_pins = {
    .nss = 5,      // 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,     // 
    .dio = {2, 21, 22} // DIO0, DIO1, DIO2
};

void onEvent(ev_t ev) {
    switch(ev) {
        case EV_JOINING:
            Serial.println("Joining LoRa network...");
            break;
        case EV_JOINED:
            Serial.println("LoRa joined successfully!");
            LoRa_joined = true;
            break;
        case EV_TXCOMPLETE:
            Serial.println("LoRa TX complete!");
            break;
        default:
            Serial.println("Unknown LoRa event");
            break;
    }
}

void LoRa_init() {
    Serial.println("Initializing LoRa...");
    os_init();
    LMIC_reset();

    // Set frequency
    LMIC_selectSubBand(1);  //  US915

    // start OTAA
    LMIC_startJoining();
}

void LoRa_sendData(const uint8_t* data, size_t size) {
    if (!LoRa_joined) {
        Serial.println("LoRa not joined, cannot send data.");
        return;
    }

    if (size > 51) {
        Serial.println("Error: Payload size exceeds LoRa limit (51 bytes)");
        return;
    }

    Serial.printf("Sending LoRa data: %d bytes\n", size);
    LMIC_setTxData2(1, (uint8_t*)data, size, 0);
}
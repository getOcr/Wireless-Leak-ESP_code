//We use Over-The-Air-Activation(OTAA), it can be concluded as followed:
//Step1. 
//  End device sends join-request message which consists of following:
//      | 8 bytes | 8 bytes | 2 bytes   |
//      | AppEUI  | DevEUI  | DevNonce  |
//Step2.
//  Network Server generates 2 session keys (NwkSKey kept by Network server and AppSKey kept by application server) 
//  and sends join-accept message which consists of following:
//      | 3 Bytes  | 3 Bytes | 4 Bytes | 1 Bytes    |  1 Bytes  | 16 Bytes (optional) |
//      | AppNonce | NetID   | DevAddr | DLSettings |  RX Delay | CFList              |
//Step3.
//  End device uses AppKey and AppNonce to derive NwkSKey and AppSKey.


#include <LoRa_operation.h>
#include <EEPROM.h>

static bool LoRa_joined = false;

// used to check if EUI is right before sending, in case of error fromm Flash to buf
void debugEUI(const char* label, const uint8_t* eui) {
    Serial.printf("%s: ", label);
    for (int i = 0; i < 8; i++) {
        Serial.printf("%02X ", eui[i]);
    }
    Serial.println();
}

// LoRaWAN device info
// PROGMEM means data is stored in Flash instead of RAM
static const u1_t PROGMEM APPEUI[8] = { 0xAE, 0x82, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };  // identifier of TTN or ChirpStack server
//static const u1_t PROGMEM DEVEUI[8] = { 0xC8, 0x41, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 }; // identifier of LoRaWAN end device
static const u1_t PROGMEM DEVEUI[8] = { 0x1E, 0x43, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 }; // for 104dev
static const u1_t PROGMEM APPKEY[16] = { 0x25, 0x0D, 0x30, 0x53, 0xAD, 0x8F, 0xE4, 0x6C, 0x66, 0x1E, 0x3A, 0x4C, 0xB0, 0xC7, 0x1C, 0x57 };  // 

void os_getArtEui (u1_t* buf) { memcpy(buf, APPEUI, 8); debugEUI("AppEUI", buf);}
void os_getDevEui (u1_t* buf) { memcpy(buf, DEVEUI, 8); debugEUI("DevEUI", buf);}
void os_getDevKey (u1_t* buf) { memcpy(buf, APPKEY, 16); }

//  LoRa frequency（915 MHz for US，868 MHz for EU）
const lmic_pinmap lmic_pins = {
    .nss = 33,      // chip select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 32,     // reset
    //.rst = LMIC_UNUSED_PIN,
    .dio = {13, 27, 12}, // DIO0, DIO1, DIO2
    //.dio = {2, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN}
    .rxtx_rx_active = 0,
    .rssi_cal = 0,
    .spi_freq = 0
};


//  ev_t is defined by LMIC to indicate the state of LoRa module
void onEvent(ev_t ev) { 
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            Serial.println("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            Serial.println("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            Serial.println("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            Serial.println("Joining LoRa network...");
            break;
        case EV_JOINED:
            Serial.println("LoRa joined successfully!");
            LoRa_joined = true; // set flag
            break;
        case EV_JOIN_FAILED:
            Serial.println("EV_JOIN_FAILED - Join failed");
            break;
        case EV_REJOIN_FAILED:
            Serial.println("EV_REJOIN_FAILED - Rejoin failed");
            break;
        case EV_TXCOMPLETE:
            Serial.println("EV_TXCOMPLETE - TX complete");
            if (LMIC.dataLen) {  // if got downlink data
                Serial.printf("Received downlink: RSSI %d, SNR %d\n", LMIC.rssi, LMIC.snr);
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println("EV_RESET");
            break;
        case EV_RXCOMPLETE:
            Serial.println("EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println("EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            Serial.println("EV_LINK_ALIVE");
            break;
        case EV_TXSTART:
            Serial.println("EV_TXSTART");
            break;
        case EV_TXCANCELED:
            Serial.println("EV_TXCANCELED");
            break;
        case EV_RXSTART:
            // print nothing in case interupt receiving
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println("EV_JOIN_TXCOMPLETE: Join request sent");
            break;
        default:
            Serial.print("Unknown LoRa event: ");
            Serial.println((unsigned) ev);
            break;
    }
}

bool LoRa_init() {
    Serial.println("Initializing LoRa...");
    os_init();
    Serial.println("os_init() finished");
    LMIC_reset();
    Serial.println("LMIC_reset() finished");

    //  Set frequency
    // According to "rp002-1-0-4-regional-parameters"
    // For LoRa uplink channel(from end device to LoRa gateway)：64+8=72 channels in total. 
    //  64 channels(125kHz BW) starting from 902.3MHz for relatively low data rate DR0 ~ DR3;
    //  8 channels starting from 903MHz for higher rate: DR4 with 500kHz BW || DR5~DR6 with LRFHSS 1.523MHz BW.
    // For LoRa downlink channel(from LoRa gateway to end device)：8 channels in total starting from 923.3MHz.
    // For LMIC_selectSubBand(): input is ranging in [0,8), which separates the 72 channels into 8 sub-bands.
    LMIC_selectSubBand(0);  //  US915:[0,8)

    // debug for devEui and end device MAC
    /* uint8_t devEui[8];
    EEPROM.get(0, devEui);

    Serial.print("Stored DevEUI: ");
    for (int i = 0; i < 8; i++) {
        Serial.printf("%02X", devEui[i]);  // 
        if (i < 7) {
            Serial.print(":");  // 
        }
    }
    Serial.println();  // */
    /*uint8_t mac[MAC_LENGTH];
    
    digitalWrite(NSS, LOW);
    SPI.transfer(REG_MAC_ADDRESS | 0x80);  // 读模式
    for (int i = 0; i < MAC_LENGTH; i++) {
        mac[i] = SPI.transfer(0x00);
    }
    digitalWrite(NSS, HIGH);

    Serial.print("Real MAC Address: ");
    for (int i = 0; i < MAC_LENGTH; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < MAC_LENGTH - 1) Serial.print(":");
    }
    Serial.println();
    */

    //  Set data rate
    //LMIC_setDrTxpow(US915_DR_SF8C, 14); // DR4: 222 bytes  DR4:SF8@500kHz 
    LMIC_setDrTxpow(US915_DR_SF10, 14); // DR0

    //  Start OTAA
    LMIC_startJoining();

    while (!LoRa_joined) {
        os_runloop_once();  
        Serial.println("Trying to join LoRa network...");
        delay(2000);  // avoid polling too fast
    }

    Serial.println("LoRa successfully joined!");
    return LoRa_joined;
}

void LoRa_sendData(const uint8_t* data, size_t size) {
    unsigned long startWaitTime = millis();
    
    while (!LoRa_joined && millis() - startWaitTime < 30000) { // wait up to 30s
        Serial.println("Waiting for LoRa to join network...");
        delay(1000);
        os_runloop_once();  // LMIC activity
    }
    
    if (!LoRa_joined) {
        Serial.println("LoRa join timeout. Cannot send data.");
        return;
    }

    if (size > 51) {
        Serial.println("Error: Payload size exceeds LoRa limit (51 bytes)");
        return;
    }

    Serial.printf("Sending LoRa data: %d bytes\n", size);
    LMIC_setTxData2(1, (uint8_t*)data, size, 1); //the last parameter: 0-->unconform 1-->conform
}

void LoRa_connectCheck(){
    
    Serial.println("Waiting for LoRaWAN join...");
    while (!LoRa_joined) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nLoRaWAN Joined!");
}
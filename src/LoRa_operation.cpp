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

osjob_t sendjob; //is a struct for task schedule in LMIC
static uint8_t queuedPayload[53] = {0};  //
static size_t payloadSize;
// static bool LoRa_joined = false;
extern bool LoRa_joined = false;
// For periodic sending--Schedule TX every this many seconds 
const unsigned TX_INTERVAL = 60; // (might become longer due to duty cycle limitations).

void do_send(osjob_t* j) {
    Serial.println(F("do_send() start"));
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));// Check if previous TX/RX job is still pending, OP_TXRXPEND is a flag indicating that
    } else {
        LMIC_setTxData2(1, queuedPayload, payloadSize, 0); //the last parameter: 0-->unconform 1-->conform
        //Serial.println(F("LoRa Packet queued"));
    }
    // time for next round sending is set in EV_TXCOMPLETE
}

// provide an API to change the static queuedPayload and payloadSize in this file
void preparePayload(const uint8_t* data, size_t size) {
    memcpy(queuedPayload, data, size);
    payloadSize = size;
}

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
static const u1_t PROGMEM DEVEUI[8] = { 0xC8, 0x41, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 }; // identifier of LoRaWAN end device
//static const u1_t PROGMEM DEVEUI[8] = { 0x1E, 0x43, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 }; // for 1.0.4dev which will introduce the problem of "DevNounce is too small"
static const u1_t PROGMEM APPKEY[16] = { 0x25, 0x0D, 0x30, 0x53, 0xAD, 0x8F, 0xE4, 0x6C, 0x66, 0x1E, 0x3A, 0x4C, 0xB0, 0xC7, 0x1C, 0x57 };  // 

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); debugEUI("AppEUI", buf);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); debugEUI("DevEUI", buf);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

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

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

//  ev_t is defined by LMIC to indicate the state of LoRa module
void onEvent(ev_t ev) { 
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	        // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            LoRa_joined = true;
            if(LoRa_joined) {Serial.println("LoRa_Joined = 1");}
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // schedule next round sending
            //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void LoRa_init() {
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
    LMIC_selectSubBand(1);  //  US915:[0,8)

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
    //LMIC_setDrTxpow(US915_DR_SF10, 14); // DR0
    LMIC_setDrTxpow(US915_DR_SF9, 14); // DR1
    //  Start OTAA
    //LMIC_startJoining(); // comment it because do_send automatically starts OTAA too

    /*while (!LoRa_joined) {
        os_runloop_once();  
        Serial.println("Trying to join LoRa network...");
        delay(2000);  // avoid polling too fast
    }*/

    //Serial.println("LoRa successfully joined!");
    //return LoRa_joined;
}


void LoRa_sendData(const uint8_t* data, size_t size) {
    if (size > 51) {
        Serial.println("Payload too large(>53 bytes)");
        return;
    }

    preparePayload(data, size);

    do_send(&sendjob);  // automatically start OTAA join
}


void LoRa_connectCheck(){
    
    Serial.println("Waiting for LoRaWAN join...");
    while (!LoRa_joined) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nLoRaWAN Joined!");
}
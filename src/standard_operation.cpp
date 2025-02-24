#include <standard_operation.h>

#include <globals.h>

/**
 * Sends the ID to the server given a packet id (useful for when pair reset is implemented)
*/
void sendId(uint8_t packetId) {
    uint8_t data[] = {
        packetId,
        (id >> 8) & 0xFF,
        (id) & 0xFF
    };

    client.write(data, sizeof(data));
}

/**
 * Receives information about the pair.
*/
void receivePair(IPAddress* address, uint16_t* port, uint16_t* id) {
    uint8_t pair[9] = {0}; //000 000 000
    while (client.available() < 9); // make sure receive 9 bytes from server
    client.readBytes(reinterpret_cast<uint8_t*>(&pair), 9); // read bytes then store in pair

    if (pair[0] != server_Header) {
        Serial.printf("Invalid packet id for pair data: %d", pair[0]);
        return;
    }

    *address = IPAddress(pair[1], pair[2], pair[3], pair[4]);
    *port = pair[5] << 8 | pair[6];
    *id = pair[7] << 8 | pair[8];
}

/**
 * Synchronization constants.
*/
// micros
#define SYNC_DELAY 500000
// millis
#define UDP_WAIT 500

/**
 * Sends a ping to the other device and then waits for a pong.
 * This is a time critical function, and no extra logic should be run, ie Serial.print.
*/
int sendPing(WiFiUDP* udp, IPAddress pairIp) {
    udp->beginPacket(pairIp, 1234);
    udp->write('\x00');

    delay(UDP_WAIT); // Make sure that the other device is online
    uint32_t time = micros();
    udp->endPacket(); // Send ping

    while (udp->parsePacket() < 1); // Wait for pong
    uint32_t dt = micros() - time;

    // Wait SYNC_DELAY minus a one way packet send
    // 23.5ms for manual adjustment
    return (SYNC_DELAY - dt / 4 - 23500);
}

/**
 * Waits for a ping from the other device and then sends a pong.
 * This is a time critical function, and no extra logic should be run, ie Serial.print.
*/
int sendPong(WiFiUDP* udp, IPAddress peer_ip) {
    udp->begin(1234);

    // Wait for ping
    while (udp->parsePacket() < 1);
    assert(udp->remoteIP() == peer_ip);

    udp->beginPacket(udp->remoteIP(), udp->remotePort());
    udp->write('\x00');
    udp->endPacket(); // send pong

    // Wait SYNC_DELAY for travel
    return SYNC_DELAY;
}

IPAddress pairIp;
uint16_t pairPort, pairId;

void connect() {
    // Connect to App
    sendId(1);
    Serial.printf("Sent id.\n");

    // Get Pair information
    receivePair(&pairIp, &pairPort, &pairId);
    Serial.printf("Pair at %s:%d with ID %d\n", pairIp.toString(), pairPort, pairId);
}

// Create the UDP client statically to reduce sync time.
WiFiUDP* udp = new WiFiUDP();
void synchronize() {
    int delayTime = 0;
    if (id < pairId) {
        delayTime = sendPong(udp, pairIp);
    } else {
        delayTime = sendPing(udp, pairIp);
    }
    delayMicroseconds(delayTime);
}


const uint16_t maxSamples = 9000 * 3;
/**
 * Statically allocated sample data. This makes sure we dont have a stack overflow.
 * Stores the X, Y[, Z] channels repeatedly.
*/
int16_t sampleData [maxSamples];

int16_t* collectData(BaseSensor& sensor) {
    const uint32_t SAMPLE_MICROS = 1000000; // microsec for 1 sec

    // Collect the sample data
    uint32_t start = micros();
    uint16_t collectedSamples = 0;
    while (micros() - start < SAMPLE_MICROS) {
        if (collectedSamples >= maxSamples) {
            continue;
        }
        
        sensor.read(&sampleData[collectedSamples]);
        collectedSamples += sensor.axes();

        if (collectedSamples >= maxSamples) {
            Serial.printf("WARNING, collected too many samples! dt: %06d\n", micros() - start);
        }
    }
    Serial.printf("Collected %d samples in 1 second!\n", collectedSamples);

    /*// Send the data header packet
    char data[] = {
        sensorData_Header,
        (collectedSamples >> 8) & 0xFF,
        (collectedSamples) & 0xFF,
        sensor.axes()
    };
    client.write(data, sizeof(data));
    Serial.printf("Sent data header to server.\n");

    // Send the data to the server, in chunks of 1000 samples to not overwhelm the ESP.
    for (int i = 0; i < collectedSamples; i += 1000) {
        size_t sending = 2 * 1000;

        if (i + 1000 > collectedSamples) { // Make sure we dont read past the end of the buffer
            sending = 2 * (collectedSamples - i);
        }

        client.write((char *) (&sampleData[i]), sending);
    }
    Serial.printf("Sent data to server.\n");*/
    
    // send data to PC，every sample costs 2 bytes（int16_t）
    for (int i = 0; i < collectedSamples; i++) {
        Serial.printf("%d ", sampleData[i]); 
    }

    Serial.println("\nEND_DATA"); // end of data sending
    Serial.flush(); // make sure sent

    Serial.println("Data Transmission to PC Complete.");

    return sampleData;
}

void goToSleep(uint32_t sleep_time) {
    Serial.printf("Going to sleep for %d seconds...\n", sleep_time);
    
    esp_sleep_enable_timer_wakeup(sleep_time * 1000000ULL); // unit: microS
    esp_deep_sleep_start();  // deep sleep
}

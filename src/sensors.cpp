#include <sensor.h>

#include <Arduino.h>
#include <SPI.h>

#define Sensor_CS 21

/**
 * Reads data from SPI.
*/
void spiRead(uint8_t address, uint8_t* data, uint8_t len) {
    digitalWrite(Sensor_CS, LOW); // trigger SPI for sensor
    SPI.transfer(address | 0b10000000); // set read mode (bit7 =1
    for (int i = 0; i < len; i ++) {
        data[i] = SPI.transfer(0x00);
    }
    digitalWrite(Sensor_CS, HIGH);
}

/**
 * Reads a single byte from SPI.
*/
uint8_t spiRead(uint8_t address) {
    uint8_t val = 0;
    spiRead(address, &val, 1);
    return val;
}

/**
 * Writes data to SPI.
*/
void spiWrite(uint8_t address, uint8_t* data, uint8_t len) {
    digitalWrite(Sensor_CS, LOW);
    SPI.transfer(address | 0b00000000);
    for (int i = 0; i < len; i ++) {
        SPI.transfer(data[i]);
    }
    digitalWrite(Sensor_CS, HIGH);
}

/**
 * Writes a single byte to SPI.
*/
void spiWrite(uint8_t address, uint8_t data) {
    spiWrite(address, &data, 1);
}

// checkWhoAmI() is used for debug
void checkWhoAmI() {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));  // try SPI_MODE3
    digitalWrite(5, LOW);  // make sure cs low
    SPI.transfer(0x8F);  // 0x0F | 0x80 (read WHO_AM_I)
    uint8_t whoami = SPI.transfer(0x00);  // 
    digitalWrite(5, HIGH);
    SPI.endTransaction();

    Serial.printf("handy WHO_AM_I register: 0x%02X\n", whoami);
}

// wrapper class for LIS3DH sensor
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define STATUS 0x27
#define LIS3DH_REG_OUT_X_L 0x28
#define LIS3DH_REG_OUT_X_H 0x29
#define LIS3DH_REG_OUT_Y_L 0x2A
#define LIS3DH_REG_OUT_Y_H 0x2B
#define LIS3DH_REG_OUT_Z_L 0x2C
#define LIS3DH_REG_OUT_Z_H 0x2D

LIS3DH::LIS3DH() {
    
}

void LIS3DH::initialize() {
    pinMode(Sensor_CS, OUTPUT);
    //SPI.begin();

    // Read WHO_AM_I to ensure correct device
    //checkWhoAmI();
    uint8_t whoami = spiRead(WHO_AM_I);

    if (whoami != 0x33) { //0x11 is the default ID for LIS3DHH, 0x33 is for LIS3DH
        Serial.printf("Found wrong WAI: %02x", whoami);
        while (true) {}
    }
    Serial.println("sensor detected!");

    // SW Reset
    //spiWrite(CTRL_REG1, 0x04);

    // SW Reset（LIS3DH's reset register is CTRL_REG2 (0x21)）
    spiWrite(0x21, 0x04);  // reset
    delay(10);  // wait for reset

    // set CTRL_REG1（enable XYZ axis，sample rate: 100Hz）
    spiWrite(CTRL_REG1, 0x57);  // 0x57 = 100Hz 
    
    // Wait for reset to go low
    //while ((spiRead(CTRL_REG1) & 0x4) != 0);

    //spiWrite(CTRL_REG1, 0x80 | 0x40 | 0x01); // Enable the device, SPI Addr Increment, and BDU

    Serial.printf("LIS3DHH sucessfully initalized.\n");
}

void LIS3DH::read(int16_t* storage) {
    uint8_t buffer[6] = { 0 }; // 6 8bits data: xl xh yl yh zl zh

    //  To perform a multiple-byte read or write, bit 6 of the sub-address field must be set to 1.
        //  0xC0 = 0x80 (read) + 0x40 (auto-increment)
    spiRead(LIS3DH_REG_OUT_X_L | 0xC0, buffer, 6); 

    storage[0] = (int16_t)((buffer[1] << 8) | buffer[0]); // X, 2Bytes
    storage[1] = (int16_t)((buffer[3] << 8) | buffer[2]); // Y, 2Bytes
    storage[2] = (int16_t)((buffer[5] << 8) | buffer[4]); // Z, 2Bytes
}


// Wrapper class for the mma sensor
MMA8451::MMA8451(): mma() {
}

void MMA8451::initialize() {
    if (!this->mma.begin()) {
        Serial.println("Unable to connect to MMA");
        while(1);
    }

    this->mma.setRange(MMA8451_RANGE_2_G);
    this->mma.setDataRate(MMA8451_DATARATE_800_HZ);
    Serial.printf("MMA8451 sucessfully initalized.\n");
}

void MMA8451::read(int16_t* storage) {
    this->mma.read();

    storage[0] = this->mma.x;
    storage[1] = this->mma.y;
    storage[2] = this->mma.z;
}

// TODO: Implement other sensors
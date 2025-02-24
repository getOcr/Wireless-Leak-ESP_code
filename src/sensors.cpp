#include <sensor.h>

#include <Arduino.h>
#include <SPI.h>

#define SPI_CS 5
/**
 * Reads data from SPI.
*/
void spiRead(uint8_t address, uint8_t* data, uint8_t len) {
    digitalWrite(SPI_CS, LOW);
    SPI.transfer(address | 0b10000000);
    for (int i = 0; i < len; i ++) {
        data[i] = SPI.transfer(0x00);
    }
    digitalWrite(SPI_CS, HIGH);
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
    digitalWrite(SPI_CS, LOW);
    SPI.transfer(address | 0b00000000);
    for (int i = 0; i < len; i ++) {
        SPI.transfer(data[i]);
    }
    digitalWrite(SPI_CS, HIGH);
}

/**
 * Writes a single byte to SPI.
*/
void spiWrite(uint8_t address, uint8_t data) {
    spiWrite(address, &data, 1);
}

/**
 * LIS3DHH Registers.
*/
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define STATUS 0x27
#define X_LOW 0x28

LIS3DHH::LIS3DHH() {
    
}

void LIS3DHH::initialize() {
    pinMode(SPI_CS, OUTPUT);
    SPI.begin();

    // Read WHO_AM_I to ensure correct device
    checkWhoAmI();
    uint8_t whoami = spiRead(WHO_AM_I);

    if (whoami != 0x33) { //0x11 is the default ID
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
void checkWhoAmI() {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));  // 试试 SPI_MODE3
    digitalWrite(5, LOW);  // 确保 CS 正确拉低
    SPI.transfer(0x8F);  // 0x0F | 0x80 (读取 WHO_AM_I)
    uint8_t whoami = SPI.transfer(0x00);  // 读取返回值
    digitalWrite(5, HIGH);
    SPI.endTransaction();

    Serial.printf("handy WHO_AM_I register: 0x%02X\n", whoami);
}

void LIS3DHH::read(int16_t* storage) {
    uint8_t data[6] = { 0 };

    spiRead(X_LOW, data, 6); 

    storage[0] = (int16_t) ((((uint16_t) data[1]) << 8) | data[0]);
    storage[1] = (int16_t) ((((uint16_t) data[3]) << 8) | data[2]);
    storage[2] = (int16_t) ((((uint16_t) data[5]) << 8) | data[4]);
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
#pragma once

#include <stdint.h>

#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

/**
 * A base abstraction for the sensors.
*/
class BaseSensor {
public:
    /**
     * Initializes the sensor.
    */
    virtual void initalize();

    /**
     * Reads values from the accelerometer and stores them into the pointer.
     * Write exactly the number of `int16_t`s as `axes` returns
    */
    virtual void read(int16_t* storage);

    /**
     * The number of axes supported by the accelerometer
    */
    virtual uint8_t axes();
};

/**
 * Handling for the LIS3DHH sensor from ST.
 * Datasheet: https://www.st.com/resource/en/datasheet/lis3dhh.pdf
 * Code Example: https://github.com/STMicroelectronics/lis3dhh-pid
*/
class LIS3DHH : public BaseSensor {
public:
    LIS3DHH();
    void initalize();
    void read(int16_t* storage);
    uint8_t axes() { return 3; };
};

/**
 * Wrapper class for the MMA8451 sensor from Adafruit.
*/
class MMA8451 : public BaseSensor {
public:
    MMA8451();
    void initalize();
    void read(int16_t* storage);
    uint8_t axes() { return 3; };
private: 
    Adafruit_MMA8451 mma;
};

/**
 * Handling for the IIS2ICLX sensor from ST.
 * Datasheet: https://www.st.com/resource/en/datasheet/iis2iclx.pdf
 * Code Example: https://github.com/STMicroelectronics/iis2iclx-pid
*/
// TODO: Create Implementation

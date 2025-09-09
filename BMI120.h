/******************************************************************************
 * BMI120.h
 *
 * C++ Library for the Bosch BMI120 6-DoF IMU.
 * This library supports both I2C and SPI communication.
 *
 * Merged and refactored from separate I2C/SPI libraries.
 ******************************************************************************/

#ifndef BMI120_H
#define BMI120_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// I2C Addresses
#define BMI120_I2C_ADDR_SECONDARY 0x69 // SDO pin is HIGH
#define BMI120_I2C_ADDR_PRIMARY   0x68 // SDO pin is LOW

// Register Addresses
#define BMI120_REG_CHIP_ID      0x00
#define BMI120_REG_PMU_STATUS   0x03 // Power management status
#define BMI120_REG_DATA_START   0x0C // Starts with Gyro data
#define BMI120_REG_ACC_CONF     0x40
#define BMI120_REG_ACC_RANGE    0x41
#define BMI120_REG_GYR_CONF     0x42
#define BMI120_REG_GYR_RANGE    0x43
#define BMI120_REG_CMD          0x7E

// Command Register Commands
#define BMI120_CMD_ACC_NORMAL   0x11
#define BMI120_CMD_GYR_NORMAL   0x15
#define BMI120_CMD_SOFT_RESET   0xB6

// Chip ID
#define BMI120_CHIP_ID_VAL      0xD3

// Gravity Constant
#define GRAVITY_MSS 9.80665f

// Enums for configuration to make code more readable
enum AccelRange {
    RANGE_2G  = 0x03,
    RANGE_4G  = 0x05,
    RANGE_8G  = 0x08,
    RANGE_16G = 0x0C
};

enum GyroRange {
    RANGE_2000DPS = 0x00,
    RANGE_1000DPS = 0x01,
    RANGE_500DPS  = 0x02,
    RANGE_250DPS  = 0x03,
    RANGE_125DPS  = 0x04
};

/**
 * @class BMI120_Bus
 * @brief Abstract base class to handle different communication protocols (I2C/SPI).
 * This is an internal helper class.
 */
class BMI120_Bus {
public:
    virtual ~BMI120_Bus() {}
    virtual bool begin() = 0;
    virtual void writeByte(uint8_t reg, uint8_t val) = 0;
    virtual uint8_t readByte(uint8_t reg) = 0;
    virtual void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) = 0;
};

/**
 * @class BMI120
 * @brief Main class for the BMI120 sensor.
 */
class BMI120 {
public:
    // Public variables for raw and calculated sensor data
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    float ax_mps2, ay_mps2, az_mps2;
    float gx_dps, gy_dps, gz_dps;

    // --- Constructors ---
    // Constructor for I2C communication
    BMI120(TwoWire &wire = Wire);
    // Constructor for SPI communication
    BMI120(int8_t cs_pin, SPIClass &spi = SPI);
    
    // Destructor
    ~BMI120();

    // --- Public Methods ---
    // Initialization and Communication Test
    bool begin(uint8_t address = BMI120_I2C_ADDR_SECONDARY); // Address is for I2C only
    bool beginSPI(); // Convenience method for SPI begin

    // Configuration
    void setAccelRange(AccelRange range);
    void setGyroRange(GyroRange range);
    void softReset();

    // Data Reading
    bool readSensor();

private:
    enum CommsMode { I2C_MODE, SPI_MODE };
    CommsMode _mode;
    BMI120_Bus* _bus; // Pointer to the bus interface object (I2C or SPI)

    float _accel_sensitivity;
    float _gyro_sensitivity;
};

#endif // BMI120_H


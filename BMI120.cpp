/******************************************************************************
 * BMI120.cpp
 *
 * C++ Library for the Bosch BMI120 6-DoF IMU.
 * This library supports both I2C and SPI communication.
 ******************************************************************************/

#include "BMI120.h"

//==============================================================================
// INTERNAL HELPER CLASSES FOR I2C AND SPI BUS
//==============================================================================

/**
 * @class BMI120_I2C_Bus
 * @brief Handles I2C communication for the BMI120.
 */
class BMI120_I2C_Bus : public BMI120_Bus {
public:
    BMI120_I2C_Bus(TwoWire &wire, uint8_t addr) : _wire(&wire), _i2c_addr(addr) {}

    bool begin() override {
        _wire->begin();
        return (readByte(BMI120_REG_CHIP_ID) == BMI120_CHIP_ID_VAL);
    }

    void writeByte(uint8_t reg, uint8_t val) override {
        _wire->beginTransmission(_i2c_addr);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }

    uint8_t readByte(uint8_t reg) override {
        uint8_t buffer;
        readBytes(reg, &buffer, 1);
        return buffer;
    }

    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) override {
        _wire->beginTransmission(_i2c_addr);
        _wire->write(reg);
        _wire->endTransmission(false); // Send repeated start
        _wire->requestFrom((uint8_t)_i2c_addr, len);
        
        for (uint8_t i = 0; i < len; i++) {
            if (_wire->available()) {
                buffer[i] = _wire->read();
            }
        }
    }

private:
    TwoWire* _wire;
    uint8_t _i2c_addr;
};


/**
 * @class BMI120_SPI_Bus
 * @brief Handles SPI communication for the BMI120.
 */
class BMI120_SPI_Bus : public BMI120_Bus {
public:
    BMI120_SPI_Bus(int8_t cs_pin, SPIClass &spi) : _cs_pin(cs_pin), _spi(&spi) {
         _spi_settings = SPISettings(1000000, MSBFIRST, SPI_MODE0);
    }

    bool begin() override {
        pinMode(_cs_pin, OUTPUT);
        digitalWrite(_cs_pin, HIGH);
        _spi->begin();
        return (readByte(BMI120_REG_CHIP_ID) == BMI120_CHIP_ID_VAL);
    }

    void writeByte(uint8_t reg, uint8_t val) override {
        _spi->beginTransaction(_spi_settings);
        digitalWrite(_cs_pin, LOW);
        _spi->transfer(reg & 0x7F); // Set MSB to 0 for write
        _spi->transfer(val);
        digitalWrite(_cs_pin, HIGH);
        _spi->endTransaction();
    }

    uint8_t readByte(uint8_t reg) override {
        uint8_t data;
        _spi->beginTransaction(_spi_settings);
        digitalWrite(_cs_pin, LOW);
        _spi->transfer(reg | 0x80); // Set MSB to 1 for read
        data = _spi->transfer(0x00); // Send dummy byte to receive data
        digitalWrite(_cs_pin, HIGH);
        _spi->endTransaction();
        return data;
    }
    
    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) override {
        _spi->beginTransaction(_spi_settings);
        digitalWrite(_cs_pin, LOW);
        _spi->transfer(reg | 0x80); // Set MSB to 1 for read
        for (uint8_t i = 0; i < len; i++) {
            buffer[i] = _spi->transfer(0x00);
        }
        digitalWrite(_cs_pin, HIGH);
        _spi->endTransaction();
    }

private:
    int8_t _cs_pin;
    SPIClass* _spi;
    SPISettings _spi_settings;
};

//==============================================================================
// BMI120 MAIN CLASS IMPLEMENTATION
//==============================================================================

// I2C Constructor
BMI120::BMI120(TwoWire &wire) {
    _bus = new BMI120_I2C_Bus(wire, BMI120_I2C_ADDR_SECONDARY); // Default address
    _mode = I2C_MODE;
    _accel_sensitivity = 0;
    _gyro_sensitivity = 0;
}

// SPI Constructor
BMI120::BMI120(int8_t cs_pin, SPIClass &spi) {
    _bus = new BMI120_SPI_Bus(cs_pin, spi);
    _mode = SPI_MODE;
    _accel_sensitivity = 0;
    _gyro_sensitivity = 0;
}

// Destructor
BMI120::~BMI120() {
    delete _bus;
}

// Initialization
bool BMI120::begin(uint8_t address) {
    // If this is an I2C object, recreate it with the correct user-specified address
    if (_mode == I2C_MODE) {
        delete _bus;
        _bus = new BMI120_I2C_Bus(Wire, address);
    }
    
    if (!_bus->begin()) {
        return false; // Communication failed or wrong sensor
    }

    // Perform a soft reset to ensure a clean state
    softReset();
    delay(50); // Wait for reset to complete

    // --- Robust Initialization Sequence ---
    // 1. Explicitly set a valid configuration for the accelerometer before turning it on.
    // 0x28 = 100Hz ODR, normal bandwidth. This is a common default.
    _bus->writeByte(BMI120_REG_ACC_CONF, 0x28);
    delay(5);

    // 2. Set Accelerometer to Normal Mode
    _bus->writeByte(BMI120_REG_CMD, BMI120_CMD_ACC_NORMAL);
    delay(50); // Recommended startup time from datasheet

    // 3. Set Gyroscope to Normal Mode
    _bus->writeByte(BMI120_REG_CMD, BMI120_CMD_GYR_NORMAL);
    delay(100); // Recommended startup time from datasheet

    // 4. Set default ranges (these can be changed by the user later)
    setAccelRange(RANGE_2G);
    setGyroRange(RANGE_2000DPS);

    // 5. Verify that the accelerometer is in normal mode.
    // The PMU status bits for the accelerometer are [5:4] in register 0x03.
    // A value of 0b01 indicates normal mode.
    uint8_t pmu_status = _bus->readByte(BMI120_REG_PMU_STATUS);
    if ((pmu_status & 0b00110000) != 0b00010000) {
        return false; // Accelerometer failed to enter normal mode.
    }

    return true;
}

// Convenience wrapper for SPI begin
bool BMI120::beginSPI() {
    return begin(0); // Address doesn't matter for SPI
}

// Perform a soft reset
void BMI120::softReset() {
    _bus->writeByte(BMI120_REG_CMD, BMI120_CMD_SOFT_RESET);
}

// Set Accelerometer Range
void BMI120::setAccelRange(AccelRange range) {
    _bus->writeByte(BMI120_REG_ACC_RANGE, (uint8_t)range);
    switch(range) {
        case RANGE_2G:  _accel_sensitivity = 16384.0f; break;
        case RANGE_4G:  _accel_sensitivity = 8192.0f;  break;
        case RANGE_8G:  _accel_sensitivity = 4096.0f;  break;
        case RANGE_16G: _accel_sensitivity = 2048.0f;  break;
    }
}

// Set Gyroscope Range
void BMI120::setGyroRange(GyroRange range) {
    _bus->writeByte(BMI120_REG_GYR_RANGE, (uint8_t)range);
    switch(range) {
        case RANGE_2000DPS: _gyro_sensitivity = 16.4f;  break;
        case RANGE_1000DPS: _gyro_sensitivity = 32.8f;  break;
        case RANGE_500DPS:  _gyro_sensitivity = 65.6f;  break;
        case RANGE_250DPS:  _gyro_sensitivity = 131.2f; break;
        case RANGE_125DPS:  _gyro_sensitivity = 262.4f; break;
    }
}

// Read all sensor data
bool BMI120::readSensor() {
    uint8_t buffer[12];
    // Perform a burst read of 12 bytes, starting from the gyro data registers
    _bus->readBytes(BMI120_REG_DATA_START, buffer, 12);

    // Combine bytes into 16-bit integers (Little Endian)
    gx_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    gy_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
    gz_raw = (int16_t)((buffer[5] << 8) | buffer[4]);

    ax_raw = (int16_t)((buffer[7] << 8) | buffer[6]);
    ay_raw = (int16_t)((buffer[9] << 8) | buffer[8]);
    az_raw = (int16_t)((buffer[11] << 8) | buffer[10]);

    // Convert raw data to standard units
    ax_mps2 = -(ax_raw / _accel_sensitivity) * GRAVITY_MSS;
    ay_mps2 = -(ay_raw / _accel_sensitivity) * GRAVITY_MSS;
    az_mps2 = -(az_raw / _accel_sensitivity) * GRAVITY_MSS;

    gx_dps = gx_raw / _gyro_sensitivity;
    gy_dps = gy_raw / _gyro_sensitivity;
    gz_dps = gz_raw / _gyro_sensitivity;

    return true;
}


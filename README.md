# Arduino Library for Bosch BMI120 6-DoF IMU

This is a versatile and easy-to-use Arduino library for the Bosch BMI120 6-axis Inertial Measurement Unit (IMU), supporting both I2C and SPI communication protocols with a single, unified interface.

---

## Background & Motivation
**A note from the author:**  
I created this library after purchasing a sensor from AliExpress that was listed as an LSM6DS3. After discovering the I2C address and chip ID (0xD3) didn't match, I found it was actually a Bosch BMI120. This library provides a reliable solution for anyone who has this sensor.

---

## Features
- **Dual Communication Protocol:** Seamlessly works with both I2C and SPI.  
- **Unified Interface:** Use the same functions to read sensor data for both protocols.  
- **Easy Initialization:** Simple `begin()` methods to get you started quickly.  
- **Configurable Ranges:** Adjust accelerometer and gyroscope measurement ranges.  
- **Example Sketches:** Includes clear examples for both I2C and SPI.  

---

## Installation
1. Download the library as a ZIP file from the repository.  
2. In the Arduino IDE, navigate to **Sketch > Include Library > Add .ZIP Library...**  
3. Select the downloaded ZIP file to install.  
4. Examples are available in **>>test**.  

---

## How to Use

### 1. Include the Library
```cpp
#include <BMI120.h> 
```
### 2. Initialize the Sensor
  For I²C Communication:

```cpp
BMI120 myIMU; // Uses Wire by default

void setup() {
    Serial.begin(115200);
    // Use BMI120_I2C_ADDR_PRIMARY or BMI120_I2C_ADDR_SECONDARY
    if (!myIMU.begin(BMI120_I2C_ADDR_SECONDARY)) {
        Serial.println("Failed to initialize I2C IMU!");
        while (1);
    }
}

```
  For SPI Communication:
```cpp
#define CS_PIN D7 // Your chip select pin
BMI120 myIMU(CS_PIN); // Pass the CS pin to the constructor

void setup() {
    Serial.begin(115200);
    if (!myIMU.beginSPI()) {
        Serial.println("Failed to initialize SPI IMU!");
        while (1);
    }
}

```
### 3. Reading Sensor Data
```cpp
void loop() {
    if (myIMU.readSensor()) {
        // Accelerometer data in m/s^2
        Serial.print("Accel X: ");
        Serial.print(myIMU.ax_mps2);
        // Gyroscope data in degrees/s
        Serial.print("\t Gyro X: ");
        Serial.println(myIMU.gx_dps);
    }
    delay(100);
}

```
### 4. Changing Sensor Ranges
```cpp
// Set accelerometer range to ±8g
myIMU.setAccelRange(RANGE_8G);

// Set gyroscope range to ±500 degrees per second
myIMU.setGyroRange(RANGE_500DPS);

```
## Contributing

Contributions are welcome! If you have improvements or bug fixes, please open an issue or submit a pull request on the GitHub repository.

#include <Wire.h>
#include <SPI.h>
#include "BMI120.h"

// --- Configuration ---
// I2C Configuration
const uint8_t I2C_ADDR = BMI120_I2C_ADDR_SECONDARY; // 0x69

// SPI Configuration
const int CS_PIN_1 = 3;  // CS for first SPI IMU
const int CS_PIN_2 = 7;  // CS for second SPI IMU

// --- Sensor Objects ---
BMI120 imu_i2c;          // For I2C
BMI120 imu_spi1(CS_PIN_1); // For first SPI sensor
BMI120 imu_spi2(CS_PIN_2); // For second SPI sensor

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Multiple BMI120 IMU Example");

  // Initialize I2C IMU
  if (imu_i2c.begin(I2C_ADDR)) {
    Serial.println("I2C IMU initialized successfully.");
    imu_i2c.setAccelRange(RANGE_4G);
    imu_i2c.setGyroRange(RANGE_1000DPS);
    Serial.println("I2C IMU ranges set to 4G and 1000dps.");
  } else {
    Serial.println("I2C IMU initialization failed!");
  }

  // Initialize first SPI IMU
  if (imu_spi1.beginSPI()) {
    Serial.println("SPI IMU 1 (CS_PIN_1) initialized successfully.");
    imu_spi1.setAccelRange(RANGE_4G);
    imu_spi1.setGyroRange(RANGE_1000DPS);
    Serial.println("SPI IMU 1 ranges set to 4G and 1000dps.");
  } else {
    Serial.println("SPI IMU 1 (CS_PIN_1) initialization failed!");
  }

  // Initialize second SPI IMU
  if (imu_spi2.beginSPI()) {
    Serial.println("SPI IMU 2 (CS_PIN_2) initialized successfully.");
    imu_spi2.setAccelRange(RANGE_4G);
    imu_spi2.setGyroRange(RANGE_1000DPS);
    Serial.println("SPI IMU 2 ranges set to 4G and 1000dps.");
  } else {
    Serial.println("SPI IMU 2 (CS_PIN_2) initialization failed!");
  }

  Serial.println("\nStarting sensor readings...");
}

void loop() {
  // Read and print data from I2C IMU
  if (imu_i2c.readSensor()) {
    Serial.println("--- I2C IMU (0x69) ---");
    printSensorData(imu_i2c);
  }

  // Read and print data from first SPI IMU
  if (imu_spi1.readSensor()) {
    Serial.println("--- SPI IMU 1 (CS D3) ---");
    printSensorData(imu_spi1);
  }

  // Read and print data from second SPI IMU
  if (imu_spi2.readSensor()) {
    Serial.println("--- SPI IMU 2 (CS D7) ---");
    printSensorData(imu_spi2);
  }

  delay(1000); 
}


void printSensorData(BMI120 &sensor) {
  Serial.print("  Accel: ");
  Serial.print(sensor.ax_mps2, 2); Serial.print(", ");
  Serial.print(sensor.ay_mps2, 2); Serial.print(", ");
  Serial.println(sensor.az_mps2, 2);

  Serial.print("  Gyro:  ");
  Serial.print(sensor.gx_dps, 2); Serial.print(", ");
  Serial.print(sensor.gy_dps, 2); Serial.print(", ");
  Serial.println(sensor.gz_dps, 2);
  Serial.println();
}

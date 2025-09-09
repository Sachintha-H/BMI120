#include <Wire.h>
#include <SPI.h>
#include "BMI120.h"


const int CS_PIN_1 = D3; 
const int CS_PIN_2 = D7; 

// --- Sensor Object Creation ---
// 1. Create an instance for the I2C IMU.
BMI120 imuI2C; 

// 2. Create an instance for the first SPI IMU.
BMI120 imuSPI1(CS_PIN_1);

// 3. Create an instance for the second SPI IMU.
BMI120 imuSPI2(CS_PIN_2);


void setup() {
  Serial.begin(115200);
  while (!Serial); 

  Serial.println("Multiple BMI120 IMU Example (I2C & SPI)");
  Serial.println("----------------------------------------");

  // --- Initialize I2C IMU ---
  if (imuI2C.begin()) {
    Serial.println("I2C IMU initialization successful!");
  } else {
    Serial.println("I2C IMU initialization failed. Check wiring and address.");
  }
  
  // --- Initialize First SPI IMU ---
  if (imuSPI1.beginSPI()) {
    Serial.println("SPI IMU #1 (Pin D3) initialization successful!");
  } else {
    Serial.println("SPI IMU #1 (Pin D3) initialization failed. Check wiring.");
  }

  // --- Initialize Second SPI IMU ---
  if (imuSPI2.beginSPI()) {
    Serial.println("SPI IMU #2 (Pin D7) initialization successful!");
  } else {
    Serial.println("SPI IMU #2 (Pin D7) initialization failed. Check wiring.");
  }
  Serial.println("----------------------------------------\n");
}

void loop() {
  // Read and print data from the I2C sensor
  Serial.println("--- I2C Sensor Data ---");
  if (imuI2C.readSensor()) {
    printSensorData(imuI2C);
  } else {
    Serial.println("Failed to read from I2C sensor.");
  }

  // Read and print data from the first SPI sensor
  Serial.println("\n--- SPI Sensor #1 (D3) Data ---");
  if (imuSPI1.readSensor()) {
    printSensorData(imuSPI1);
  } else {
    Serial.println("Failed to read from SPI sensor #1.");
  }

  // Read and print data from the second SPI sensor
  Serial.println("\n--- SPI Sensor #2 (D7) Data ---");
  if (imuSPI2.readSensor()) {
    printSensorData(imuSPI2);
  } else {
    Serial.println("Failed to read from SPI sensor #2.");
  }

  Serial.println("\n========================================");
  
  delay(100); 
}


void printSensorData(BMI120 &imu) {
  Serial.print(" Accel: X=");
  Serial.print(imu.ax_mps2, 2);
  Serial.print(" Y=");
  Serial.print(imu.ay_mps2, 2);
  Serial.print(" Z=");
  Serial.print(imu.az_mps2, 2);
  Serial.println(" m/s^2");

  Serial.print(" Gyro:  X=");
  Serial.print(imu.gx_dps, 2);
  Serial.print(" Y=");
  Serial.print(imu.gy_dps, 2);
  Serial.print(" Z=");
  Serial.print(imu.gz_dps, 2);
  Serial.println(" dps");
}

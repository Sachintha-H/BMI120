#include <Wire.h>
#include "BMI120.h"


BMI120 myIMU; 

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("BMI120 I2C Example");

  // Initialize the sensor. You can specify the I2C address if needed.
  // Default is 0x69 (SDO HIGH). Use 0x68 for SDO LOW.
  if (myIMU.begin(BMI120_I2C_ADDR_SECONDARY)) {
    Serial.println("BMI120 initialization successful!");
  } else {
    Serial.println("BMI120 initialization failed. Check wiring.");
    while (1);
  }
  
  // Optionally, change the sensor ranges
  // myIMU.setAccelRange(RANGE_4G);
  // myIMU.setGyroRange(RANGE_500DPS);
}

void loop() {
  // Read sensor data
  if (myIMU.readSensor()) {
    // Print accelerometer data (in m/s^2)
    Serial.print("Accel X: ");
    Serial.print(myIMU.ax_mps2, 2);
    Serial.print(" m/s^2\t");
    Serial.print("Accel Y: ");
    Serial.print(myIMU.ay_mps2, 2);
    Serial.print(" m/s^2\t");
    Serial.print("Accel Z: ");
    Serial.print(myIMU.az_mps2, 2);
    Serial.println(" m/s^2");

    // Print gyroscope data (in degrees/s)
    Serial.print("Gyro X: ");
    Serial.print(myIMU.gx_dps, 2);
    Serial.print(" dps\t");
    Serial.print("Gyro Y: ");
    Serial.print(myIMU.gy_dps, 2);
    Serial.print(" dps\t");
    Serial.print("Gyro Z: ");
    Serial.print(myIMU.gz_dps, 2);
    Serial.println(" dps");
    
    Serial.println("------------------------------------");
  }

  delay(100); 
}

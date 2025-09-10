#include <SPI.h>
#include "BMI120.h"


const int CS_PIN = D7; 

BMI120 myIMU(CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial); 

  Serial.println("BMI120 SPI Example");


  if (myIMU.beginSPI()) {
    Serial.println("BMI120 initialization successful!");

    // Set the accelerometer range to +/- 4G
    myIMU.setAccelRange(RANGE_4G);
    Serial.println("Accelerometer range set to +/- 4G.");

    // Set the gyroscope range to +/- 1000 dps
    myIMU.setGyroRange(RANGE_1000DPS);
    Serial.println("Gyroscope range set to +/- 1000 dps.");

  } else {
    Serial.println("BMI120 initialization failed. Check wiring.");
    while (1);
  }
}

void loop() {

  if (myIMU.readSensor()) {

    Serial.print("Accel X: ");
    Serial.print(myIMU.ax_mps2, 2);
    Serial.print(" m/s^2\t");
    Serial.print("Accel Y: ");
    Serial.print(myIMU.ay_mps2, 2);
    Serial.print(" m/s^2\t");
    Serial.print("Accel Z: ");
    Serial.print(myIMU.az_mps2, 2);
    Serial.println(" m/s^2");


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

  delay(500); 
}


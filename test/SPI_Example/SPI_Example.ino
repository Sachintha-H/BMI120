#include <SPI.h>
#include "BMI120.h"


const int CS_PIN = D3; 


BMI120 myIMU(CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial); 

  Serial.println("BMI120 SPI Example");

  // Initialize the sensor using the specific SPI begin function.
  if (myIMU.beginSPI()) {
    Serial.println("BMI120 initialization successful!");
  } else {
    Serial.println("BMI120 initialization failed. Check wiring.");
    while (1);
  }
}

void loop() {
  // The rest of the code is identical to the I2C example
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

  delay(500); // Delay for readability
}

#include <Wire.h> 

#include <MPU6050.h> 

  

MPU6050 mpu; 

  

void setup() { 

  Serial.begin(9600); 

  Wire.begin(); 

  

  // Initialize MPU6050 

  mpu.initialize(); 

  

  // Check if the sensor is connected and responding 

  if (mpu.testConnection()) { 

    Serial.println("Sensor acknowledged."); 

  } else { 

    Serial.println("Sensor not acknowledged."); 

  } 

} 

  

void loop() { 

  // Nothing to do in the loop for this example 

} 
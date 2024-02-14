#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "IMU.h"

Adafruit_ICM20948 icm;

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(115200);
  IMU_init();
}

void loop() {
  // put your main code here, to run repeatedly:

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  float gyroX;
  float X_angle;
  float gyroY;
  float Y_angle;
  float gyroZ;
  float Z_angle;

  float accX;
  float velX;
  float disX;

  float accY;
  float velY;
  float disY;

  float accZ;
  float velZ;
  float disZ;


  //float gyroScaleFactor = 2000.0 / 32767.0;

  float deltaTime;
  float currentTime;
  float previousTime;
  
  while(1){
    
    icm.getEvent(&accel, &gyro, &temp, &mag);
    currentTime = millis();
    deltaTime = (currentTime - previousTime) / 1000.0;

    // read gyrometer's values
    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;

    // calculate angle change in 3 axis
    X_angle +=  gyroX * deltaTime;
    Y_angle +=  gyroY * deltaTime;
    Z_angle +=  gyroZ * deltaTime;

    // coordinate transformation according to orientation
    

    // read accelerometer's values
    accX = accel.acceleration.x;
    accY = accel.acceleration.y;
    accZ = accel.acceleration.z;

    // calculate velocity in 3 axis
    velX = accX * deltaTime;
    velY = accY * deltaTime;
    velZ = accZ * deltaTime;

    // calculate displacement in 3 axis
    disX = velX * deltaTime;
    disY = velY * deltaTime;
    disZ = velZ * deltaTime;

    //Serial.println(X_angle);
    //Serial.println(deltaTime);
    delay(10);

    previousTime = currentTime;
  }
}

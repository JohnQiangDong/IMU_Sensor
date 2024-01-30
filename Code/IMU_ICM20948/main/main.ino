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
  float gyroScaleFactor = 2000.0 / 32767.0;

  float deltaTime;
  float currentTime;
  float previousTime;
  
  while(1){
    
    icm.getEvent(&accel, &gyro, &temp, &mag);
    currentTime = millis();
    deltaTime = (currentTime - previousTime) / 1000.0;

    gyroX = gyro.gyro.x;
    //X_angle +=  gyroX * deltaTime;
    //Serial.println(X_angle);
    //Serial.println(deltaTime);
    delay(10);

    previousTime = currentTime;
  }
}

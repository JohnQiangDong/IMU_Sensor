// IMU.h
#ifndef _IMU_H
#define _IMU_H

#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


extern Adafruit_ICM20948 icm;




void IMU_init();


#endif
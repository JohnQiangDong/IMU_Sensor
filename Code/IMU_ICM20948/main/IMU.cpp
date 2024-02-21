#include "IMU.h"
#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

extern Adafruit_ICM20948 icm;
extern uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// For SPI mode, we need a CS pin
#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11


void IMU_init(){
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit ICM20948 test!");

    // Try to initialize!
    if (!icm.begin_I2C()) {
        // if (!icm.begin_SPI(ICM_CS)) {
        // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

        Serial.println("Failed to find ICM20948 chip");
        while (1) {
        delay(10);
        }
    }
    Serial.println("ICM20948 Found!");
    // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
    Serial.print("Accelerometer range set to: ");
    switch (icm.getAccelRange()) {
    case ICM20948_ACCEL_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case ICM20948_ACCEL_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case ICM20948_ACCEL_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case ICM20948_ACCEL_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    Serial.println("OK");

    // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    Serial.print("Gyro range set to: ");
    switch (icm.getGyroRange()) {
    case ICM20948_GYRO_RANGE_250_DPS:
        Serial.println("250 degrees/s");
        break;
    case ICM20948_GYRO_RANGE_500_DPS:
        Serial.println("500 degrees/s");
        break;
    case ICM20948_GYRO_RANGE_1000_DPS:
        Serial.println("1000 degrees/s");
        break;
    case ICM20948_GYRO_RANGE_2000_DPS:
        Serial.println("2000 degrees/s");
        break;
    }
}


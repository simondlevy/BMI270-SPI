/* Read BMI270 sensor via I^2C bus
 *
 * Copyright (c) 2023 Simon D. Levy
 *
 * MIT License
 *
 */

#include <Arduino.h>
#include <Wire.h>

#include <BMI270.h>

static const uint8_t INT_PIN = 14;

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static BMI270 imu = BMI270(Wire);

void setup(void)
{
    Serial.begin(115200);

    Wire.begin();

    imu.begin();

    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop(void) 
{
    if (gotInterrupt) {

        gotInterrupt = false;

        imu.readSensor();

        Serial.print("ax=");
        Serial.print(imu.getAccelX());
        Serial.print("  ay=");
        Serial.print(imu.getAccelY());
        Serial.print("  az=");
        Serial.print(imu.getAccelZ());

        Serial.print("  gx=");
        Serial.print(imu.getGyroX());
        Serial.print("  gy=");
        Serial.print(imu.getGyroY());
        Serial.print("  gz=");
        Serial.print(imu.getGyroZ());

        Serial.println();
    }

    delay(10);
}

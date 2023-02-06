/* Read BMI270 sensor via SPI bus
 *
 * Copyright (c) 2023 Simon D. Levy
 *
 * MIT License
 *
 */

#include <SPI.h>

#include "BMI270.h"

static const uint8_t CS_PIN  = 10;
static const uint8_t INT_PIN = 9;

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static BMI270 imu = BMI270(CS_PIN);

static void printval(const int16_t val, const char * label)
{
    Serial.print(label);
    Serial.print("=");
    char tmp[10];
    sprintf(tmp, "%+06d  ", val);
    Serial.print(tmp);
}

void setup(void)
{
    Serial.begin(115200);

    SPI.begin();

    imu.begin();

    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop(void) 
{
    if (gotInterrupt) {

        gotInterrupt = false;

        imu.readSensor();

        printval(imu.getAccelX(), "ax");
        printval(imu.getAccelY(), "ay");
        printval(imu.getAccelZ(), "az");

        printval(imu.getGyroX(), "gx");
        printval(imu.getGyroY(), "gy");
        printval(imu.getGyroZ(), "gz");

        Serial.println();
    }

    delay(10);
}

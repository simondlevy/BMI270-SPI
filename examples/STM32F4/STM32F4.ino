/* Read BMI270 sensor on STM32F4 MCU via SPI bus
 *
 * Copyright (c) 2023 Simon D. Levy
 *
 * MIT License
 *
 */

#include <SPI.h>

#include "BMI270.h"

static const uint8_t MOSI_PIN = PA7;
static const uint8_t MISO_PIN = PA6;
static const uint8_t SCLK_PIN = PA5;

static const uint8_t LED_PIN = PC13;
static const uint8_t CS_PIN  = PA4;
static const uint8_t INT_PIN = PA1;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static BMI270 imu = BMI270(spi, CS_PIN);

static void blinkLed(void)
{
    auto msec = millis();
    static uint32_t prev;

    if (msec - prev > 500) {
        static bool on;
        prev = msec;
        on = !on;
        digitalWrite(LED_PIN, on);
    }
}

void setup(void)
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    spi.begin();

    imu.begin();

    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop(void) 
{
    blinkLed();

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

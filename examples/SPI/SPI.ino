/*
   BMI270 SPI example

   This file is part of the BMI270 library.

   Copyright (c) 2019,2023 Arduino SA and Simon D. Levy. 
   All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "BMI270.h"

// Set to 0 for continuous polling
static const uint8_t INTERRUPT_PIN = 10;

// LED will blink when sensor is connected
static const uint8_t LED_PIN = 13;

BMI270 imu = BMI270(Wire);

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static void blinkLed(void)
{
    const auto msec = millis();
    static uint32_t prev;

    if (msec - prev > 500) {
        static bool on;
        digitalWrite(LED_PIN, on);
        on = !on;
        prev = msec;
    }

}

void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    if (INTERRUPT_PIN > 0) {
        pinMode(INTERRUPT_PIN, INPUT);
        attachInterrupt(INTERRUPT_PIN, handleInterrupt, RISING);
        imu.enableInterrupts();
    }

    Wire.begin();

    imu.begin();
}

static void report(void)
{
    if (imu.accelerationAvailable()) {

        int16_t x, y, z;

        imu.readAcceleration(x, y, z);

        Serial.print("accel: \t");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.println();
    }

    if (imu.gyroscopeAvailable()) {

        int16_t x, y, z;

        imu.readGyroscope(x, y, z);

        Serial.print("gyro: \t");
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(z);
        Serial.println();
    }
}

void loop() 
{
    if (INTERRUPT_PIN == 0 || gotInterrupt) {

        gotInterrupt = false;

        report();

    }

    blinkLed();
}

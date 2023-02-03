/*
   BMI270 I^2C example

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

static const uint8_t INTERRUPT_PIN   = 0;
static const uint8_t CHIP_SELECT_PIN = 5;

BMI270 imu = BMI270(SPI, CHIP_SELECT_PIN);

static bool gotInterrupt;

void setup() 
{
    Serial.begin(115200);

    SPI.begin();

    imu.begin();
}

static void report(void)
{
    int16_t ax=0, ay=0, az=0;
    imu.readAccel(ax, ay, az);

    int16_t gx=0, gy=0, gz=0;
    imu.readGyro(gx, gy, gz);

    Serial.print("ax=");
    Serial.print(ax);
    Serial.print("  ay=");
    Serial.print(ay);
    Serial.print("  az=");
    Serial.print(az);

    Serial.print("  gx=");
    Serial.print(gx);
    Serial.print("  gy=");
    Serial.print(gy);
    Serial.print("  gz=");
    Serial.print(gz);

    Serial.println();
}

void loop() 
{
    if (INTERRUPT_PIN == 0 || gotInterrupt) {

        gotInterrupt = false;

        report();

    }
}

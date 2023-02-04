/* BMI sensor class supporting I^2C and SPI buses
 *
 * Copyright (c) 2023 Simon D. Levy
 *
 * Adapted from 
 *   https://forum.arduino.cc/t/need-help-for-getting-mag-data-of-bmi270-imu/1000323
 *
 * MIT License
 *
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "api/bmi270.h"

class BMI270 {

    public:

        BMI270(TwoWire & wire)
            : BMI270()
        {
            (void)wire;
        }

        void begin(void)
        {
        }

        void readSensor(void)
        {
        }

        int16_t getAccelX(void)
        {
            return 0;
        }

        int16_t getAccelY(void)
        {
            return 0;
        }

        int16_t getAccelZ(void)
        {
            return 0;
        }

        int16_t getGyroX(void)
        {
            return 0;
        }

        int16_t getGyroY(void)
        {
            return 0;
        }

        int16_t getGyroZ(void)
        {
            return 0;
        }

    private:

        BMI270(void)
        {
        }

}; // class BMI270

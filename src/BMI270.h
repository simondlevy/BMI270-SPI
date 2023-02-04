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
#include <SPI.h>

#include "api/bmi270.h"

class BMI270 {

    private:

        static const uint8_t CS_PIN  = 5;

    public:

        BMI270(const uint8_t csPin)
        {
            (void)csPin;

            config[0].type = BMI2_ACCEL;
            config[1].type = BMI2_GYRO;

            config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
            config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
            config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

            /* The user can change the following configuration parameter according to
             * their required Output data Rate. By default ODR is set as 200Hz for
             * gyro */
            config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
            /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps */
            config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
            config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
            config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
            config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

            m_bmi2.intf = BMI2_SPI_INTF;
            m_bmi2.read = BMI270::read_spi;
            m_bmi2.write = BMI270::write_spi;
            m_bmi2.read_write_len = 32;
            m_bmi2.delay_us = delay_usec;

            /* Config file pointer should be assigned to NULL, so that default file
             * address is assigned in bmi270_init */
            m_bmi2.config_file_ptr = NULL;

            data_int_cfg.pin_type = BMI2_INT1;
            data_int_cfg.int_latch = BMI2_INT_NON_LATCH;
            data_int_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; 
            data_int_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
            data_int_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;     
            data_int_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE; 
        }

        void begin(void)
        {
            pinMode(CS_PIN, OUTPUT);
            digitalWrite(CS_PIN, HIGH);

            checkResult(bmi270_init(&m_bmi2), "bmi270_init");

            checkResult(
                    bmi2_set_sensor_config(config, 2, &m_bmi2), "bmi2_set_sensor_config");

            checkResult(
                    bmi2_sensor_enable(sens_list, 2, &m_bmi2), "bmi2_sensor_enable");

            checkResult(bmi2_set_int_pin_config(&data_int_cfg, &m_bmi2),
                    "bmi2_set_int_pin_config");

            checkResult(bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &m_bmi2),
                    "bmi2_map_data_int");
        }

        void readSensor(void)
        {
            bmi2_get_sensor_data(&sensor_data, &m_bmi2);
        }

        int16_t getAccelX(void)
        {
            return sensor_data.acc.x;
        }

        int16_t getAccelY(void)
        {
            return sensor_data.acc.y;
        }

        int16_t getAccelZ(void)
        {
            return sensor_data.acc.z;
        }

        int16_t getGyroX(void)
        {
            return sensor_data.gyr.x;
        }

        int16_t getGyroY(void)
        {
            return sensor_data.gyr.y;
        }

        int16_t getGyroZ(void)
        {
            return sensor_data.gyr.z;
        }

    private:

        struct bmi2_dev m_bmi2;

        struct bmi2_int_pin_config data_int_cfg;

        uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

        struct bmi2_sens_config config[3];

        struct bmi2_sens_data sensor_data;

        static void delay_usec(uint32_t period_us, void *intf_ptr)
        {
            delayMicroseconds(period_us);
        }

        static void checkResult(const int8_t rslt, const char * funname)
        {
            while (rslt) {
                Serial.print(funname);
                Serial.print(" failed with code ");
                Serial.println(rslt);
                delay(500);
            }
        }

        static int8_t read_spi(
                const uint8_t addr,
                uint8_t * data,
                const uint32_t count,
                void * intf_ptr)
        {
            (void)intf_ptr;

            digitalWrite(CS_PIN, LOW);

            SPI.transfer(0x80 | addr);

            for (auto k = 0; k < count; k++) {
                data[k] = SPI.transfer(0);
            }

            digitalWrite(CS_PIN, HIGH);

            return 0;
        }

        static int8_t write_spi(
                uint8_t addr,
                const uint8_t *data,
                uint32_t count,
                void *intf_ptr)
        {
            (void)(intf_ptr);

            digitalWrite(CS_PIN, LOW);

            SPI.transfer(addr);

            for (auto k = 0; k < count; k++) {
                SPI.transfer(data[k]);
            }

            digitalWrite(CS_PIN, HIGH);

            return 0;
        }

}; // class BMI270

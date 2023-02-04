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
#include <Wire.h>

#include "api/bmi270.h"

class BMI270 {

    public:

        BMI270(SPIClass & spi, const uint8_t csPin)
            : BMI270()
        {
            m_busData.spi = &spi;
            m_busData.csPin = csPin;

            m_bmi2.intf = BMI2_SPI_INTF;

            m_bmi2.read = spi_read;
            m_bmi2.write = spi_write;
        }

        BMI270(TwoWire & wire)
            : BMI270()
        {
            m_busData.wire = &wire;
            m_busData.addr = 0x68;
            m_busData.csPin = 0;

            m_bmi2.intf = BMI2_I2C_INTF;

            m_bmi2.read = i2c_read;
            m_bmi2.write = i2c_write;
        }

        void begin(void)
        {
            // I^2C
            if (m_busData.csPin == 0) {
            }

            // SPI
            else {
                pinMode(m_busData.csPin, OUTPUT);
                digitalWrite(m_busData.csPin, HIGH);
            }

            checkResult(bmi270_init(&m_bmi2), "bmi270_init");

            checkResult(
                    bmi2_set_sensor_config(m_config, 2, &m_bmi2), "bmi2_set_sensor_config");
        
            uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

            checkResult(
                    bmi2_sensor_enable(sens_list, 2, &m_bmi2), "bmi2_sensor_enable");

            checkResult(bmi2_set_int_pin_config(&m_dataIntCfg, &m_bmi2),
                    "bmi2_set_int_pin_config");

            checkResult(bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &m_bmi2),
                    "bmi2_map_data_int");
        }

        void readSensor(void)
        {
            bmi2_get_sensor_data(&m_sensorData, &m_bmi2);
        }

        int16_t getAccelX(void)
        {
            return m_sensorData.acc.x;
        }

        int16_t getAccelY(void)
        {
            return m_sensorData.acc.y;
        }

        int16_t getAccelZ(void)
        {
            return m_sensorData.acc.z;
        }

        int16_t getGyroX(void)
        {
            return m_sensorData.gyr.x;
        }

        int16_t getGyroY(void)
        {
            return m_sensorData.gyr.y;
        }

        int16_t getGyroZ(void)
        {
            return m_sensorData.gyr.z;
        }

    private:

        typedef struct {

            SPIClass * spi;
            uint8_t csPin;

            TwoWire * wire;
            uint8_t addr;

        } busData_t;

        struct bmi2_dev m_bmi2;

        struct bmi2_int_pin_config m_dataIntCfg;

        struct bmi2_sens_config m_config[2];

        struct bmi2_sens_data m_sensorData;

        busData_t m_busData;

        BMI270(void)
        {
            m_config[0].type = BMI2_ACCEL;
            m_config[1].type = BMI2_GYRO;

            m_config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;

            // Gravity range of the sensor (+/- 2G, 4G, 8G, 16G)
            m_config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
            m_config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
            m_config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

            // The user can change the following configuration parameter
            // according to their required Output data Rate. By default ODR is
            // set as 200Hz for gyro 
            m_config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;

            // Gyroscope Angular Rate Measurement Range.By default the range is 2000dps
            m_config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
            m_config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
            m_config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
            m_config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

            m_bmi2.intf_ptr = &m_busData;
            m_bmi2.read_write_len = 32;
            m_bmi2.delay_us = delay_usec;

            // Config file pointer should be assigned to NULL, so that default
            // file address is assigned in bmi270_init
            m_bmi2.config_file_ptr = NULL;

            m_dataIntCfg.pin_type = BMI2_INT1;
            m_dataIntCfg.int_latch = BMI2_INT_NON_LATCH;
            m_dataIntCfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; 
            m_dataIntCfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
            m_dataIntCfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;     
            m_dataIntCfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE; 
        }

 
        static void delay_usec(uint32_t period_us, void * intf_ptr)
        {
            (void)intf_ptr;

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

        static int8_t spi_read(
                const uint8_t reg,
                uint8_t * data,
                const uint32_t count,
                void * intf_ptr)
        {
            busData_t * busData = (busData_t *)intf_ptr;

            SPIClass * spi = busData->spi;
            const uint8_t csPin = busData->csPin;

            digitalWrite(csPin, LOW);

            spi->transfer(0x80 | reg);

            for (auto k = 0; k < count; k++) {
                data[k] = spi->transfer(0);
            }

            digitalWrite(csPin, HIGH);

            return 0;
        }

        static int8_t spi_write(
                uint8_t reg,
                const uint8_t *data,
                uint32_t count,
                void *intf_ptr)
        {
            busData_t * busData = (busData_t *)intf_ptr;

            SPIClass * spi = busData->spi;
            const uint8_t csPin = busData->csPin;

            digitalWrite(csPin, LOW);

            spi->transfer(reg);

            for (auto k = 0; k < count; k++) {
                spi->transfer(data[k]);
            }

            digitalWrite(csPin, HIGH);

            return 0;
        }

        static int8_t i2c_read(
                const uint8_t reg,
                uint8_t * data,
                const uint32_t count,
                void * intf_ptr)
        {
            busData_t * busData = (busData_t *)intf_ptr;

            TwoWire * wire = busData->wire;

            (void)busData;

            return 0;
        }

        static int8_t i2c_write(
                uint8_t reg,
                const uint8_t *data,
                uint32_t count,
                void *intf_ptr)
        {
            busData_t * busData = (busData_t *)intf_ptr;

            (void)busData;

            return 0;
        }

}; // class BMI270

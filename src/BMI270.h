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

#include <SPI.h>

#include "api/bmi270.h"

class BMI270 {

    public:

        typedef enum {

            GYRO_ODR_25_HZ = 6,
            GYRO_ODR_50_HZ,
            GYRO_ODR_100_HZ,
            GYRO_ODR_200_HZ,
            GYRO_ODR_400_HZ,
            GYRO_ODR_800_HZ,
            GYRO_ODR_1600_HZ,
            GYRO_ODR_3200_HZ

        } gyroOdr_e;

        typedef enum {

            GYRO_RANGE_2000_DPS,
            GYRO_RANGE_1000_DPS,
            GYRO_RANGE_500_DPS,
            GYRO_RANGE_250_DPS,
            GYRO_RANGE_125_DPS

        } gyroRange_e;
        
        typedef enum {

            ACCEL_ODR_0_78_HZ = 1,
            ACCEL_ODR_1_56_HZ,
            ACCEL_ODR_3_12_HZ,
            ACCEL_ODR_6_25_HZ,
            ACCEL_ODR_12_5_HZ,
            ACCEL_ODR_25_HZ,
            ACCEL_ODR_50_HZ,
            ACCEL_ODR_100_HZ,
            ACCEL_ODR_200_HZ,
            ACCEL_ODR_400_HZ,
            ACCEL_ODR_800_HZ,
            ACCEL_ODR_1600_HZ

        } accelOdr_e;

        typedef enum {

            ACCEL_RANGE_2_G,
            ACCEL_RANGE_4_G,
            ACCEL_RANGE_8_G,
            ACCEL_RANGE_16_G

        } accelRange_e;

        BMI270(
                const uint8_t csPin,
                const accelRange_e accelRange = ACCEL_RANGE_2_G,
                const accelOdr_e accelOdr = ACCEL_ODR_100_HZ,
                const gyroRange_e gyroRange = GYRO_RANGE_2000_DPS,
                const gyroOdr_e gyroOdr = GYRO_ODR_100_HZ)
            : BMI270()
        {
            m_busData.spi = &SPI;
            m_busData.csPin = csPin;

            m_bmi2.intf = BMI2_SPI_INTF;

            m_bmi2.read = spi_read;
            m_bmi2.write = spi_write;
        }

        BMI270(
                SPIClass & spi,
                const uint8_t csPin,
                const accelRange_e accelRange = ACCEL_RANGE_2_G,
                const accelOdr_e accelOdr = ACCEL_ODR_100_HZ,
                const gyroRange_e gyroRange = GYRO_RANGE_2000_DPS,
                const gyroOdr_e gyroOdr = GYRO_ODR_100_HZ)
            : BMI270()
        {
            m_busData.spi = &spi;
            m_busData.csPin = csPin;

            m_bmi2.intf = BMI2_SPI_INTF;

            m_bmi2.read = spi_read;
            m_bmi2.write = spi_write;
        }

        void begin(void)
        {
            struct bmi2_int_pin_config interruptConfig;

            interruptConfig.pin_type = BMI2_INT1;
            interruptConfig.int_latch = BMI2_INT_NON_LATCH;
            interruptConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; 
            interruptConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
            interruptConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;     
            interruptConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE; 

            pinMode(m_busData.csPin, OUTPUT);
            digitalWrite(m_busData.csPin, HIGH);

            checkResult(bmi270_init(&m_bmi2), "bmi270_init");

            checkResult(
                    bmi2_set_sensor_config(m_config, 2, &m_bmi2),
                    "bmi2_set_sensor_config");

            uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

            checkResult(
                    bmi2_sensor_enable(sens_list, 2, &m_bmi2), "bmi2_sensor_enable");

            checkResult(bmi2_set_int_pin_config(&interruptConfig, &m_bmi2),
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

        } busData_t;

        struct bmi2_dev m_bmi2;

        struct bmi2_sens_config m_config[2];

        struct bmi2_sens_data m_sensorData;

        busData_t m_busData;

        BMI270(
                const accelRange_e accelRange = ACCEL_RANGE_2_G,
                const accelOdr_e accelOdr = ACCEL_ODR_100_HZ,
                const gyroRange_e gyroRange = GYRO_RANGE_2000_DPS,
                const gyroOdr_e gyroOdr = GYRO_ODR_100_HZ)
        {
            m_config[0].type = BMI2_ACCEL;
            m_config[0].cfg.acc.odr = accelOdr;
            m_config[0].cfg.acc.range = accelRange;
            m_config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
            m_config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

            m_config[1].type = BMI2_GYRO;
            m_config[1].cfg.gyr.odr = gyroOdr;
            m_config[1].cfg.gyr.range = gyroRange;
            m_config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
            m_config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
            m_config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

            m_bmi2.intf_ptr = &m_busData;
            m_bmi2.read_write_len = 32;
            m_bmi2.delay_us = delay_usec;

            // Config file pointer should be assigned to NULL, so that default
            // file address is assigned in bmi270_init
            m_bmi2.config_file_ptr = NULL;

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

}; // class BMI270

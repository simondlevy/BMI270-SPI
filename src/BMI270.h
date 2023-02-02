/*
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

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "api/bmi270.h"

class BMI270 {

    public:

        BMI270(TwoWire& wire)
        {
            _wire = &wire;
        }

        void begin() 
        {
            _bmi2.chip_id = BMI2_I2C_PRIM_ADDR;
            _bmi2.read = bmi2_i2c_read;
            _bmi2.write = bmi2_i2c_write;
            _bmi2.delay_us = bmi2_delay_us;
            _bmi2.intf = BMI2_I2C_INTF;
            _bmi2.intf_ptr = &_accel_gyro_dev_info;
            _bmi2.read_write_len = 30; // Limitation of the Wire library
            _bmi2.config_file_ptr = NULL; // Use the default BMI270 config file

            _accel_gyro_dev_info._wire = _wire;
            _accel_gyro_dev_info.dev_addr = _bmi2.chip_id;

            int8_t rslt = bmi270_init(&_bmi2);
            checkResult(rslt);

            rslt = configure_sensor(&_bmi2);
            checkResult(rslt);
        }

        void readAcceleration(int16_t& x, int16_t& y, int16_t& z) 
        {
            struct bmi2_sens_data sensor_data;
            bmi2_get_sensor_data(&sensor_data, &_bmi2);
            x = sensor_data.acc.x;
            y = sensor_data.acc.y;
            z = sensor_data.acc.z;
        }

        void readGyroscope(int16_t& x, int16_t& y, int16_t& z) 
        {
            struct bmi2_sens_data sensor_data;
            bmi2_get_sensor_data(&sensor_data, &_bmi2);
            x = sensor_data.gyr.x;
            y = sensor_data.gyr.y;
            z = sensor_data.gyr.z;
        }

        bool accelerationAvailable() 
        {
            return dataAvailable(BMI2_ACC_DRDY_INT_MASK);
        }

        bool gyroscopeAvailable() 
        {
            return dataAvailable(BMI2_GYR_DRDY_INT_MASK);
        }

        // XXX Do we really need this?  It appears to duplicate configure_sensor()
        int8_t enableInterrupts(void)
        {
            int8_t rslt = BMI2_OK;
            uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

            struct bmi2_int_pin_config int_pin_cfg;
            int_pin_cfg.pin_type = BMI2_INT1;
            int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
            int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
            int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
            int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
            int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

            struct bmi2_sens_config sens_cfg[2];
            sens_cfg[0].type = BMI2_ACCEL;
            sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
            sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
            sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
            sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
            sens_cfg[1].type = BMI2_GYRO;
            sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
            sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
            sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
            sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
            sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

            rslt = bmi2_set_int_pin_config(&int_pin_cfg, &_bmi2);
            if (rslt != BMI2_OK)
                return rslt;

            rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &_bmi2);
            if (rslt != BMI2_OK)
                return rslt;

            rslt = bmi2_set_sensor_config(sens_cfg, 2, &_bmi2);
            if (rslt != BMI2_OK)
                return rslt;

            rslt = bmi2_sensor_enable(sens_list, 2, &_bmi2);
            if (rslt != BMI2_OK)
                return rslt;

            return rslt;
         }

    private:

        typedef struct {
            TwoWire* _wire;
            uint8_t dev_addr;
        } i2c_info_t;

        TwoWire* _wire;

        i2c_info_t _accel_gyro_dev_info;

        struct bmi2_dev _bmi2;

        uint16_t _int_status;

        bool dataAvailable(const uint16_t mask) 
        {
            uint16_t status = 0;
            bmi2_get_int_status(&status, &_bmi2);
            uint16_t ret = ((status | _int_status) & mask);
            _int_status = status;
            _int_status &= ~mask;
            return ret != 0;
        }

        int8_t configure_sensor(struct bmi2_dev *dev)
        {
            int8_t rslt;
            uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

            struct bmi2_int_pin_config int_pin_cfg;
            int_pin_cfg.pin_type = BMI2_INT1;
            int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
            int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
            int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
            int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
            int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

            struct bmi2_sens_config sens_cfg[2];
            sens_cfg[0].type = BMI2_ACCEL;
            sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
            sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
            sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
            sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
            sens_cfg[1].type = BMI2_GYRO;
            sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
            sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
            sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
            sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
            sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

            rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
            if (rslt != BMI2_OK)
                return rslt;

            rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
            if (rslt != BMI2_OK)
                return rslt;

            rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
            if (rslt != BMI2_OK)
                return rslt;

            rslt = bmi2_sensor_enable(sens_list, 2, dev);
            if (rslt != BMI2_OK)
                return rslt;

            return rslt;
        }


        static int8_t bmi2_i2c_read(
                uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
        {
            if ((reg_data == NULL) || (len == 0) || (len > 32)) {
                return -1;
            }
            uint8_t bytes_received;

            i2c_info_t* dev_info = (i2c_info_t*)intf_ptr;
            uint8_t dev_id = dev_info->dev_addr;

            dev_info->_wire->beginTransmission(dev_id);
            dev_info->_wire->write(reg_addr);
            if (dev_info->_wire->endTransmission() == 0) {
                bytes_received = dev_info->_wire->requestFrom(dev_id, len);
                // Optionally, throw an error if bytes_received != len
                for (uint16_t i = 0; i < bytes_received; i++)
                {
                    reg_data[i] = dev_info->_wire->read();
                }
            } else {
                return -1;
            }

            return 0;
        }

        static int8_t bmi2_i2c_write(
                uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
        {
            if ((reg_data == NULL) || (len == 0) || (len > 32)) {
                return -1;
            }

            i2c_info_t* dev_info = (i2c_info_t*)intf_ptr;
            uint8_t dev_id = dev_info->dev_addr;
            dev_info->_wire->beginTransmission(dev_id);
            dev_info->_wire->write(reg_addr);
            for (uint16_t i = 0; i < len; i++)
            {
                dev_info->_wire->write(reg_data[i]);
            }
            if (dev_info->_wire->endTransmission() != 0) {
                return -1;
            }

            return 0;
        }

        static void bmi2_delay_us(uint32_t period, void *intf_ptr)
        {
            delayMicroseconds(period);
        }

        static void panic_led_trap(void)
        {
            pinMode(LED_BUILTIN, OUTPUT);
            while (1)
            {
                digitalWrite(LED_BUILTIN, LOW);
                delay(100);
                digitalWrite(LED_BUILTIN, HIGH);
                delay(100);
            }
        }

        static void reportForever(const int8_t rslt, const char * msg)
        {
            while (true) {
                Serial.print("Error ");
                Serial.print(rslt);
                Serial.print(": ");
                Serial.println(msg);
                delay(500);
            }
        }

        static void checkResult(int8_t rslt)
        {
            switch (rslt) {

                case BMI2_OK: 
                    return; 

                case BMI2_E_NULL_PTR:
                    reportForever(rslt, "Null pointer");

                case BMI2_E_COM_FAIL:
                    reportForever(rslt, "Communication failure");

                case BMI2_E_DEV_NOT_FOUND:
                    reportForever(rslt, "Device not found");

                case BMI2_E_OUT_OF_RANGE:
                    reportForever(rslt, "Out of range");

                case BMI2_E_ACC_INVALID_CFG:
                    reportForever(rslt, "Invalid accel configuration");

                case BMI2_E_GYRO_INVALID_CFG:
                    reportForever(rslt, "Invalid gyro configuration");

                case BMI2_E_ACC_GYR_INVALID_CFG:
                    reportForever(rslt, "Invalid accel/gyro configuration");

                case BMI2_E_INVALID_SENSOR:
                    reportForever(rslt, "Invalid sensor");

                case BMI2_E_CONFIG_LOAD:
                    reportForever(rslt, "Configuration loading error");

                case BMI2_E_INVALID_PAGE:
                    reportForever(rslt, "Invalid page ");

                case BMI2_E_INVALID_FEAT_BIT:
                    reportForever(rslt, "Invalid feature bit");

                case BMI2_E_INVALID_INT_PIN:
                    reportForever(rslt, "Invalid interrupt pin");

                case BMI2_E_SET_APS_FAIL:
                    reportForever(rslt, "Setting advanced power mode failed");

                case BMI2_E_AUX_INVALID_CFG:
                    reportForever(rslt, "Invalid auxilliary configuration");

                case BMI2_E_AUX_BUSY:
                    reportForever(rslt, "Auxilliary busy");

                case BMI2_E_SELF_TEST_FAIL:
                    reportForever(rslt, "Self test failed");

                case BMI2_E_REMAP_ERROR:
                    reportForever(rslt, "Remapping error");

                case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
                    reportForever(rslt, "Gyro user gain update failed");

                case BMI2_E_SELF_TEST_NOT_DONE:
                    reportForever(rslt, "Self test not done");

                case BMI2_E_INVALID_INPUT:
                    reportForever(rslt, "Invalid input");

                case BMI2_E_INVALID_STATUS:
                    reportForever(rslt, "Invalid status");

                case BMI2_E_CRT_ERROR:
                    reportForever(rslt, "CRT error");

                case BMI2_E_ST_ALREADY_RUNNING:
                    reportForever(rslt, "Self test already running");

                case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
                    reportForever(rslt, "CRT ready for DL fail abort");

                case BMI2_E_DL_ERROR:
                    reportForever(rslt, "DL error");

                case BMI2_E_PRECON_ERROR:
                    reportForever(rslt, "PRECON error");

                case BMI2_E_ABORT_ERROR:
                    reportForever(rslt, "Abort error");

                case BMI2_E_GYRO_SELF_TEST_ERROR:
                    reportForever(rslt, "Gyro self test error");

                case BMI2_E_GYRO_SELF_TEST_TIMEOUT:
                    reportForever(rslt, "Gyro self test timeout");

                case BMI2_E_WRITE_CYCLE_ONGOING:
                    reportForever(rslt, "Write cycle ongoing");

                case BMI2_E_WRITE_CYCLE_TIMEOUT:
                    reportForever(rslt, "Write cycle timeout");

                case BMI2_E_ST_NOT_RUNING:
                    reportForever(rslt, "Self test not running");

                case BMI2_E_DATA_RDY_INT_FAILED:
                    reportForever(rslt, "Data ready interrupt failed");

                case BMI2_E_INVALID_FOC_POSITION:
                    reportForever(rslt, "Invalid FOC position");

                default:
                    reportForever(rslt, "Unknown error code");
            }
        }
};

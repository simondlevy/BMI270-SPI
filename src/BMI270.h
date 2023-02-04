// Adapted from https://forum.arduino.cc/t/need-help-for-getting-mag-data-of-bmi270-imu/1000323

#include "api/bmi270.h"

class BMI270 {

    private:
    
        static const uint8_t CS_PIN  = 5;

        static void delay_usec(uint32_t period_us, void *intf_ptr)
        {
            delayMicroseconds(period_us);
        }

    public:

        uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
        struct bmi2_sens_config config[3];
        struct bmi2_dev bmi2;

        BMI270(void)
        {
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

            bmi2.intf = BMI2_SPI_INTF;
            bmi2.read = BMI270::read_spi;
            bmi2.write = BMI270::write_spi;
            bmi2.read_write_len = 32;
            bmi2.delay_us = delay_usec;

            /* Config file pointer should be assigned to NULL, so that default file
             * address is assigned in bmi270_init */
            bmi2.config_file_ptr = NULL;
        }

        void begin(void)
        {
            pinMode(CS_PIN, OUTPUT);
            digitalWrite(CS_PIN, HIGH);

            checkResult(bmi270_init(&bmi2), "bmi270_init");

            checkResult(
                    bmi2_set_sensor_config(config, 2, &bmi2), "bmi2_set_sensor_config");

            checkResult(
                    bmi2_sensor_enable(sens_list, 2, &bmi2), "bmi2_sensor_enable");
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
                uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
        {
            uint32_t cnt;
            int8_t rev = 0;
            (void)(intf_ptr);
            reg_addr = 0x80 | reg_addr;
            digitalWrite(CS_PIN, LOW);
            SPI.transfer(reg_addr);
            for (cnt = 0; cnt < length; cnt++)
            {
                *(reg_data + cnt) = SPI.transfer(0x00);
            }
            digitalWrite(CS_PIN, HIGH);
            return rev;
        }

        static int8_t write_spi(
                uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
        {
            uint32_t cnt;
            int8_t rev = 0;
            (void)(intf_ptr);
            digitalWrite(CS_PIN, LOW);
            SPI.transfer(reg_addr);
            for (cnt = 0; cnt < length; cnt++)
            {
                SPI.transfer(*(reg_data + cnt));
            }
            digitalWrite(CS_PIN, HIGH);
            return rev;
        }

}; // class BMI270

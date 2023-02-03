// Adapted from https://forum.arduino.cc/t/need-help-for-getting-mag-data-of-bmi270-imu/1000323

#include <Arduino.h>
#include <SPI.h>

#include <BMI270.h>

static const uint8_t CS_PIN  = 5;
static const uint8_t INT_PIN = 22;

static uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};

/* Structure to define BMI2 sensor configurations */
static struct bmi2_dev bmi2;

// Sensor initialization configuration.
/* Structure to define the type of the sensor and its configurations */
static struct bmi2_sens_config config[3];

static void delay_usec(uint32_t period_us, void *intf_ptr)
{
    delayMicroseconds(period_us);
}

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static void BMI270_Init()
{
    BMI270::checkResult(bmi270_init(&bmi2), "bmi270_init");

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

    /* Set the accel configurations */
    BMI270::checkResult(bmi2_set_sensor_config(config, 2, &bmi2), "bmi2_set_sensor_config");

}

void setup() {

    // Configure type of feature 
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Variable to define result */
    pinMode(CS_PIN, OUTPUT);

    //pinMode(interruptPin, INPUT_PULLUP);
    digitalWrite(CS_PIN, HIGH);

    Serial.begin(115200);

    SPI.begin();

    bmi2.intf = BMI2_SPI_INTF;
    bmi2.read = BMI270::read_spi;
    bmi2.write = BMI270::write_spi;
    bmi2.read_write_len = 32;
    bmi2.delay_us = delay_usec;

    /* Config file pointer should be assigned to NULL, so that default file
     * address is assigned in bmi270_init */
    bmi2.config_file_ptr = NULL;

    BMI270_Init();
    
    BMI270::checkResult(bmi2_sensor_enable(sens_list, 2, &bmi2), "bmi2_sensor_enable");

    // Interrupt PINs configuration
    struct bmi2_int_pin_config data_int_cfg;
    data_int_cfg.pin_type = BMI2_INT1;
    data_int_cfg.int_latch = BMI2_INT_NON_LATCH;
    data_int_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; 
    data_int_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    data_int_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;     
    data_int_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE; 

    BMI270::checkResult(bmi2_set_int_pin_config(&data_int_cfg, &bmi2), "bmi2_set_int_pin_config");
    
    BMI270::checkResult(bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi2), "bmi2_map_data_int");

    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop() 
{
    if (gotInterrupt) {

        gotInterrupt = false;

        struct bmi2_sens_data sensor_data;

        bmi2_get_sensor_data(&sensor_data, &bmi2);

        auto ax = sensor_data.acc.x;
        auto ay = sensor_data.acc.y;
        auto az = sensor_data.acc.z;

        auto gx = sensor_data.gyr.x;
        auto gy = sensor_data.gyr.y;
        auto gz = sensor_data.gyr.z;

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

    delay(10);
}

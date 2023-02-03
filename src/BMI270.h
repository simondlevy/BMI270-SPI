// Adapted from https://forum.arduino.cc/t/need-help-for-getting-mag-data-of-bmi270-imu/1000323

#include "api/bmi270.h"

class BMI270 {

    private:
    
        static const uint8_t CS_PIN  = 5;

    public:

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

#if 0

#define CONFIG_INT_WDT_CHECK_CPU0 0

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_SPI UINT8_C(1)

static const uint8_t CS_PIN  = 5;
static const uint8_t INT_PIN = 22;

static uint8_t spi_bus;

static uint8_t sens_int = BMI2_DRDY_INT;

/* List the sensors which are required to enable */
//static uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
static uint8_t sens_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};

/* Structure to define BMI2 sensor configurations */
static struct bmi2_dev bmi2;

// Sensor initialization configuration.
/* Structure to define the type of the sensor and its configurations */
static struct bmi2_sens_config config[3];

static int8_t BMI270_read_spi(
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

static int8_t BMI270_write_spi(
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

void bmi2xy_hal_delay_usec(uint32_t period_us, void *intf_ptr)
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

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static void BMI270_Init()
{
    checkResult(bmi270_init(&bmi2), "bmi270_init");

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
    checkResult(bmi2_set_sensor_config(config, 2, &bmi2), "bmi2_set_sensor_config");

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
    spi_bus = CS_PIN;

    SPI.begin();

    bmi2.intf_ptr = &spi_bus;
    bmi2.intf = BMI2_SPI_INTF;
    bmi2.read = BMI270_read_spi;
    bmi2.write = BMI270_write_spi;
    bmi2.read_write_len = 32;
    bmi2.delay_us = bmi2xy_hal_delay_usec;

    /* Config file pointer should be assigned to NULL, so that default file
     * address is assigned in bmi270_init */
    bmi2.config_file_ptr = NULL;

    BMI270_Init();
    
    checkResult(bmi2_sensor_enable(sens_list, 2, &bmi2), "bmi2_sensor_enable");

    // Interrupt PINs configuration
    struct bmi2_int_pin_config data_int_cfg;
    data_int_cfg.pin_type = BMI2_INT1;
    data_int_cfg.int_latch = BMI2_INT_NON_LATCH;
    data_int_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; 
    data_int_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    data_int_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;     
    data_int_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE; 

    checkResult(bmi2_set_int_pin_config(&data_int_cfg, &bmi2), "bmi2_set_int_pin_config");
    
    checkResult(bmi2_map_data_int(sens_int, BMI2_INT1, &bmi2), "bmi2_map_data_int");

    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}
#endif

// Adapted from https://forum.arduino.cc/t/need-help-for-getting-mag-data-of-bmi270-imu/1000323

#include <Arduino.h>
#include <SPI.h>

#include <BMI270.h>

static const uint8_t INT_PIN = 22;

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static BMI270 imu;

void setup() {

    Serial.begin(115200);

    SPI.begin();

    imu.begin();

    attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop() 
{
    if (gotInterrupt) {

        gotInterrupt = false;

        struct bmi2_sens_data sensor_data;

        bmi2_get_sensor_data(&sensor_data, &imu.bmi2);

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

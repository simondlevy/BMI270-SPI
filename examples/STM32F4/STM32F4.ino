/* Read BMI270 sensor on STM32F4 MCU via SPI bus
 *
 * Copyright (c) 2023 Simon D. Levy
 *
 * MIT License
 *
 */

#include <SPI.h>

#include "BMI270.h"

static const uint8_t MOSI_PIN = PA7;
static const uint8_t MISO_PIN = PA6;
static const uint8_t SCLK_PIN = PA5;

static const uint8_t LED_PIN = PC13;
static const uint8_t CS_PIN  = PA4;


// Set to 0 for polling
static const uint8_t INT_PIN = 0; // PA1;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);

static bool gotInterrupt;

static void handleInterrupt(void)
{
    gotInterrupt = true;
}

static BMI270 imu = BMI270(spi, CS_PIN);

static void blinkLed(void)
{
    auto msec = millis();
    static uint32_t prev;

    if (msec - prev > 500) {
        static bool on;
        prev = msec;
        on = !on;
        digitalWrite(LED_PIN, on);
    }
}

static void reboot(void)
{
    __enable_irq();
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0;
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    const uint32_t p = (*((uint32_t *) 0x1FFF0000));
    __set_MSP( p );

    void (*SysMemBootJump)(void);
    SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
    SysMemBootJump();

    NVIC_SystemReset();
}

void setup(void)
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    spi.begin();

    imu.begin();

    if (INT_PIN > 0) {
        attachInterrupt(INT_PIN, handleInterrupt, RISING);
    }
}

void loop(void) 
{
    blinkLed();

    if (Serial.available() && Serial.read() == 'R') {
        reboot();
    }

    if (INT_PIN == 0 || gotInterrupt) {

        gotInterrupt = false;

        imu.readSensor();

        Serial.print("ax=");
        Serial.print(imu.getRawAccelX());
        Serial.print("  ay=");
        Serial.print(imu.getRawAccelY());
        Serial.print("  az=");
        Serial.print(imu.getRawAccelZ());

        Serial.print("  gx=");
        Serial.print(imu.getRawGyroX());
        Serial.print("  gy=");
        Serial.print(imu.getRawGyroY());
        Serial.print("  gz=");
        Serial.print(imu.getRawGyroZ());

        Serial.println();
    }
}

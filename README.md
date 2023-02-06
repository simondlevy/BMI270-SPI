
## About

This is a simple, header-only Arduino library for the Bosch BMI270 inertial
measurement units using the SPI bus.  

## Use cases

I have tested this library on the following hardware:

* [MikroE 6DOF IMU 12 Click](https://www.mikroe.com/6dof-imu-12-click)
with [TinyPICO](https://www.tinypico.com/) ESP32 development board.


* [GEPRC-12A-F4 Flight Controller 12A ESC](https://geprc.com/product/gep-12a-f4-flight-controller-12a-esc/)

Note that for the MirkoE board you have to remove the zero-Ohm resistors on the
MikroE's I<sup>2</sup> pads and solder a bridge across the adjacent SPI pads.

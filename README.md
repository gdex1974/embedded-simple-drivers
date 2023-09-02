# SimpleDrivers

## The main concept

There are many device drivers useful in embedded development for the devices like STM32 or Arduino. But most of them are all-in-one solutions responsible for everything from GPIO pin initialization to formatted printing written on pure C with all these macros and global structures.
This library is an opposite. It contains C++ classes which do its job in a simple and efficient way delegating transport and initialization to other components from general-support-library and client code. 

## SPS30

This is a set of classes for interaction with Senserion SPS30 air quality sensor using I2C or UART interfaces.

## BME280

This is a driver for Bosch BME280/BMP280 temperature-pressure(-humidity) combined sensor.

## eInk

This is a driver for Waveshare eInk displays.
The folder contains a hardware interface abstraction for the SPI e-Ink displays and a specific driver for 3,7" one.

## License

This code is licenced under BSD 3-clause license.

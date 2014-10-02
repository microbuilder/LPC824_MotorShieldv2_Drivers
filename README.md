# LPC824 Motor Shield v2 Drivers

This repository contains drivers for the Motor Shield from Adafruit industries, available here:

- Product Page: https://www.adafruit.com/product/1438
- Learning Guide: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino

The drivers are meant to be used with the LPC82x Xpresso v2 board from NXP in the free LPCXpresso IDE.

# Supported Motor Types

At present, **DC motors** (standard toy DC motors, DC vibration motors, etc.) and **Servo motors** are supported via helper functions.  DC motors are controlled by the PCA9685 PWM driver, which is driver via I2C, and servo motors are driven from a PWM signal generated via the SCTimer peripheral on the LPC824.  See the documention in the root folder for further information.

The following motors were tested during developent:

- Small DC Motor: https://www.adafruit.com/products/711
- Vibrating Mini Motor Disk: https://www.adafruit.com/products/1201
- Continuous Rotation Servo: https://www.adafruit.com/products/1201
- Standard Servo: https://www.adafruit.com/products/155
- Mini Pan-Tilt Kit: https://www.adafruit.com/products/1967

# SW Requirements

The latest version of LPCXpresso is required to build this (7.4.0 or higher): http://www.lpcware.com/content/forum/lpcxpresso-latest-release

This driver is based on the LPCOpen platform (v2.14): http://www.lpcware.com/content/nxpfile/lpcopen-software-development-platform-lpc8xx-packages

The following libraries from the LPCOpen package will also need to be open in your workspace in LPCXpresso:

- lpc_chip_82x
- lpc_board_nxp_lpcxpresso_824

# HW Requirements

The motor shield uses VIN as a power supply, but the VIN pin is not connected on the LPC82x Xpresso v2 board, meaning that you will need to solder a wire between **5V** and **VIN** on the shield to use this board.  See the .PDF documentation included in the root folder of this repository for an example of how to connected 5V and VIN.
/*
 * pca9685.h
 *
 *  Created on: 31 août 2014
 *      Author: ktown
 */

#ifndef PCA9685_H_
#define PCA9685_H_

#include <stdint.h>
#include <stdbool.h>

#define PCA9685_I2CADDR_BASE   (0x60 << 1) /* 1.1.[A4].[A3].[A2].[A1].[A0] */

/* PCA9685 registers */
typedef enum
{
  PCA9685_REG_MODE1          = 0x00,
  PCA9685_REG_SUBADR1        = 0x02,
  PCA9685_REG_SUBADR2        = 0x03,
  PCA9685_REG_SUBADR3        = 0x04,
  PCA9685_REG_LED0_ON_L      = 0x06,
  PCA9685_REG_LED0_ON_H      = 0x07,
  PCA9685_REG_LED0_OFF_L     = 0x08,
  PCA9685_REG_LED0_OFF_H     = 0x09,
  PCA9685_REG_ALL_LED_ON_L   = 0xFA,
  PCA9685_REG_ALL_LED_ON_H   = 0xFB,
  PCA9685_REG_ALL_LED_OFF_L  = 0xFC,
  PCA9685_REG_ALL_LED_OFF_H  = 0xFD,
  PCA9685_REG_PRESCALE       = 0xFE
} pca9685Reg_t;

bool pca9685Init(uint8_t addr);
void pca9685SetPWM(uint8_t channel, uint16_t on, uint16_t off);
void pca9685SetPWMFreq(float freq);

#endif /* PCA9685_H_ */

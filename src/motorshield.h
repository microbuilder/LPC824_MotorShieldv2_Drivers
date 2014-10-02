/*
 * motorshield.h
 *
 *  Created on: 11 August 2014
 *      Author: ktown
 */

#ifndef MOTORSHIELD_H_
#define MOTORSHIELD_H_

#include <stdint.h>
#include <stdbool.h>
#include "pca9685.h"

/* Defines the direction that the motor is rotating */
typedef enum
{
  MS_DIR_FORWARD  = 1,
  MS_DIR_BACKWARD = 2,
  MS_DIR_RELEASE  = 3
} msDirection_t;

/* Placeholder object to store motor config data */
typedef struct
{
  bool          initialized;  /* True if the motor has been intialised */
  msDirection_t dir;          /* The direction the motor is currently rotating */
  uint8_t       pwm;          /* The PWM channel on the PCA9685 */
  uint8_t       in1;          /* The IN1 channel on the PCA9685 */
  uint8_t       in2;          /* The IN2 channel on the PCA9685 */
} msDCMotor_t;

bool          msInit       ( uint8_t addr );
void          msDCInit     ( uint8_t motorNum );
void          msDCRun      ( uint8_t motorNum, msDirection_t direction, uint8_t speed );
void          msServoInit  ( uint8_t motorNum );
uint32_t      msServoGetMinDutyCycle ( void );
uint32_t      msServoGetMaxDutyCycle ( void );
void          msServoSetDutyCycle ( uint8_t motorNum, uint32_t dutyCycle );


#endif /* MOTORSHIELD_H_ */

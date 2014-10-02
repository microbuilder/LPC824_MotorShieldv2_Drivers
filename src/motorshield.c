/*
 * motorshield.c
 *
 *  Created on: 11 August 2014
 *      Author: ktown
 */

#include <string.h>
#include <math.h>

#include "motorshield.h"
#include "board.h"

/* The maximum number of motors supported by v2 of the motor shield */
#define MS_MOTORCOUNT_DC       (4)                   /* Maximum number of DC motors */
#define MS_MOTORCOUNT_STEPPER  (2)                   /* Maximum number of stepper motors */
#define MS_MOTORCOUNT_SERVO    (2)                   /* Maximum number of servo motors */

/* These dividers set the min and max duty cycle (width) for the servo, and may */
/* need to be adjusted depending on the servo that you are using.  Assuming a   */
/* 50Hz base clock (20000us), the current min and max values are set to 544us   */
/* and 2400uS. */
#define MS_SERVOS_MINTICKS_DIV (36765)               /* Min ticks divider = 20000us/544us (50Hz base clock) = 36.765 */
#define MS_SERVOS_MAXTICKS_DIV (8333)                /* Max ticks divider = 20000us/2400us (50Hz base clock) = 8.333 */

static bool          m_ms_initialized = false;       /* Whether msInit has been called or not */
static msDCMotor_t   m_ms_dcmotor[MS_MOTORCOUNT_DC]; /* DC motor array */

/**************************************************************************/
/*!
    @brief  Initialises the motor shield, setting pins up as appropriate

    @param  addr
            The 7-bit I2C address for the PCA9685 PWM driver

    @return true if the motor shield was properly initialised, otherwise
            false (the most likely cause is an incorrect I2C address or
            bad solder joints)
*/
/**************************************************************************/
bool msInit(uint8_t addr)
{
  memset(&m_ms_dcmotor, 0, sizeof(msDCMotor_t) * MS_MOTORCOUNT_DC);

  /* Initialises the PCA9685 (uses I2C0) */
  if (!pca9685Init(addr)) return false;

  m_ms_initialized = true;

  return true;
}

/**************************************************************************/
/*!
    @brief  Initialises the specified DC motor (1..4 for DC motors)

    @param  motorNum
            The DC motor to initialise (1..4)
*/
/**************************************************************************/
void msDCInit(uint8_t motorNum)
{
  uint8_t pwm, in1, in2;
  pwm = in1 = in2 = 0;

  if ((motorNum > MS_MOTORCOUNT_DC) || (motorNum == 0) || (!m_ms_initialized))
  {
    return;
  }

  /* Set the appropriate PWM channels for the selected motor */
  /* These values assume the pinout for the REV 2.0 Motor Shield */
  /* from Adafruit Industries: */
  /* https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/resources */
  switch (motorNum)
  {
  case 1:
    pwm = 8;
    in2 = 9;
    in1 = 10;
    break;
  case 2:
    pwm = 13;
    in2 = 12;
    in1 = 11;
    break;
  case 3:
    pwm = 2;
    in2 = 3;
    in1 = 4;
    break;
  case 4:
    pwm = 7;
    in2 = 6;
    in1 = 5;
    break;
  }

  m_ms_dcmotor[motorNum-1].pwm = pwm;
  m_ms_dcmotor[motorNum-1].in1 = in1;
  m_ms_dcmotor[motorNum-1].in2 = in2;
  m_ms_dcmotor[motorNum-1].dir = MS_DIR_RELEASE;
  m_ms_dcmotor[motorNum-1].initialized = true;
}

/**************************************************************************/
/*!
    @brief  Updates the speed and direction of the specified DC motor

    @param  motorNum
            The DC motor to update (1..4)
    @param  direction
            The direction that the motor should move (see msDirection_t)
    @param  speed
            The speed that the motor should rotate, which is an 8-bit
            value (0..255)
*/
/**************************************************************************/
void msDCRun(uint8_t motorNum, msDirection_t direction, uint8_t speed)
{
  if ((motorNum > MS_MOTORCOUNT_DC) || (motorNum == 0) || (!m_ms_initialized))
  {
    return;
  }

  /* Simulate GPIO using PWM outputs */
  switch (direction)
  {
  case MS_DIR_FORWARD:
    /* Set the LOW pin first to avoid 'break' */
    pca9685SetPWM(m_ms_dcmotor[motorNum-1].in2, 0, 0);
    pca9685SetPWM(m_ms_dcmotor[motorNum-1].in1, 4096, 0);
    break;
  case MS_DIR_BACKWARD:
    /* Set the LOW pin first to avoid 'break' */
    pca9685SetPWM(m_ms_dcmotor[motorNum-1].in1, 0, 0);
    pca9685SetPWM(m_ms_dcmotor[motorNum-1].in2, 4096, 0);
    break;
  case MS_DIR_RELEASE:
    pca9685SetPWM(m_ms_dcmotor[motorNum-1].in1, 0, 0);
    pca9685SetPWM(m_ms_dcmotor[motorNum-1].in2, 0, 0);
    break;
  }

  /* Set the PWM output (speed) */
  pca9685SetPWM(m_ms_dcmotor[motorNum-1].pwm,
                speed == 255 ? 4096 : 0,
                speed == 255 ? 0    : speed*16);
}

/**************************************************************************/
/*!
    @brief  Initialises the specified servo motor (1..2)

    @param  motorNum
            The servo motor to initialise (1..2)
*/
/**************************************************************************/
void msServoInit(uint8_t motorNum)
{
  if ((motorNum > MS_MOTORCOUNT_SERVO) || (motorNum == 0) || (!m_ms_initialized))
  {
    return;
  }

  Chip_SCTPWM_Init(LPC_SCT);          /* Initialize the SCT as PWM */
  Chip_SCTPWM_SetRate(LPC_SCT, 50);   /* Set frequency in Hz (50Hz) */

#if defined(BOARD_NXP_LPCXPRESSO_824)
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  /* Initialise the PWM outputs for the specified motor */
  switch(motorNum)
  {
  case 1:
    /* D9 on the Arduino headers */
    Chip_SWM_MovablePinAssign(SWM_SCT_OUT0_O, 15);
    Chip_SCTPWM_SetOutPin(LPC_SCT, 1, 0);
    Chip_SCTPWM_SetDutyCycle(LPC_SCT, 1, 0); /* 0% duty cycle */
    //Chip_SCTPWM_SetDutyCycle(LPC_SCT, 1, Chip_SCTPWM_GetTicksPerCycle(LPC_SCT)/2); /* 50% duty cycle */
    break;
  case 2:
    /* Note: Pin 27 on the LPC824 LPCXpresso board has an RGB LED */
    /* SJ1 must be cut before this channel can be used */
    /* D10 on the Arduino headers */
    Chip_SWM_MovablePinAssign(SWM_SCT_OUT1_O, 27);
    Chip_SCTPWM_SetOutPin(LPC_SCT, 2, 1);
    Chip_SCTPWM_SetDutyCycle(LPC_SCT, 2, 0); /* 0% duty cycle */
    //Chip_SCTPWM_SetDutyCycle(LPC_SCT, 2, Chip_SCTPWM_GetTicksPerCycle(LPC_SCT)/4); /* 25* duty cycle */
    break;
  default:
    break;
  }
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
#else
  /* Configure your own I2C pin muxing here if needed */
  #error "Unsupported board definition"
#endif

  /* Start the PWM output */
  Chip_SCTPWM_Start(LPC_SCT);
}

/**************************************************************************/
/*!
    @brief  Returns the minimum number of ticks that can be used for the
            PWM duty cycle (based on a 50Hz clock)
*/
/**************************************************************************/
uint32_t msServoGetMinDutyCycle(void)
{
  /* Multiply base clock by 1000 to take into account the divider values */
  /* This shouldn't overflow at any clock speed the LPC8xx sries can */
  /* handle but to be safe with faster chips use uint64_t for ticks */
  uint64_t ticksPerCycle = Chip_SCTPWM_GetTicksPerCycle(LPC_SCT) * 1000;
  return (uint32_t)(ticksPerCycle / MS_SERVOS_MINTICKS_DIV);
}

/**************************************************************************/
/*!
    @brief  Returns the maximum number of ticks that can be used for the
            PWM duty cycle (based on a 50Hz clock)
*/
/**************************************************************************/
uint32_t msServoGetMaxDutyCycle(void)
{
  /* Multiply base clock by 1000 to take into account the divider values */
  /* This shouldn't overflow at any clock speed the LPC8xx sries can */
  /* handle but to be safe with faster chips use uint64_t for ticks */
  uint64_t ticksPerCycle = Chip_SCTPWM_GetTicksPerCycle(LPC_SCT) * 1000;
  return (uint32_t)(ticksPerCycle / MS_SERVOS_MAXTICKS_DIV);
}

/**************************************************************************/
/*!
    @brief  Adjusts the servo's duty cycle (which will control the speed or
            position of the servo motor depending on the motor type)

    @param  motorNum
            The servo motor to initialise (1..2)
    @param  dutyCycle
            The duty cycle (in ticks) for the PWM output, based on a 50Hz
            clock.
*/
/**************************************************************************/
void msServoSetDutyCycle(uint8_t motorNum, uint32_t dutyCycle)
{
  if ((motorNum > MS_MOTORCOUNT_SERVO) || (motorNum == 0) || (!m_ms_initialized))
  {
    return;
  }

  /* Min/Max range for the servo */
  if (dutyCycle < msServoGetMinDutyCycle())
  {
    dutyCycle = msServoGetMinDutyCycle();
  }

  if (dutyCycle > msServoGetMaxDutyCycle())
  {
    dutyCycle = msServoGetMaxDutyCycle();
  }

  /* Adjust tick counter based on duty cycle percent */
  Chip_SCTPWM_SetDutyCycle(LPC_SCT, motorNum, dutyCycle);
}

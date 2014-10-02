/*
===============================================================================
 Name        : LPCOpen_Motor_Shield_V2.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#include "stopwatch.h"
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "motorshield.h"

/* Systick tick rate */
#define TICKRATE_HZ (10)

void delay(void)
{
  uint32_t volatile i;
  for (i=0;i<0xFFFF;i++)
  {
    __asm volatile("nop");
  }
}

#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
void SysTick_Handler(void)
{
	Board_LED_Toggle(0);
}
#endif
#endif

int main(void)
{
#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
  SystemCoreClockUpdate();
  Board_Init();
  Board_LED_Set(0, false);
#endif
  SysTick_Config(SystemCoreClock / TICKRATE_HZ);
  NVIC_EnableIRQ(SysTick_IRQn);
#endif

  /* Initialize the motor shield using the default address (0x60) */
  if (!msInit(PCA9685_I2CADDR_BASE))
  {
    /* Something failed while initializing the motor shield ... check your solder joints? */
    while(1);
  }

  /* Select one of these options to test either servo or DC motors */
#if 0
  /* Servo example */
  /* Initialise Servo 1 */
  msServoInit(1);

  while(1)
  {
    uint32_t volatile dutyCycle;

    /* Cover the entire servo motion range */
    for(dutyCycle=msServoGetMinDutyCycle(); dutyCycle<msServoGetMaxDutyCycle(); dutyCycle++)
    {
      msServoSetDutyCycle(1, dutyCycle);
    }
    for(dutyCycle=msServoGetMaxDutyCycle(); dutyCycle>msServoGetMinDutyCycle(); dutyCycle--)
    {
      msServoSetDutyCycle(1, dutyCycle);
    }

    //__WFI();
  }
#else
  /* DC motor example */
  /* Initialise two DC motors (insert motors in M1 and M2 term blocks) */
  msDCInit(1);
  msDCInit(2);

  while(1)
  {
    uint8_t i;

    /* Forward */
    for (i=0; i<255; i++)
    {
      msDCRun(1, MS_DIR_FORWARD, i);
      msDCRun(2, MS_DIR_BACKWARD, i);
      delay();
    }
    for (i=255; i!=0; i--)
    {
      msDCRun(1, MS_DIR_FORWARD, i);
      msDCRun(2, MS_DIR_BACKWARD, i);
      delay();
    }

    /* Backward */
    for (i=0; i<255; i++)
    {
      msDCRun(1, MS_DIR_BACKWARD, i);
      msDCRun(2, MS_DIR_FORWARD, i);
      delay();
    }
    for (i=255; i!=0; i--)
    {
      msDCRun(1, MS_DIR_BACKWARD, i);
      msDCRun(2, MS_DIR_FORWARD, i);
      delay();
    }

    /* Release */
    msDCRun(1, MS_DIR_RELEASE, 0);
    msDCRun(2, MS_DIR_RELEASE, 0);

    //__WFI();
  }
#endif

  return 0;
}

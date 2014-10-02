/*
 * pca9685.c
 *
 *  Created on: 31 août 2014
 *      Author: ktown
 */

#include <math.h>
#include "pca9685.h"
#include "board.h"

#define I2C_CLK_DIVIDER        (40)
#define I2C_BITRATE            (100000)

static uint8_t       m_pca9685_i2caddr;                   /* I2C device address placeholder */
static I2C_HANDLE_T *m_pca9685_i2cHandleMaster;           /* I2C master handle and memory for ROM API */
static uint32_t      m_pca9685_i2cMasterHandleMEM[0x20];  /* I2C buffer */

/**************************************************************************/
/*!
    @brief  Reads the value of the specified 8-bit register

    @param  addr
            The address of the register that should be read

    @return The contents of the register
*/
/**************************************************************************/
uint8_t pca9685Read8(uint8_t addr)
{
  uint8_t sendData[10] = { 0 };
  uint8_t recvData[10] = { 0 };
  I2C_PARAM_T param;
  I2C_RESULT_T result;
  ErrorCode_t error_code;

  sendData[0] = m_pca9685_i2caddr;
  sendData[1] = addr;
  recvData[0] = m_pca9685_i2caddr | 0x01;

  /* Setup I2C parameters */
  param.num_bytes_send    = 2;
  param.num_bytes_rec     = 2;
  param.buffer_ptr_send   = &sendData[0];
  param.buffer_ptr_rec    = &recvData[0];
  param.stop_flag         = 1;

  /* Set timeout (much) greater than the transfer length */
  LPC_I2CD_API->i2c_set_timeout(m_pca9685_i2cHandleMaster, 100000);

  /* Do master read transfer */
  error_code = LPC_I2CD_API->i2c_master_tx_rx_poll(m_pca9685_i2cHandleMaster, &param, &result);

  /* Completed without errors? */
  if (error_code != LPC_OK)
  {
    // DEBUGOUT("i2c_master_tx_rx error code : %x\r\b", error_code);
    return 0;
  }

  return recvData[1];
}

/**************************************************************************/
/*!
    @brief  Updates the value of the specified 8-bit register

    @param  addr
            The address of the register that should be updated
    @param  value
            The value to write to the register at 'addr'
*/
/**************************************************************************/
void pca9685Write8(uint8_t addr, uint8_t value)
{
  uint8_t      sendData[3] = { 0, 0, 0 };
  I2C_PARAM_T  param;
  I2C_RESULT_T result;
  ErrorCode_t  error_code;

  sendData[0] = m_pca9685_i2caddr;
  sendData[1] = addr;
  sendData[2] = value;

  /* Setup I2C parameters */
  param.num_bytes_send    = 3;
  param.buffer_ptr_send   = &sendData[0];
  param.num_bytes_rec     = 0;
  param.stop_flag         = 1;

  /* Set timeout (much) greater than the transfer length */
  LPC_I2CD_API->i2c_set_timeout(m_pca9685_i2cHandleMaster, 100000);

  /* Do master write transfer */
  error_code = LPC_I2CD_API->i2c_master_transmit_poll(m_pca9685_i2cHandleMaster, &param, &result);

  /* Completed without errors? */
  if (error_code != LPC_OK)
  {
    /* Likely cause is NAK */
    // DEBUGOUT("I2C transaction failed (Addr = %d)\r\n", m_pca9685_i2caddr);
  }
}

/**************************************************************************/
/*!
    @brief  Sets the PWM duty cycle for a single channel.

    @param  channel
            The PWM channel to update (0..15)
    @param  on
            The 'high' time (0..4096)
    @param  off
            The 'low' time (0..4096)
*/
/**************************************************************************/
void pca9685SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
  uint8_t      tx_buffer[6];
  I2C_PARAM_T  param;
  I2C_RESULT_T result;
  ErrorCode_t  error_code;

  if (on > 4095)
  {
    on = 4096;
  }
  if (off > 4095)
  {
    off = 4096;
  }

  tx_buffer[0] = m_pca9685_i2caddr;
  tx_buffer[1] = PCA9685_REG_LED0_ON_L + 4 * channel;
  tx_buffer[2] = (uint8_t)on;
  tx_buffer[3] = (uint8_t)(on >> 8);
  tx_buffer[4] = (uint8_t)off;
  tx_buffer[5] = (uint8_t)(off >> 8);

  /* Setup I2C parameters */
  param.num_bytes_send    = 6;
  param.buffer_ptr_send   = &tx_buffer[0];
  param.num_bytes_rec     = 0;
  param.stop_flag         = 1;

  /* Set timeout (much) greater than the transfer length */
  LPC_I2CD_API->i2c_set_timeout(m_pca9685_i2cHandleMaster, 100000);

  /* Do master write transfer */
  error_code = LPC_I2CD_API->i2c_master_transmit_poll(m_pca9685_i2cHandleMaster, &param, &result);

  /* Completed without errors? */
  if (error_code != LPC_OK)
  {
    /* Likely cause is NAK */
    // DEBUGOUT("I2C transaction failed (Addr = %d)\r\n", m_pca9685_i2caddr);
  }
}

/**************************************************************************/
/*!
    @brief  Sets the base frequency (in Hz) for the PWM outputs

    @param  freq
            Base frequency (in Hz) for the PWM outputs
*/
/**************************************************************************/
void pca9685SetPWMFreq(float freq)
{
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;

  uint8_t prescale = floor(prescaleval + 0.5);
  uint8_t oldmode = pca9685Read8(PCA9685_REG_MODE1);
  uint8_t newmode = (oldmode & 0x7F) | 0x10;            // sleep
  pca9685Write8(PCA9685_REG_MODE1, newmode);         // go to sleep
  pca9685Write8(PCA9685_REG_PRESCALE, prescale);     // set the prescaler
  pca9685Write8(PCA9685_REG_MODE1, oldmode);         // wake up again
  /* ToDo: Add 5ms HW delay */
  uint16_t x = 0xFFFF;
  while(x--)
  {
    __asm volatile("nop");
  }
  pca9685Write8(PCA9685_REG_MODE1, oldmode | 0xa1);  // turn on auto increment
}


/**************************************************************************/
/*!
    @brief  Initialises the I2C bus (I2C0) in Master mode, and resets the
            outputs on the PCA9685 PWM driver

    @param  addr
            The 7-bit I2C address to use when addressing the PCA9685
*/
/**************************************************************************/
bool pca9685Init(uint8_t addr)
{
  m_pca9685_i2caddr = addr;

  /* Setup the pinmux for I2C pins */
#if defined(BOARD_NXP_LPCXPRESSO_824)
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
  Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
#if (I2C_BITRATE > 400000)
  /* Enable Fast Mode Plus for I2C pins */
  Chip_IOCON_PinSetI2CMode(LPC_IOCON, IOCON_PIO10, PIN_I2CMODE_FASTPLUS);
  Chip_IOCON_PinSetI2CMode(LPC_IOCON, IOCON_PIO11, PIN_I2CMODE_FASTPLUS);
#endif
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
#else
  /* Configure your own I2C pin muxing here if needed */
  #error "Unsupported board definition"
#endif

  /* Setup I2C in master mode */
  Chip_I2C_Init(LPC_I2C);
  m_pca9685_i2cHandleMaster = LPC_I2CD_API->i2c_setup(LPC_I2C_BASE, m_pca9685_i2cMasterHandleMEM);
  if (m_pca9685_i2cHandleMaster == NULL)
  {
    return false;
  }
  if (LPC_I2CD_API->i2c_set_bitrate(m_pca9685_i2cHandleMaster, Chip_Clock_GetSystemClockRate(),
                    I2C_BITRATE) != LPC_OK)
  {
    return false;
  }

  /* ToDo: Ping the PCA9685 to make sure it exists */

  /* Reset the PCA9685 and disable all PWM outputs */
  pca9685Write8(PCA9685_REG_MODE1, 0x0);
  pca9685SetPWMFreq(1600);
  uint8_t i;
  for (i=0; i<16; i++)
  {
    pca9685SetPWM(i, 0, 0);
  }

  return true;
}

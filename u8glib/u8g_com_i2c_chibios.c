/*

  u8g_com_null.c
 
  communication null device

  Universal 8bit Graphics Library
 
  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.
   
  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
 
 
*/

#include "u8g.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include "debug.h"

static const I2CConfig i2c_cfg = {
  OPMODE_I2C,
  100000,
  FAST_DUTY_CYCLE_2,
};

#define I2C_ADDR        0x3c
static uint8_t g_cmd_mode;
static uint8_t g_i2c_buf[256];

static void i2c_fallback_send(int len)
{
  int delay = 30000;
  
  I2C2->CR2 = I2C2->CR2 & (~I2C_CR2_DMAEN);

  chSysPolledDelayX(delay);
  I2C2->CR1 |= I2C_CR1_START;
  chSysPolledDelayX(delay);
  I2C2->DR = I2C_ADDR << 1;
  chSysPolledDelayX(delay);
  
  (void)I2C2->SR1;
  (void)I2C2->SR2;
  
  for (int i = 0; i < len; i++)
  {
    I2C2->DR = g_i2c_buf[i];
    chSysPolledDelayX(delay);
  }
  
  I2C2->CR1 |= I2C_CR1_STOP;
  chSysPolledDelayX(delay);
}

uint8_t u8g_com_i2c_chibios_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_INIT:
      break;
      
      
    case U8G_COM_MSG_STOP:
      break;

   
    case U8G_COM_MSG_CHIP_SELECT:
      g_cmd_mode = 0x00;
      break;
      
    case U8G_COM_MSG_ADDRESS:
      // arg_val=0 is cmd mode, =1 is data mode
      g_cmd_mode = (arg_val ? 0x40 : 0x00);
      break;

    case U8G_COM_MSG_WRITE_SEQ:
    case U8G_COM_MSG_WRITE_SEQ_P:
      if (arg_val + 1 > sizeof(g_i2c_buf)) return 1;
      g_i2c_buf[0] = g_cmd_mode;
      memcpy(g_i2c_buf+1, arg_ptr, arg_val);
      i2cStart(&I2CD2, &i2c_cfg);
      if (__get_PRIMASK())
      {
        i2c_fallback_send(arg_val+1);
      }
      else
      {
        i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, g_i2c_buf, arg_val+1, NULL, 0, MS2ST(100));
      }
      i2cStop(&I2CD2);
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      g_i2c_buf[0] = g_cmd_mode;
      g_i2c_buf[1] = arg_val;

      i2cStart(&I2CD2, &i2c_cfg);
      if (__get_PRIMASK())
      {
        i2c_fallback_send(2);
      }
      else
      {
        i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, g_i2c_buf, 2, NULL, 0, MS2ST(10));
      }
      i2cStop(&I2CD2);
      break;
  }
  return 1;
}


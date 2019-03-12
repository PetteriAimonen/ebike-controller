#include "tlv493.h"

#include <ch.h>
#include <hal.h>
#include "debug.h"


static const I2CConfig i2c_cfg = {
  OPMODE_I2C,
  100000,
  FAST_DUTY_CYCLE_2,
};

#define I2C_ADDR        0x5E

void tlv493_init()
{
  uint8_t resetcmd[1] = {0xFF};
  uint8_t rxbuf[10] = {0};

  i2cStart(&I2CD2, &i2c_cfg);
  i2cMasterTransmitTimeout(&I2CD2, 0x00, resetcmd, 1, NULL, 0, MS2ST(100));
  chThdSleepMilliseconds(10);
  i2cMasterTransmitTimeout(&I2CD2, 0x00, resetcmd, 1, NULL, 0, MS2ST(100));
  chThdSleepMilliseconds(10);
  i2cMasterReceiveTimeout(&I2CD2, I2C_ADDR, rxbuf, 10, MS2ST(100));

  uint8_t txbuf[4] = {0};
  txbuf[1] |= 1; // Low power mode enable
  txbuf[1] |= rxbuf[7] & 0x18; // Reserved bits
  txbuf[2] = rxbuf[8]; // Reserved bits
  txbuf[3] |= 0x40; // 100 Hz mode
  txbuf[3] |= rxbuf[9] & 0x1F; // Reserved bits

  int parity = __builtin_popcount(txbuf[0] ^ txbuf[1] ^ txbuf[2] ^ txbuf[3]) & 1;
  if (!parity) txbuf[1] |= 0x80;

  i2cMasterTransmitTimeout(&I2CD2, I2C_ADDR, txbuf, 4, NULL, 0, MS2ST(100));
  i2cStop(&I2CD2);
}

bool tlv493_read(int *x, int *y, int *z)
{
  *x = *y = *z = 0;

  uint8_t rxbuf[6] = {0};
  bool valid = false;

  i2cStart(&I2CD2, &i2c_cfg);
  int maxtries = 5;
  do {
    i2cMasterReceiveTimeout(&I2CD2, I2C_ADDR, rxbuf, 6, MS2ST(100));
    valid = (rxbuf[3] & 0x03) == 0 && (rxbuf[5] & 0x10);
    if (!valid) chThdSleepMilliseconds(1);
  } while (!valid && maxtries-- > 0);
  i2cStop(&I2CD2);

  if (!valid)
  {
    return false;
  }

  *x = ((int)(int8_t)rxbuf[0] << 4) | (rxbuf[4] >> 4);
  *y = ((int)(int8_t)rxbuf[1] << 4) | (rxbuf[4] & 0x0F);
  *z = ((int)(int8_t)rxbuf[2] << 4) | (rxbuf[5] & 0x0F);
  return true;
}

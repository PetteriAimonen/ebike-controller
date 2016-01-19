/* NOTE: This file uses 3-wire SPI to communicate with the sensor due to
 * some soldering trouble, which made MISO unavailable. Normally you could
 * just use ChibiOS SPI driver which is much better. */

#include <ch.h>
#include <hal.h>
#include "lsm6ds3.h"
#include "debug.h"

#define LSM6DS3_SPI SPID1

void lsm6ds3_init()
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  SPI1->CR1 = (4 << 3) | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BIDIMODE |
              SPI_CR1_SSM | SPI_CR1_SSI;
}

void lsm6ds3_write(uint8_t reg, uint8_t value)
{
  volatile int maxwait = 100000;
  
  chSysPolledDelayX(100);
  palClearPad(GPIOA, GPIOA_SPI1_CS);
  chSysPolledDelayX(100);
  
  LSM6DS3_SPI.spi->CR1 |= SPI_CR1_BIDIOE;
  LSM6DS3_SPI.spi->CR1 |= SPI_CR1_SPE;
  
  while (!(SPI1->SR & SPI_SR_TXE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG"); }
  SPI1->DR = reg;
  while (!(SPI1->SR & SPI_SR_TXE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG"); }
  SPI1->DR = value;
  while (!(SPI1->SR & SPI_SR_TXE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG"); }
  while (SPI1->SR & SPI_SR_BSY) { if (maxwait-- < 0) abort_with_error("SPI_WDOG"); }
  
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_SPE;
  
  chSysPolledDelayX(100);
  palSetPad(GPIOA, GPIOA_SPI1_CS);
  chSysPolledDelayX(100);
}

uint8_t lsm6ds3_read(uint8_t reg)
{
  volatile int maxwait = 100000;
  
  chSysPolledDelayX(100);
  palClearPad(GPIOA, GPIOA_SPI1_CS);
  chSysPolledDelayX(100);
  
  LSM6DS3_SPI.spi->CR1 |= SPI_CR1_BIDIOE;
  LSM6DS3_SPI.spi->CR1 |= SPI_CR1_SPE;
  
  while (!(SPI1->SR & SPI_SR_TXE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG"); }
  SPI1->DR = 0x80 | reg;
  while (!(SPI1->SR & SPI_SR_TXE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG"); }
  while (SPI1->SR & SPI_SR_BSY) { if (maxwait-- < 0) abort_with_error("SPI_WDOG"); }
  
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_SPE;
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_BIDIOE;
  LSM6DS3_SPI.spi->CR1 |= SPI_CR1_SPE;
  
  (void)SPI1->DR;
  while (!(SPI1->SR & SPI_SR_RXNE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG1"); }
  uint8_t result = SPI1->DR;

  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_SPE;
  
  chSysPolledDelayX(100);
  palSetPad(GPIOA, GPIOA_SPI1_CS);
  chSysPolledDelayX(100);

  return result;
}

bool lsm6ds3_read_acc(int* x, int* y, int* z)
{
  int maxwait = 10;
  
  uint8_t status = lsm6ds3_read(LSM6DS3_STATUS_REG);
  if (!(status & 1))
    return false;
  
  int x1, y1, z1, x2, y2, z2;
  do {
    x1 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTX_L_XL) | ((int)lsm6ds3_read(LSM6DS3_OUTX_H_XL) << 8));
    y1 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTY_L_XL) | ((int)lsm6ds3_read(LSM6DS3_OUTY_H_XL) << 8));
    z1 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTZ_L_XL) | ((int)lsm6ds3_read(LSM6DS3_OUTZ_H_XL) << 8));
  
    // Double read to make sure the data is not updated in the middle
    x2 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTX_L_XL) | ((int)lsm6ds3_read(LSM6DS3_OUTX_H_XL) << 8));
    y2 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTY_L_XL) | ((int)lsm6ds3_read(LSM6DS3_OUTY_H_XL) << 8));
    z2 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTZ_L_XL) | ((int)lsm6ds3_read(LSM6DS3_OUTZ_H_XL) << 8));
    
    if (maxwait-- < 0) abort_with_error("ACC_REREAD");
  } while (x1 != x2 || y1 != y2 || z1 != z2);
  
  *x = x1;
  *y = y1;
  *z = -z1;
  return true;
}

bool lsm6ds3_read_gyro(int* x, int* y, int* z)
{
  int maxwait = 10;
  
  uint8_t status = lsm6ds3_read(LSM6DS3_STATUS_REG);
  if (!(status & 2))
    return false;
  
  int x1, y1, z1, x2, y2, z2;
  do {
    x1 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTX_L_G) | ((int)lsm6ds3_read(LSM6DS3_OUTX_H_G) << 8));
    y1 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTY_L_G) | ((int)lsm6ds3_read(LSM6DS3_OUTY_H_G) << 8));
    z1 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTZ_L_G) | ((int)lsm6ds3_read(LSM6DS3_OUTZ_H_G) << 8));
  
    // Double read to make sure the data is not updated in the middle
    x2 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTX_L_G) | ((int)lsm6ds3_read(LSM6DS3_OUTX_H_G) << 8));
    y2 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTY_L_G) | ((int)lsm6ds3_read(LSM6DS3_OUTY_H_G) << 8));
    z2 = (int16_t)(lsm6ds3_read(LSM6DS3_OUTZ_L_G) | ((int)lsm6ds3_read(LSM6DS3_OUTZ_H_G) << 8));
    
    if (maxwait-- < 0) abort_with_error("GYR_REREAD");
  } while (x1 != x2 || y1 != y2 || z1 != z2);
  
  *x = x1;
  *y = y1;
  *z = -z1;
  return true;
}



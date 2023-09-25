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
  
  (void)SPI1->DR;
  
  __disable_irq();
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_SPE;
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_BIDIOE;
  LSM6DS3_SPI.spi->CR1 |= SPI_CR1_SPE;
  
  while (!(SPI1->SR & SPI_SR_RXNE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG1"); }
  uint8_t result = SPI1->DR;

  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_SPE;
  __enable_irq();
  
  chSysPolledDelayX(100);
  palSetPad(GPIOA, GPIOA_SPI1_CS);
  chSysPolledDelayX(100);

  return result;
}

void lsm6ds3_readmulti(uint8_t reg, uint8_t *buffer, int count)
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
  
  (void)SPI1->DR;

  __disable_irq();
  
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_SPE;
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_BIDIOE;
  LSM6DS3_SPI.spi->CR1 |= SPI_CR1_SPE;

  for (int i = 0; i < count; i++)
  {
    while (!(SPI1->SR & SPI_SR_RXNE)) { if (maxwait-- < 0) abort_with_error("SPI_WDOG1"); }
    buffer[i] = SPI1->DR;
  }
  
  LSM6DS3_SPI.spi->CR1 &= ~SPI_CR1_SPE;
  __enable_irq();
  
  chSysPolledDelayX(100);
  palSetPad(GPIOA, GPIOA_SPI1_CS);
  chSysPolledDelayX(100);
}

bool lsm6ds3_read_acc(int* x, int* y, int* z)
{
  int maxwait = 10;
  
  uint8_t status = lsm6ds3_read(LSM6DS3_STATUS_REG);
  if (!(status & 1))
    return false;
  
  uint8_t buf[6] = {};
  lsm6ds3_readmulti(LSM6DS3_OUTX_L_XL, buf, 6);

  *x = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
  *y = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
  *z = -(int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
  return true;
}

bool lsm6ds3_read_gyro(int* x, int* y, int* z)
{
  int maxwait = 10;
  
  uint8_t status = lsm6ds3_read(LSM6DS3_STATUS_REG);
  if (!(status & 2))
    return false;
  
  uint8_t buf[6] = {};
  lsm6ds3_readmulti(LSM6DS3_OUTX_L_G, buf, 6);

  *x = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
  *y = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
  *z = -(int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
  return true;
}



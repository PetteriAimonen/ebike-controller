#include "filesystem.h"

static const SPIConfig g_spislowconfig = {
  .ssport = GPIOD,
  .sspad = GPIOD_SPI3_CS,
  .cr1 = (7 << 3) | SPI_CR1_MSTR
};

static const SPIConfig g_spifastconfig = {
  .ssport = GPIOD,
  .sspad = GPIOD_SPI3_CS,
  .cr1 = (1 << 3) | SPI_CR1_MSTR
};

static const MMCConfig g_mmcconfig = {
  &SPID3,
  &g_spislowconfig,
  &g_spifastconfig
};

static FATFS g_filesystem;
MMCDriver MMCD1;

void filesystem_init()
{
  mmcStart(&MMCD1, &g_mmcconfig);
  mmcConnect(&MMCD1);
  
  f_mount(&g_filesystem, "/", 1);
}

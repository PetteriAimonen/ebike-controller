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

MMCDriver MMCD1;
static mutex_t mmc_mutex;
static uint32_t mmc_sectorcount;

int filesystem_write(uint32_t sector, const uint8_t *data, uint32_t bytes)
{
  if (mmc_sectorcount == 0) return 0;
  sector = sector % mmc_sectorcount;

  chMtxLock(&mmc_mutex);

  mmcStartSequentialWrite(&MMCD1, sector);
  int count = 0;
  while (bytes > count * MMCSD_BLOCK_SIZE)
  {
    mmcSequentialWrite(&MMCD1, data + MMCSD_BLOCK_SIZE * count);
    count++;
  }
  mmcStopSequentialWrite(&MMCD1);
  mmcSync(&MMCD1);

  chMtxUnlock(&mmc_mutex);
  return count;
}

int filesystem_read(uint32_t sector, uint8_t *data, uint32_t bytes)
{
  if (mmc_sectorcount == 0) return 0;
  sector = sector % mmc_sectorcount;

  chMtxLock(&mmc_mutex);

  mmcStartSequentialRead(&MMCD1, sector);
  int count = 0;
  while (bytes > count * MMCSD_BLOCK_SIZE)
  {
    mmcSequentialRead(&MMCD1, data + MMCSD_BLOCK_SIZE * count);
    count++;
  }
  mmcStopSequentialRead(&MMCD1);

  chMtxUnlock(&mmc_mutex);

  return count;
}

void filesystem_init()
{
  chMtxObjectInit(&mmc_mutex);
  mmcStart(&MMCD1, &g_mmcconfig);
  mmcConnect(&MMCD1);

  BlockDeviceInfo info = {};
  mmcGetInfo(&MMCD1, &info);
  mmc_sectorcount = info.blk_num;
}

#include "u8g.h"
#include <ch.h>
#include <hal.h>

void u8g_Delay(uint16_t val)
{
  chThdSleepMilliseconds(val);
}

void u8g_MicroDelay(void)
{
  chSysPolledDelayX(168);
}

void u8g_10MicroDelay(void)
{
  chSysPolledDelayX(1680);
}

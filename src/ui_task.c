#include "ui_task.h"
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <u8g.h>

static THD_WORKING_AREA(uistack, 1024);
static u8g_t u8g = {};

uint8_t u8g_com_i2c_chibios_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

static void ui_thread(void *p)
{
  chRegSetThreadName("ui");
  
  u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_i2c_chibios_fn);
  
  u8g_FirstPage(&u8g);
  do {
    u8g_SetFont(&u8g, u8g_font_8x13);
    u8g_DrawStr(&u8g, 0, 15, "Owner:");
    u8g_DrawStr(&u8g, 0, 40, "Petteri Aimonen");
    u8g_DrawStr(&u8g, 0, 60, "jpa@kapsi.fi");
  } while (u8g_NextPage(&u8g));
  
  chThdSleepMilliseconds(5000);
  
  while (1)
  {
    chThdSleepMilliseconds(1000);
    
    int secs = chVTGetSystemTime() / MS2ST(1000);
    char buf[16];
    chsnprintf(buf, sizeof(buf), "%02d:%02d:%02d",
               secs / 3600, (secs % 3600) / 60, secs % 60);
    
    u8g_FirstPage(&u8g);
    do {
      u8g_SetFont(&u8g, u8g_font_courB18);
      u8g_DrawStr(&u8g, 5, 20, buf);
    } while (u8g_NextPage(&u8g));
  }
}

void ui_start()
{
  chThdCreateStatic(uistack, sizeof(uistack), NORMALPRIO - 1, ui_thread, NULL);
}
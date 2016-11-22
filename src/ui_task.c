#include "ui_task.h"
#include "motor_sampling.h"
#include "motor_limits.h"
#include "motor_config.h"
#include "log_task.h"
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <u8g.h>
#include <stm32f4xx.h>

static THD_WORKING_AREA(uistack, 1024);
static u8g_t u8g = {};

uint8_t u8g_com_i2c_chibios_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

static float g_total_energy_wh = 0.0f;
static int g_assist_level = 50;

int ui_get_assist_level()
{
  return g_assist_level;
}

static int g_ok_button_count = 0;

int ui_get_ok_button_clicks()
{
  int oldcount = g_ok_button_count;
  g_ok_button_count = 0;
  return oldcount;
}

static void redraw_status()
{
  char buf[64];
  
  int voltage = get_battery_voltage_mV();
  int current = get_battery_current_mA();
  g_total_energy_wh += (voltage / 1000.0f * current / 1000.0f) / 3600.0f;
  int wh = (int)(g_total_energy_wh * 10);
  
  u8g_FirstPage(&u8g);
  do {
    // Current time
    u8g_SetFont(&u8g, u8g_font_courB18);
    int secs = chVTGetSystemTime() / MS2ST(1000);
    chsnprintf(buf, sizeof(buf), "%02d:%02d:%02d",
              secs / 3600, (secs % 3600) / 60, secs % 60);
    u8g_DrawStr(&u8g, 5, 20, buf);
    
    // Total energy used, assist level
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "%3d.%01d Wh  %2d%%",
               wh / 10, wh % 10, g_assist_level, log_get_fileindex());
    u8g_DrawStr(&u8g, 5, 40, buf);
    
    // Filename
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "    %04d.txt",
               log_get_fileindex());
    u8g_DrawStr(&u8g, 5, 60, buf);
  } while (u8g_NextPage(&u8g));
}

static char ui_get_button()
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
  ADC3->CR2 = ADC_CR2_ADON;
  ADC3->SQR3 = 11;
  ADC3->CR2 |= ADC_CR2_SWSTART;
  
  while (!(ADC3->SR & ADC_SR_EOC));
  
  int adc = ADC3->DR;
  
  char button;
  if (adc < 2500)
    button = ' '; // No button pressed
  else if (adc < 2600)
    button = '-'; // Minus button
  else if (adc < 2800)
    button = '+';  // Plus button
  else
    button = 'K'; // OK button
  
  return button;
}

static void ui_thread(void *p)
{
  chRegSetThreadName("ui");
  
  u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_i2c_chibios_fn);
  
  u8g_FirstPage(&u8g);
  do {
    u8g_SetFont(&u8g, u8g_font_8x13);
    u8g_DrawStr(&u8g, 0, 20, "Owner:");
    u8g_DrawStr(&u8g, 0, 40, "Petteri Aimonen");
    u8g_DrawStr(&u8g, 0, 60, "jpa@kapsi.fi");
  } while (u8g_NextPage(&u8g));
  
  chThdSleepMilliseconds(5000);
  
  float total_power_wh = 0.0f;
  
  systime_t prevTime = chVTGetSystemTime();
  char prevButton = ' ';
  while (1)
  {
    chThdSleepMilliseconds(50);
    
    char button = ui_get_button();
    if (button == ' ' || button == prevButton)
    {
      systime_t timeNow = chVTGetSystemTime();
      if (timeNow - prevTime > MS2ST(1000))
      {
        redraw_status();
        prevTime = timeNow;
      }
    }
    else if (button == '+')
    {
      if (g_assist_level < 75)
        g_assist_level += 25;
    }
    else if (button == '-')
    {
      if (g_assist_level > 25)
        g_assist_level -= 25;
    }
    else if (button == 'K')
    {
      g_ok_button_count++;
    }
    
    if (button != prevButton && button != ' ')
    {
      // Reinit to recover from any communication failures
      // Also gives a kind of "ack" flash
      u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_i2c_chibios_fn);
      prevTime = 0;
    }
    
    prevButton = button;
  }
}

void ui_start()
{
  chThdCreateStatic(uistack, sizeof(uistack), NORMALPRIO - 1, ui_thread, NULL);
}
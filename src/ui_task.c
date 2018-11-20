#include "ui_task.h"
#include "motor_sampling.h"
#include "motor_limits.h"
#include "motor_config.h"
#include "log_task.h"
#include "sensor_task.h"
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <u8g.h>
#include <stm32f4xx.h>

static THD_WORKING_AREA(uistack, 1024);
static u8g_t u8g = {};

uint8_t u8g_com_i2c_chibios_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

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

static void config_entry(int i, int selected, bool editing, const char *name, volatile int *value, int delta)
{
  char buf[64];

  if (i == selected && delta != 0 && value)
  {
    *value += delta;
  }

  i -= selected;

  if (value)
  {
    chsnprintf(buf, sizeof(buf), "%c%8s:%c%3d%c",
              (i == 0) ? '>' : ' ', name,
              (i == 0 && editing) ? '[' : ' ',
              *value,
              (i == 0 && editing) ? ']' : ' '
              );
  }
  else
  {
    chsnprintf(buf, sizeof(buf), "%c%8s",
              (i == 0) ? '>' : ' ', name);
  }

  u8g_SetFont(&u8g, u8g_font_8x13);
  u8g_DrawStr(&u8g, 0, 32 + i * 14, buf);
}

static bool config_page(char button)
{
  static int selected;
  static bool editing;

  int delta = 0;

  if (button == '+')
    delta = 1;
  else if (button == '-')
    delta = -1;
  else if (button == 'K')
    editing = !editing;

  if (!editing)
  {
    selected += delta;
    delta = 0;
    if (selected < 0) selected = 0;
  }

  int acc_x, acc_y, acc_z;

  sensors_get_accel(&acc_x, &acc_y, &acc_z);

  u8g_FirstPage(&u8g);
  do {
    config_entry(0, selected, editing, "Exit", NULL, delta);
    config_entry(1, selected, editing, "Acc.bias", &g_system_state.accelerometer_bias_mg, delta);
    config_entry(2, selected, editing, "Acc.val", &acc_z, delta);
    config_entry(3, selected, editing, "Min.volt", &g_system_state.min_voltage_V, delta);
    config_entry(4, selected, editing, "Max.batA", &g_system_state.max_battery_current_A, delta);
    config_entry(5, selected, editing, "Max.motA", &g_system_state.max_motor_current_A, delta);
    delta = 0;
  } while (u8g_NextPage(&u8g));

  if (selected == 0 && editing)
  {
    // Exit menu
    save_system_state();
    selected = 0;
    editing = false;
    return false;
  }
  else
  {
    return true;
  }
}

static void status_page(char button)
{
  char buf[64];
  
  if (button == '+')
  {
    if (g_assist_level < 75)
      g_assist_level += 25;
  }
  else if (button == '-')
  {
    if (g_assist_level > 25)
      g_assist_level -= 25;
  }

  int Wh_x10 = g_system_state.total_energy_mJ / 360000;
  int V_x10 = get_battery_voltage_mV() / 100;
  int km_x10 = g_system_state.total_distance_m / 100;
  
  u8g_FirstPage(&u8g);
  do {
    // Current time
    u8g_SetFont(&u8g, u8g_font_courB18);
    int secs = chVTGetSystemTime() / MS2ST(1000);
    chsnprintf(buf, sizeof(buf), "%02d:%02d:%02d",
              secs / 3600, (secs % 3600) / 60, secs % 60);
    u8g_DrawStr(&u8g, 5, 18, buf);
    
    // Total energy used, assist level
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "%3d.%01d Wh   %2d%%",
               Wh_x10 / 10, Wh_x10 % 10, g_assist_level);
    u8g_DrawStr(&u8g, 0, 35, buf);
    
    // Total distance, bat volts
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "%3d.%01d km %2d.%01d V",
               km_x10 / 10, km_x10 % 10, V_x10 / 10, V_x10 % 10);
    u8g_DrawStr(&u8g, 0, 50, buf);

    // Filename
    u8g_SetFont(&u8g, u8g_font_8x13);
    int fileindex = log_get_fileindex();
    if (fileindex >= 0)
    {
      chsnprintf(buf, sizeof(buf), "    %04d.txt", fileindex);
    }
    else
    {
      chsnprintf(buf, sizeof(buf), "    NO MOTOR");
    }
    u8g_DrawStr(&u8g, 5, 64, buf);
  } while (u8g_NextPage(&u8g));
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
  
  if (get_battery_voltage_mV() > 38500)
  {
    g_system_state.total_distance_m = 0;
    g_system_state.total_energy_mJ = 0;
    g_system_state.total_time_ms = 0;
  }

  bool in_settings = false;
  systime_t prevTime = chVTGetSystemTime();
  char prevButton = ' ';
  while (1)
  {
    chThdSleepMilliseconds(50);
    
    char button = ui_get_button();
    if (button == prevButton)
    {
      button = ' ';
    }
    else
    {
      prevButton = button;
    }

    if (button != ' ')
    {
      // Reinit to recover from any communication failures
      // Also gives a kind of "ack" flash
      u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_i2c_chibios_fn);
      prevTime = 0;
    }

    if (!in_settings && button == 'K')
    {
      in_settings = true;
      button = ' ';
    }

    systime_t timeNow = chVTGetSystemTime();
    if (timeNow - prevTime > MS2ST(1000))
    {
      prevTime = timeNow;

      if (!in_settings)
      {
        status_page(button);
      }
      else
      {
        if (!config_page(button))
        {
          in_settings = false;
          prevTime = 0;
        }
      }
    }
  }
}

void ui_show_msg(const char *msg)
{
  u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_i2c_chibios_fn);
  
  u8g_FirstPage(&u8g);
  do {
    u8g_SetFont(&u8g, u8g_font_8x13);
    u8g_DrawStr(&u8g, 0, 20, "CRASH:");
    u8g_DrawStr(&u8g, 0, 40, msg);
    u8g_DrawStr(&u8g, 0, 60, &msg[16]);
  } while (u8g_NextPage(&u8g));
}

void ui_start()
{
  chThdCreateStatic(uistack, sizeof(uistack), NORMALPRIO - 1, ui_thread, NULL);
}

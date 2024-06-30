#include "ui_task.h"
#include "motor_sampling.h"
#include "motor_limits.h"
#include "motor_config.h"
#include "motor_orientation.h"
#include "log_task.h"
#include "sensor_task.h"
#include "dcdc_control.h"
#include "settings.h"
#include "bike_control_task.h"
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <u8g.h>
#include <stm32f4xx.h>

static THD_WORKING_AREA(uistack, 1024);
static u8g_t u8g = {};

uint8_t u8g_com_i2c_chibios_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

extern bool g_have_motor;
extern bool g_is_powerout;
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
  ADC3->SMPR1 = (7 << 3);
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

  if (i < -2 || i > 2)
  {
    // Out of screen
    return;
  }

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

  int entries = 23;

  if (!editing)
  {
    selected += delta;
    delta = 0;
    if (selected < 0) selected = entries - 1;
    if (selected >= entries) selected = 0;
  }

  int acc_x, acc_y, acc_z;
  sensors_get_accel(&acc_x, &acc_y, &acc_z);

  int hall_in = motor_orientation_get_hall_sector();
  int temperature = get_mosfet_temperature_mC() / 1000;
  int brake = (palReadPad(GPIOB, GPIOB_BRAKE) == 0);

  u8g_FirstPage(&u8g);
  do {
    config_entry(0,  selected, editing, "Exit", NULL, delta);
    config_entry(1,  selected, editing, "Acc.bias", &g_system_state.accelerometer_bias_mg, delta * 5);
    config_entry(2,  selected, editing, "Acc.val", &acc_z, delta);
    config_entry(3,  selected, editing, "Acc.inv", &g_system_state.accelerometer_invert, delta);
    config_entry(4,  selected, editing, "Min.volt", &g_system_state.min_voltage_V, delta);
    config_entry(5,  selected, editing, "Max.batA", &g_system_state.max_battery_current_A, delta);
    config_entry(6,  selected, editing, "Max.motA", &g_system_state.max_motor_current_A, delta);
    config_entry(7,  selected, editing, "Wheeldia", &g_system_state.wheel_diameter_inch, delta);
    config_entry(8,  selected, editing, "WeightKG", &g_system_state.bike_weight_kg, delta * 5);
    config_entry(9,  selected, editing, "WhlTicks", &g_system_state.wheel_speed_ticks_per_rotation, delta);
    config_entry(10, selected, editing, "Max.km/h", &g_system_state.max_speed_kmh, delta);
    config_entry(11, selected, editing, "Torq.N/A", &g_system_state.torque_N_per_A, delta);
    config_entry(12, selected, editing, "En.Boost", &g_system_state.enable_boost, delta);
    config_entry(13, selected, editing, "Ped.Sens", &g_system_state.has_pedal_sensor, delta);
    config_entry(14, selected, editing, "Max.kRPM", &g_system_state.max_krpm, delta);
    config_entry(15, selected, editing, "Acc.time", &g_system_state.accel_time, delta);
    config_entry(16, selected, editing, "Batt.ESR", &g_system_state.battery_esr_mohm, delta * 10);
    config_entry(17, selected, editing, "Regen.A",  &g_system_state.max_regen_A, delta);
    config_entry(18, selected, editing, "BatC.Wh",  &g_system_state.battery_capacity_Wh, delta * 10);

    config_entry(19, selected, editing, "Hall in.", &hall_in, delta);
    config_entry(20, selected, editing, "Temperat", &temperature, delta);
    config_entry(21, selected, editing, "Brake", &brake, delta);
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

static void powerout_page(char button)
{
  static int control = 0;
  static int voltage = 0;
  static int current = 1;

  if (button == 'K')
  {
    control = !control;
  }
  else if (control == 0)
  {
    if (button == '+')
      voltage++;
    else if (button == '-')
      voltage--;
  }
  else
  {
    if (button == '+')
      current++;
    else if (button == '-')
      current--;
  }

  if (voltage < 0) voltage = 0;
  if (voltage > 44) voltage = 44;
  if (current < -10) current = -10;
  if (current > 10) current = 10;

  int secs = chVTGetSystemTime() / MS2ST(1000);
  int V_x10 = get_battery_voltage_mV() / 100;
  int Wh_x10 = g_system_state.total_energy_mJ / 360000;

  int current_x100 = get_dcdc_current_mA() / 10;
  int abscur = (current_x100 < 0) ? -current_x100 : current_x100;

  u8g_FirstPage(&u8g);
  do {
    char buf[64];

    // Current time
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "%02d:%02d:%02d %2d.%01d V",
              secs / 3600, (secs % 3600) / 60, secs % 60, V_x10 / 10, V_x10 % 10);
    u8g_DrawStr(&u8g, 0, 14, buf);
    
    // Output voltage and current
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf),
              (control == 0) ? "[ %2d V ]  %2d A" : "%2d V  [ %2d A ]",
               voltage, current);
    u8g_DrawStr(&u8g, 0, 28, buf);

    // Output voltage and current
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "%s %2d.%02d A",
               (current_x100 >= 0) ? "Out:" : "In: ", abscur / 100, abscur % 100);
    u8g_DrawStr(&u8g, 0, 42, buf);

    // Total energy used
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "Tot: %3d.%01d Wh",
               Wh_x10 / 10, Wh_x10 % 10);
    u8g_DrawStr(&u8g, 0, 56, buf);
  } while (u8g_NextPage(&u8g));

  set_dcdc_mode(voltage * 1000, current * 1000);
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
  int secs = chVTGetSystemTime() / MS2ST(1000);
  u8g_FirstPage(&u8g);
  do {
    if (g_have_motor)
    {
      // Current time
      u8g_SetFont(&u8g, u8g_font_courB18);
      chsnprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                secs / 3600, (secs % 3600) / 60, secs % 60);
      u8g_DrawStr(&u8g, 5, 18, buf);
    }
    else
    {
      u8g_SetFont(&u8g, u8g_font_courB18);
      u8g_DrawStr(&u8g, 5, 18, "NO MOTOR");
    }
    
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

    // Total distance ever
    u8g_SetFont(&u8g, u8g_font_8x13);
    chsnprintf(buf, sizeof(buf), "Total: %5d km", g_system_state.alltime_distance_m / 1000);
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
  
  for (int i = 0; i < 500; i++)
  {
    chThdSleepMilliseconds(10);
    if (ui_get_button() != ' ') break;
  }
  
  bool in_settings = false;
  systime_t prevTime = chVTGetSystemTime();
  systime_t pressTime = chVTGetSystemTime();
  char prevButton = ' ';
  uint32_t iter = 0;

  systime_t prev_sysstate_save = chVTGetSystemTime();

  while (1)
  {
    iter++;
    chThdSleepMilliseconds(50);

    char button = ui_get_button();
    chThdSleepMilliseconds(20);
    char button2 = ui_get_button();

    // Debounce loop
    while (button != button2)
    {
        button = ui_get_button();
        chThdSleepMilliseconds(20);
        button2 = ui_get_button();
    }

    if (button == '+') log_event(EVENT_BUTTON_PLUS);
    if (button == '-') log_event(EVENT_BUTTON_MINUS);
    if (button == 'K') log_event(EVENT_BUTTON_OK);

    // Keep track how many milliseconds button is pressed for
    if (prevButton == ' ' && button != ' ')
    {
      pressTime = chVTGetSystemTime();

      // Reinit to recover from any communication failures
      // Also gives a kind of "ack" flash
      u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_i2c_chibios_fn);
      prevTime = 0;
    }
    int ms_pressed = (systime_t)(chVTGetSystemTime() - pressTime);

    if (!in_settings && prevButton == 'K' && button == ' ')
    {
      // Enter settings menu if K pressed for 3-10 seconds
      if (ms_pressed >= 3000 && ms_pressed <= 10000)
      {
        in_settings = true;
      }
    }

    // Autorepeat plus and minus keys if held for more than a second
    if (button == prevButton && ms_pressed > 1000 && (button == '+' || button == '-'))
    {
      // Autorepeat
      if (ms_pressed < 2000)
      {
        if ((iter & 7) != 0) button = ' ';
      }
    }
    else if (button == prevButton)
    {
      // Button is ' ' while it is being held down
      button = ' ';
    }
    else
    {
      prevButton = button;
    }

    // Display currently active page
    systime_t timeNow = chVTGetSystemTime();
    if (timeNow - prevTime > MS2ST(1000) || button != ' ')
    {
      prevTime = timeNow;

      if (in_settings)
      {
        if (!config_page(button))
        {
          in_settings = false;
          prevTime = 0;
        }
      }
      else if (g_is_powerout)
      {
        powerout_page(button);
      }
      else
      {
        status_page(button);
      }
    }

    // Periodically save system state when stopped
    systime_t time_since_save = chVTGetSystemTime() - prev_sysstate_save;
    if (!in_settings && time_since_save > S2ST(30) && bike_control_get_motor_current() == 0)
    {
      save_system_state();
      prev_sysstate_save = chVTGetSystemTime();
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

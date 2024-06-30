#include <ch.h>
#include <hal.h>
#include <shell.h>
#include "board.h"
#include "usb_usart.h"
#include "bluetooth_usart.h"
#include "filesystem.h"
#include "motor_control.h"
#include "motor_orientation.h"
#include "motor_sampling.h"
#include "dcdc_control.h"
#include "log_task.h"
#include "sensor_task.h"
#include "bike_control_task.h"
#include "ui_task.h"
#include "debug.h"
#include "ws2812.h"
#include "settings.h"

void enable_trace()
{
  // Enable SWO trace
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
  
  if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
  {
      // Some (all?) STM32s don't allow writes to DBGMCU register until
      // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
      // CPU itself, so in practice you need to connect to the CPU with
      // a debugger once before resetting it.
      return;
  }

  /* Configure Trace Port Interface Unit */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
  TPI->ACPR = 1; // Trace clock = HCLK/(x+1) = 84MHz
  TPI->SPPR = 1; // Pin protocol = Manchester
  TPI->FFCR = 0x100; // ITM trace only
  
  /* Configure PC sampling and exception trace  */
  DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                          // 0 = x32, 1 = x512
            | (7 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                              // Divider = value + 1
            | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
            | (0 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                              // 0 = Off, 1 = Every 2^23 cycles,
                                              // 2 = Every 2^25, 3 = Every 2^27
            | (0 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
            | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter
            
  /* Configure instrumentation trace macroblock */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
            | (1 << ITM_TCR_DWTENA_Pos) // Enable events from DWT
            | (1 << ITM_TCR_SYNCENA_Pos) // Enable sync packets
            | (1 << ITM_TCR_ITMENA_Pos); // Main enable for ITM
  ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports 
}

static void check_battery_full()
{
  static bool cleared = false;
  if (cleared) return;

  int battery_mV = get_battery_voltage_mV();
  if (battery_mV > 41000 || (battery_mV > 40000 && g_system_state.total_energy_mJ > 100000000))
  {
    // Battery fully charged
    g_system_state.total_distance_m = 0;
    g_system_state.total_energy_mJ = 0;
    g_system_state.total_time_ms = 0;
    save_system_state();
    cleared = true;
  }
}

static void bootup_animation()
{
  // Bootup animation and battery level display
  int batt_level = battery_percent() * 17 / 100;
  for (int i = 0; i < 27; i++)
  {
    int r = 0, g = 0, b = 0;
    if (i < 10)
    {
      r = 64;
      g = 0;
      b = 0;
    }
    else if (i - 10 < batt_level)
    {
      if (i - 10 < 10)
      {
        r = 64;
        g = 32;
        b = 0;
      }
      else
      {
        r = 0;
        g = 64;
        b = 0;
      }
    }
    ws2812_write_led(26 - i, r, g, b);
    ws2812_write_led(27 + i, r, g, b);
    chThdSleepMilliseconds(25);
  }
}

bool g_have_motor;
bool g_is_powerout;

int main(void)
{
    halInit();
    chSysInit();
    ws2812_init();
    shellInit();

    load_system_state();

    start_bluetooth_shell();
    
    filesystem_init();
    sensors_start();
    ui_start();

    
    g_have_motor = (motor_orientation_get_hall_sector() >= 0);
    
    if (!g_have_motor)
    {
        // Retry to make sure
        chThdSleepMilliseconds(100);
        g_have_motor = (motor_orientation_get_hall_sector() >= 0);
        g_is_powerout = (motor_orientation_get_hall_sector() == -2);
    }
    
    if (g_have_motor)
    {
      start_motor_control();
      start_log();
      start_bike_control();
    }
    else if (g_is_powerout)
    {
      start_dcdc_control();
    }
    else
    {
      motor_sampling_init(); // For battery voltage
    }
    
    chThdSleepMilliseconds(50);
    check_battery_full();
    bootup_animation();

    enable_trace();
    
    while (true)
    {
        if (!g_have_motor && !g_is_powerout)
        {
          motor_sampling_update();
          bike_control_update_leds(); // For battery status
        }

        chThdSleepMilliseconds(5);
        
        check_usb_usart();

        if (chVTGetSystemTime() < 5000)
        {
          check_battery_full();
        }
    }
}

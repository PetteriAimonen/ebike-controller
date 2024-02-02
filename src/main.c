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

bool g_have_motor;

int main(void)
{
    halInit();
    chSysInit();
    ws2812_init();
    shellInit();

    start_bluetooth_shell();
    
    filesystem_init();
    sensors_start();
    ui_start();

    // Bootup animation
    for (int i = 0; i < 27; i++)
    {
      ws2812_write_led(26 - i, 64, 0, 0);
      ws2812_write_led(27 + i, 64, 0, 0);
      chThdSleepMilliseconds(25);
    }
    
    g_have_motor = (motor_orientation_get_hall_sector() >= 0);
    
    if (!g_have_motor)
    {
        // Retry to make sure
        chThdSleepMilliseconds(100);
        g_have_motor = (motor_orientation_get_hall_sector() >= 0);
    }
    
    load_system_state();

    if (g_have_motor)
    {
      start_motor_control();
      start_log();
      start_bike_control();
    }
    else
    {
      motor_sampling_init(); // For battery voltage
    }
    
    enable_trace();
    
    while (true)
    {
        if (!g_have_motor)
        {
          motor_sampling_update();
          bike_control_update_leds(); // For battery status
        }

        chThdSleepMilliseconds(5);
        
        check_usb_usart();
    }
}

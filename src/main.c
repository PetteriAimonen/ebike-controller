#include <ch.h>
#include <hal.h>
#include <shell.h>
#include "board.h"
#include "usb_usart.h"
#include "bluetooth_usart.h"
#include "filesystem.h"
#include "motor_control.h"
#include "motor_orientation.h"
#include "log_task.h"
#include "sensor_task.h"
#include "bike_control_task.h"
#include "ui_task.h"
#include "debug.h"

void enable_trace()
{
  // Enable SWO trace
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
  
  /* Configure Trace Port Interface Unit */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
  TPI->ACPR = 20; // Trace clock = HCLK/(x+1) = 8MHz
  TPI->SPPR = 2; // Pin protocol = NRZ/USART
  TPI->FFCR = 0x100; // ITM trace only
  
  /* Configure PC sampling and exception trace  */
  DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                          // 0 = x32, 1 = x512
            | (3 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                              // Divider = value + 1
            | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
            | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                              // 0 = Off, 1 = Every 2^23 cycles,
                                              // 2 = Every 2^25, 3 = Every 2^27
            | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
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
    shellInit();

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
    }
    
    if (g_have_motor)
    {
      start_motor_control();
      start_log();
      start_bike_control();
    }
    
//     enable_trace();
    
    while (true)
    {
        palClearPad(GPIOC, GPIOC_LED_GREEN);
        chThdSleepMilliseconds(500);
        palSetPad(GPIOC, GPIOC_LED_GREEN);
        chThdSleepMilliseconds(500);
    }
}

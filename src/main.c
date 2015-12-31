#include <ch.h>
#include <hal.h>
#include <shell.h>
#include "board.h"
#include "usb_usart.h"
#include "bluetooth_usart.h"

int main(void)
{
    halInit();
    chSysInit();
    shellInit();

    start_bluetooth_shell();
    
    while (true)
    {
        palClearPad(GPIOC, GPIOC_LED_GREEN);
        chThdSleepMilliseconds(500);
        palSetPad(GPIOC, GPIOC_LED_GREEN);
        chThdSleepMilliseconds(500);
        
        check_usb_usart();
    }
}

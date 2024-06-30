/* Handles the starting/stopping of worker thread for the USB shell */

#include <ch.h>
#include <hal.h>
#include <shell.h>
#include "board.h"
#include "debug.h"
#include "usbcfg.h"
#include "usb_usart.h"
#include "shell_commands.h"

SerialUSBDriver SDU1;

static const ShellConfig usb_shellconfig = {
    (BaseSequentialStream*)&SDU1,
    shell_commands
};

static THD_WORKING_AREA(usb_shellstack, 1024);
static thread_t *usb_shell = NULL;
static bool usb_initialized = false;

void check_usb_usart()
{
  if (!usb_initialized)
  {
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
    * Activates the USB driver and then the USB bus pull-up on D+.
    * Note, a delay is inserted in order to not have to disconnect the cable
    * after a reset.
    */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
    
    usb_initialized = true;
  }
  
  if (!usb_shell && (SDU1.config->usbp->state == USB_ACTIVE))
  {
    usb_shell = shellCreateStatic(&usb_shellconfig, usb_shellstack, sizeof(usb_shellstack), NORMALPRIO);
  }
  else if (usb_shell && chThdTerminatedX(usb_shell))
  {
    usb_shell = NULL;
  }
}

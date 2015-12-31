
#include <ch.h>
#include <hal.h>
#include <shell.h>
#include "shell_commands.h"

static const ShellConfig btshellconfig = {
    (BaseSequentialStream*)&SD6,
    shell_commands
};

static const SerialConfig serialconfig =
{
  9600,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

static THD_WORKING_AREA(btshellstack, 1024);

void start_bluetooth_shell()
{
  sdStart(&SD6, &serialconfig);
  shellCreateStatic(&btshellconfig, btshellstack, sizeof(btshellstack), NORMALPRIO);
}

#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <chprintf.h>
#include "board.h"

void dbg(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  chvprintf((BaseSequentialStream*)&SD6, fmt, ap);
  chprintf((BaseSequentialStream*)&SD6, "\r\n");
  va_end(ap);
}

void **HARDFAULT_PSP;
register void *stack_pointer asm("sp");
static volatile int delay;

void __attribute__((naked)) HardFault_Handler()
{
    // Hijack the process stack pointer to make backtrace work
    asm("mrs %0, psp" : "=r"(HARDFAULT_PSP) : :);
    (void)SCB->CFSR;
    
    stack_pointer = HARDFAULT_PSP;

    for (;;) {}
}

#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <chprintf.h>
#include "board.h"
#include "usb_usart.h"

void dbg(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  chvprintf((BaseSequentialStream*)&SD6, fmt, ap);
  chprintf((BaseSequentialStream*)&SD6, "\r\n");
  va_end(ap);
}

void abort_with_error(const char *fmt, ...)
{
  palClearPad(GPIOB, GPIOB_EN_GATE);
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  
  while (1)
  {
    va_list ap;
    va_start(ap, fmt);
    chvprintf((BaseSequentialStream*)&SD6, fmt, ap);
    chprintf((BaseSequentialStream*)&SD6, "\r\n");
    va_end(ap);
    
    chSysPolledDelayX(168000000);
  }
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

    abort_with_error("HF %08x", *((uint32_t*)stack_pointer + 6));
}

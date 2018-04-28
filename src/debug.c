#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <chprintf.h>
#include <memstreams.h>
#include <string.h>
#include "board.h"
#include "usb_usart.h"
#include "ui_task.h"

static char g_dbgmsg[128];

void vdbg(const char *fmt, va_list ap)
{
  MemoryStream ms;
  msObjectInit(&ms, (uint8_t *)g_dbgmsg, sizeof(g_dbgmsg) - 1, 0);
  memset(g_dbgmsg, 0, sizeof(g_dbgmsg));
  
  chvprintf((BaseSequentialStream*)&ms, fmt, ap);
  chvprintf((BaseSequentialStream*)&ms, "\r\n", ap);
  
  for (char *p = g_dbgmsg; *p; p++)
  {
    while ((USART6->SR & USART_SR_TXE) == 0) {}
    USART6->DR = *p;
  }
}

void dbg(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vdbg(fmt, ap);
  va_end(ap);
}

void abort_with_error(const char *fmt, ...)
{
  __disable_irq();
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM3->CNT = 0;
  
  for (;;)
  {
    va_list ap;
    va_start(ap, fmt);
    vdbg(fmt, ap);
    va_end(ap);
    
    ui_show_msg(g_dbgmsg);
    
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

    abort_with_error("HF %08x %08x", *((uint32_t*)stack_pointer + 6), SCB->CFSR);
}

#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <stm32f4xx.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "shell_commands.h"

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
    size_t n, size;

    n = chHeapStatus(NULL, &size);
    chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
    chprintf(chp, "heap fragments   : %u\r\n", n);
    chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

extern unsigned long __main_thread_stack_base__, __main_thread_stack_end__; // From linker script
static void thread_free_stack(thread_t *thread, int *free_stack_now, int *free_stack_min)
{
    uint32_t current_sp = (uint32_t)thread->p_ctx.r13;
    uint32_t stack_bottom;
    if (current_sp >= (uint32_t)&__main_thread_stack_base__
        && current_sp <= (uint32_t)&__main_thread_stack_end__)
        stack_bottom = (uint32_t)&__main_thread_stack_base__;
    else
        stack_bottom = (uint32_t)(thread + 1);
    
    *free_stack_now = current_sp - stack_bottom;
    
    uint32_t *stackentry = (uint32_t*)stack_bottom;
    uint32_t empty_val = *stackentry;
    while (*stackentry == empty_val) stackentry++;
    *free_stack_min = (uint32_t)stackentry - stack_bottom;
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
    static const char *states[] = {CH_STATE_NAMES};
    thread_t *tp;

    chprintf(chp, "             addr   stack   prio  state   free stack now/min\r\n");
    tp = chRegFirstThread();
    do {
        int stacknow, stackmin;
        thread_free_stack(tp, &stacknow, &stackmin);
        
        chprintf(chp, "%8s %.8lx %.8lx %4lu %9s %lu/%lu\r\n",
            tp->p_name, (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, states[tp->p_state], stacknow, stackmin);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[])
{
    NVIC_SystemReset();
}


const ShellCommand shell_commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"reboot", cmd_reboot},
  {NULL, NULL}
};

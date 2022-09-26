#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <stm32f4xx.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "board.h"
#include "shell_commands.h"
#include "motor_control.h"
#include "dcdc_control.h"
#include "motor_orientation.h"
#include "motor_sampling.h"
#include <ff.h>
#include "sensor_task.h"
#include "bike_control_task.h"
#include "motor_limits.h"
#include "wheel_speed.h"

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

static void cmd_hall(BaseSequentialStream *chp, int argc, char *argv[])
{
  int b = 0;
  do {
    chprintf(chp, "HALL1: %d, HALL2: %d, HALL3: %d\r\n",
           palReadPad(GPIOB, GPIOB_HALL_1),
           palReadPad(GPIOB, GPIOB_HALL_2),
           palReadPad(GPIOC, GPIOC_HALL_3));
    
    // End if enter is pressed
    b = chnGetTimeout((BaseChannel*)chp, MS2ST(10));
  } while (argc > 0 && b != Q_RESET && b != '\r');
}

static void cmd_start_motor_control(BaseSequentialStream *chp, int argc, char *argv[])
{
  start_motor_control();
}

static void cmd_motor_pwm(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 2)
  {
    chprintf(chp, "Usage: motor_pwm <angle> <pwm>\r\n");
    return;
  }
  
  set_motor_pwm(atoi(argv[0]), atoi(argv[1]));
  
  int b = 0;
  do {
    chprintf(chp, "Hall sector: %d\r\n", motor_orientation_get_hall_sector());
    
    // End if enter is pressed
    b = chnGetTimeout((BaseChannel*)chp, MS2ST(10));
  } while (argc > 0 && b != Q_RESET && b != '\r');
}

static void cmd_motor_rotate(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 2)
  {
    chprintf(chp, "Usage: motor_rotate <rpm> <pwm>\r\n");
    return;
  }
  
  int rpm = atoi(argv[0]);
  int pwm = atoi(argv[1]);
  
  int degs_per_ms = (rpm * 360) / 60000;
  if (degs_per_ms < 1) degs_per_ms = 1;
  
  int angle = 0;
  int b = 0;
  do {
    angle += degs_per_ms;
    if (angle >= 360) angle -= 360;
    set_motor_pwm(angle, pwm);
    
    // End if enter is pressed
    b = chnGetTimeout((BaseChannel*)chp, MS2ST(1));
    
    char buf[64];
    chsnprintf(buf, sizeof(buf), "%4d %4d %4d %4d\r\n",
               angle, motor_orientation_get_angle(), motor_orientation_get_fast_rpm(), motor_orientation_get_hall_angle());
    chSequentialStreamWrite(chp, (void*)buf, strlen(buf));
  } while (argc > 0 && b != Q_RESET && b != '\r');
}

static void cmd_motor_run(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 2)
  {
    chprintf(chp, "Usage: motor_run <torque_mA> <advance_deg>\r\n");
    return;
  }
  
  int torque_mA = atoi(argv[0]);
  int advance = atoi(argv[1]);
  
  motor_run(torque_mA, advance);
  
  int b = 0;
  int i = 0;
  do {
    // End if enter is pressed
    b = chnGetTimeout((BaseChannel*)chp, MS2ST(100));    
    
    chprintf(chp, "%6d %6d RPM, %6d mV, %6d mA, %6d Tmotor, %6d Tmosfet, %d ticks\r\n",
             i++, motor_orientation_get_fast_rpm(), get_battery_voltage_mV(), get_battery_current_mA(),
             get_motor_temperature_mC() / 1000, get_mosfet_temperature_mC() / 1000,
             motor_get_interrupt_time()
            );
  } while (argc > 0 && b != Q_RESET && b != '\r');
}

static void cmd_motor_samples(BaseSequentialStream *chp, int argc, char *argv[])
{
  motor_sampling_print(chp);
}

static void cmd_dcdc_out(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 2)
  {
    chprintf(chp, "Usage: dcdc_out <voltage_mV> <current_mA>\r\n");
    return;
  }

  int mV = atoi(argv[0]);
  int mA = atoi(argv[1]);

  start_dcdc_control();
  set_dcdc_mode(DCDC_OUTPUT_CCCV, mV, mA);
}

static void cmd_ls(BaseSequentialStream *chp, int argc, char *argv[])
{
  DIR directory;
  FILINFO file;
  FRESULT status;
  
  status = f_opendir(&directory, "/");
  
  if (status != FR_OK)
  {
    chprintf(chp, "Failed to open SD card: %d\r\n", status);
    return;
  }
  
  while ((status = f_readdir(&directory, &file)) == FR_OK
         && file.fname[0] != '\0')
  {
    chprintf(chp, "%8ld %s\r\n", file.fsize, file.fname);
  }
}

static void cmd_cat(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0)
  {
    chprintf(chp, "usage: cat <file>\r\n");
    return;
  }
  
  FIL f;
  FRESULT status;
  status = f_open(&f, argv[0], FA_READ);
  
  if (status != FR_OK)
  {
    chprintf(chp, "Failed to open file: %d\r\n", status);
    return;
  }
  
  char buf[128];
  unsigned bytes_read;
  while ((status = f_read(&f, buf, sizeof(buf), &bytes_read)) == FR_OK
         && bytes_read > 0)
  {
    chSequentialStreamWrite(chp, (void*)buf, bytes_read);
  }
  
  f_close(&f);
}

static void cmd_sensors(BaseSequentialStream *chp, int argc, char *argv[])
{
  int b = 0;
  do {
    int x, y, z, gx, gy, gz;
    sensors_get_accel(&x, &y, &z);
    sensors_get_gyro(&gx, &gy, &gz);
    chprintf(chp, "ACC: %8d %8d %8d    GYRO: %8d %8d %8d\r\n", x, y, z, gx, gy, gz);
    
    // End if enter is pressed
    b = chnGetTimeout((BaseChannel*)chp, MS2ST(100));
  } while (b != Q_RESET && b != '\r');
}

static void cmd_status(BaseSequentialStream *chp, int argc, char *argv[])
{
  chprintf(chp, "Battery voltage:      %8d mV\r\n", get_battery_voltage_mV());
  chprintf(chp, "Battery current:      %8d mA\r\n", get_battery_current_mA());
  chprintf(chp, "Mosfet temperature:   %8d mC\r\n", get_mosfet_temperature_mC());
  chprintf(chp, "Motor RPM:            %8d\r\n",    motor_orientation_get_rpm());
  chprintf(chp, "Wheel velocity:       %8d m/s\r\n", (int)wheel_speed_get_velocity());
  chprintf(chp, "Wheel distance:       %8d m\r\n",  wheel_speed_get_distance());
  chprintf(chp, "Acceleration:         %8d mg\r\n", bike_control_get_acceleration());
  chprintf(chp, "Motor target current: %8d mA\r\n", bike_control_get_motor_current());
  chprintf(chp, "Motor max duty:       %8d\r\n",    motor_limits_get_max_duty());
  
  int p1, p3;
  motor_get_currents(&p1, &p3);
  chprintf(chp, "Phase currents:       %8d mA, %8d mA\r\n", p1, p3);
}

static void cmd_i2c(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc < 3)
  {
    chprintf(chp, "Usage: i2c <addr> <reg> <data>\r\n");
  }
  
  int addr = strtol(argv[0], NULL, 16);
  int mode = strtol(argv[1], NULL, 16);
  int reg = strtol(argv[2], NULL, 16);
  int dat = strtol(argv[3], NULL, 16);
  
  static const I2CConfig i2c_cfg = {
    OPMODE_I2C,
    50000,
    STD_DUTY_CYCLE,
  };
  
  i2cStart(&I2CD2, &i2c_cfg);
  uint8_t txbuf[3] = {mode, reg, dat};
  msg_t s = i2cMasterTransmitTimeout(&I2CD2, addr, txbuf, 3, NULL, 0, MS2ST(100));
  i2cStop(&I2CD2);
  
  chprintf(chp, "Status: %d\r\n", s);
}

#include "u8g.h"

uint8_t u8g_com_i2c_chibios_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

// #include "oled.h"

static void cmd_oled(BaseSequentialStream *chp, int argc, char *argv[])
{
  u8g_t u8g = {};
  u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_i2c_chibios_fn);
  
  chThdSleepMilliseconds(50);
  
  u8g_FirstPage(&u8g);
  do {
    u8g_SetFont(&u8g, u8g_font_courB18);
    u8g_DrawStr(&u8g, 10, 10, "Hello!");
    u8g_DrawStr(&u8g, 10, 30, "Hello!");
  } while (u8g_NextPage(&u8g));
  
//   oled_init();
}

const ShellCommand shell_commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"reboot", cmd_reboot},
  {"hall", cmd_hall},
  {"start_motor_control", cmd_start_motor_control},
  {"motor_pwm", cmd_motor_pwm},
  {"motor_rotate", cmd_motor_rotate},
  {"motor_run", cmd_motor_run},
  {"motor_samples", cmd_motor_samples},
  {"ls", cmd_ls},
  {"cat", cmd_cat},
  {"sensors", cmd_sensors},
  {"status", cmd_status},
  {"i2c", cmd_i2c},
  {"oled", cmd_oled},
  {"dcdc_out", cmd_dcdc_out},
  {NULL, NULL}
};

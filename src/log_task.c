#include <ch.h>
#include <hal.h>
#include <ff.h>
#include <chprintf.h>
#include <stdlib.h>
#include "log_task.h"
#include "motor_sampling.h"
#include "motor_orientation.h"
#include "sensor_task.h"

static uint8_t g_logbuffer1[4096];
static uint8_t g_logbuffer2[4096];
static THD_WORKING_AREA(logsaverstack, 1024);
static THD_WORKING_AREA(logwriterstack, 1024);
static thread_t *g_logsaver;

#define EVENT_BUF1 1
#define EVENT_BUF2 2

static int next_free_filename()
{
  DIR directory;
  FILINFO file;
  FRESULT status;
  
  status = f_opendir(&directory, "/");
  
  int max = 0;
  while ((status = f_readdir(&directory, &file)) == FR_OK
         && file.fname[0] != '\0')
  {
    int i = atoi(file.fname);
    if (i > max)
      max = i;
  }
  
  return max + 1;
}

void log_saver_thread(void *p)
{
  chRegSetThreadName("logsaver");
  
  char filename[16];
  chsnprintf(filename, sizeof(filename), "%04d.txt", next_free_filename());
  
  unsigned bytes_written;
  FIL file;
  f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
  
  static const char header[] = "# SysTime   BattU    BattI     Tmotor  Tmosfet     RPM     "
                               "AccX    AccY    AccZ\r\n"
                               "#      ms      mV       mA         mC       mC             "
                               "  mg      mg      mg\r\n";
  f_write(&file, header, sizeof(header) - 1, &bytes_written);
  
  for (;;)
  {
    eventmask_t event = chEvtWaitOne(EVENT_BUF1 | EVENT_BUF2);
    
    if (event & EVENT_BUF1)
    {
      f_write(&file, g_logbuffer1, sizeof(g_logbuffer1), &bytes_written);
    }
    
    if (event & EVENT_BUF2)
    {
      f_write(&file, g_logbuffer2, sizeof(g_logbuffer2), &bytes_written);
    }
    
    f_sync(&file);
  }
}

void log_writer_thread(void *p)
{
  int write_next = EVENT_BUF1;
  int writeptr = 0;
  
  chRegSetThreadName("logwriter");
  chThdSleepMilliseconds(2000);
  
  for (;;)
  {
    chThdSleepMilliseconds(100);
    
    int x, y, z;
    sensors_get_accel(&x, &y, &z);
    
    static char buf[512];
    chsnprintf(buf, sizeof(buf),
             "%8d %8d %8d %8d %8d %8d %8d %8d %8d\r\n",
             chVTGetSystemTime(),
             get_battery_voltage_mV(), get_battery_current_mA(),
             get_motor_temperature_mC(), get_mosfet_temperature_mC(),
             motor_orientation_get_rpm(), x, y, z);
    
    char *p = buf;
    while (*p)
    {
      uint8_t *dest = (write_next == EVENT_BUF1) ? g_logbuffer1 : g_logbuffer2;
      while (*p && writeptr < sizeof(g_logbuffer1))
      {
        dest[writeptr++] = *p++;
      }
      
      if (writeptr == sizeof(g_logbuffer1))
      {
        chEvtSignal(g_logsaver, write_next);
        write_next = (write_next == EVENT_BUF1) ? EVENT_BUF2 : EVENT_BUF1;
        writeptr = 0;
      }
    }
  }
}

void start_log()
{
  g_logsaver = chThdCreateStatic(logsaverstack, sizeof(logsaverstack), NORMALPRIO + 1, log_saver_thread, NULL);
  chThdCreateStatic(logwriterstack, sizeof(logwriterstack), NORMALPRIO + 2, log_writer_thread, NULL);
}


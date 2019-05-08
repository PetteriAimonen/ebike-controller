#include <ch.h>
#include <hal.h>
#include <ff.h>
#include <chprintf.h>
#include <stdlib.h>
#include "log_task.h"
#include "ui_task.h"
#include "motor_sampling.h"
#include "motor_orientation.h"
#include "motor_limits.h"
#include "wheel_speed.h"

volatile system_state_t g_system_state = {
  .accelerometer_bias_mg = 0,
  .min_voltage_V = 33,
  .max_motor_current_A = 15,
  .max_battery_current_A = 8
};

static uint8_t g_logbuffer1[4096];
static uint8_t g_logbuffer2[4096];
static THD_WORKING_AREA(logsaverstack, 1024);
static THD_WORKING_AREA(logwriterstack, 1024);
static thread_t *g_logsaver;
static int g_fileindex = -1;

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

int log_get_fileindex()
{
  return g_fileindex;
}

void log_saver_thread(void *p)
{
  chRegSetThreadName("logsaver");
  
  char filename[16];
  g_fileindex = next_free_filename();
  chsnprintf(filename, sizeof(filename), "%04d.txt", g_fileindex);
  
  unsigned bytes_written;
  FIL file;
  f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
  
  static const char header[] = "# SysTime    Dist.     Vel.    WAcc.    BattU    BattI    Tmosfet     RPM     Duty"
                               "   Accel  Current    State   Clicks\r\n"
                               "#      ms       m      mm/s   mm^2/s       mV       mA         mC     rpm      pwm"
                               "   mm/s^2        mA       \r\n";
  f_write(&file, header, sizeof(header) - 1, &bytes_written);
  
  systime_t prev_sysstate_save = chVTGetSystemTime();

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

    if (chVTGetSystemTime() > prev_sysstate_save + S2ST(10))
    {
      save_system_state();
      prev_sysstate_save = chVTGetSystemTime();
    }
  }
}

void log_writer_thread(void *p)
{
  int write_next = EVENT_BUF1;
  int writeptr = 0;
  
  chRegSetThreadName("logwriter");
  chThdSleepMilliseconds(2000);
  
  int prev_distance = 0;
  systime_t prev_time = 0;

  for (;;)
  {
    chThdSleepMilliseconds(50);
    
    static char buf[512];
    chsnprintf(buf, sizeof(buf),
             "%8d %8d %8d %8d %8d %8d %8d %8d %8d %8d\r\n",
             chVTGetSystemTime(),
             wheel_speed_get_distance(), (int)(wheel_speed_get_velocity() * 1000.0f), (int)(wheel_speed_get_acceleration() * 1000.0f),
             get_battery_voltage_mV(), get_battery_current_mA(),
             get_mosfet_temperature_mC(), motor_orientation_get_rpm(), motor_limits_get_max_duty());
    
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

    g_system_state.prev_voltage_mV = get_battery_voltage_mV();
    int delta_d = wheel_speed_get_distance() - prev_distance;
    g_system_state.total_distance_m += delta_d;
    prev_distance += delta_d;
    systime_t delta_t = chVTGetSystemTime() - prev_time;
    g_system_state.total_time_ms += ST2MS(delta_t);
    prev_time += delta_t;
    g_system_state.total_energy_mJ += (get_battery_voltage_mV() * get_battery_current_mA() / 1000 * (int)(ST2MS(delta_t)) + 500) / 1000;
  }
}

void start_log()
{
  g_logsaver = chThdCreateStatic(logsaverstack, sizeof(logsaverstack), NORMALPRIO + 1, log_saver_thread, NULL);
  chThdCreateStatic(logwriterstack, sizeof(logwriterstack), NORMALPRIO + 2, log_writer_thread, NULL);
}

void load_system_state()
{
  system_state_t newstate = g_system_state;
  unsigned bytes_read;
  FIL file;
  f_open(&file, "sysstate.bin", FA_READ | FA_OPEN_EXISTING);
  f_read(&file, &newstate, sizeof(newstate), &bytes_read);
  f_close(&file);
  g_system_state = newstate;
}

void save_system_state()
{
  system_state_t state = g_system_state;

  unsigned bytes_written;
  FIL file;
  f_open(&file, "sysstate.bin", FA_WRITE | FA_CREATE_ALWAYS);
  f_write(&file, &state, sizeof(state), &bytes_written);
  f_close(&file);
}

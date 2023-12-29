#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <stdlib.h>
#include <string.h>
#include "log_task.h"
#include "filesystem.h"
#include "ui_task.h"
#include "motor_sampling.h"
#include "motor_orientation.h"
#include "motor_limits.h"
#include "sensor_task.h"
#include "wheel_speed.h"
#include "bike_control_task.h"
#include "settings.h"
#include "motor_control.h"

static uint8_t g_logbuffer1[4096];
static uint8_t g_logbuffer2[4096];
static THD_WORKING_AREA(logsaverstack, 1024);
static THD_WORKING_AREA(logwriterstack, 1024);
static thread_t *g_logsaver;
static int g_fileindex = -1;
static volatile uint32_t g_log_events = 0;

#define EVENT_BUF1 1
#define EVENT_BUF2 2

void log_event(log_eventtype_t event)
{
  __atomic_or_fetch(&g_log_events, event, __ATOMIC_ACQUIRE);
}

void log_saver_thread(void *p)
{
  chRegSetThreadName("logsaver");
  
  for (;;)
  {
    eventmask_t event = chEvtWaitOne(EVENT_BUF1 | EVENT_BUF2);

    if (event & EVENT_BUF1)
    {
      int status = filesystem_write(g_system_state.sd_log_sector, g_logbuffer1, sizeof(g_logbuffer1));
      if (status > 0) g_system_state.sd_log_sector += status;
    }
    
    if (event & EVENT_BUF2)
    {
      int status = filesystem_write(g_system_state.sd_log_sector, g_logbuffer2, sizeof(g_logbuffer2));
      if (status > 0) g_system_state.sd_log_sector += status;
    }
  }
}

static uint32_t compute_checksum(eventlog_store_t *entry)
{
  uint32_t checksum = 0xAAAA;
  for (int i = 1; i < 64; i++)
  {
    checksum ^= entry->raw[i];
    checksum ^= checksum << 13;
	  checksum ^= checksum >> 17;
	  checksum ^= checksum << 5;
  }

  return checksum;
}

void log_writer_thread(void *p)
{
  int write_next = EVENT_BUF1;
  int writeptr = 0;
  
  chRegSetThreadName("logwriter");
  chThdSleepMilliseconds(2000);
  
  int prev_distance = 0;
  systime_t prev_time = 0;

  int log_div = 0;
  
  for (;;)
  {
    chThdSleepMilliseconds(50);
    
    int log_interval = (motor_orientation_get_rpm() == 0) ? 10 : 1;

    log_div++;
    if (log_div >= log_interval)
    {
      log_div = 0;

      eventlog_store_t logentry = {};

      logentry.log.systime = chVTGetSystemTime();
      logentry.log.alltime_distance_m = g_system_state.alltime_distance_m;
      logentry.log.trip_distance_m = wheel_speed_get_distance();
      logentry.log.wheel_velocity = wheel_speed_get_velocity();
      logentry.log.wheel_accel = wheel_speed_get_acceleration();
      logentry.log.motor_rpm = motor_orientation_get_rpm();
      logentry.log.motor_target_current = bike_control_get_motor_current() / 1000.0f;
      logentry.log.battery_voltage = get_battery_voltage_mV() / 1000.0f;
      logentry.log.battery_current = get_battery_current_mA() / 1000.0f;
      logentry.log.mosfet_temperature = get_mosfet_temperature_mC() / 1000.0f;
      logentry.log.duty_limit = motor_limits_get_fraction();
      logentry.log.hill_accel = bike_control_get_hill_accel() / 1000.0f;
      logentry.log.pedal_accel = bike_control_get_pedal_accel() / 1000.0f;
      strncpy(logentry.log.state, bike_control_get_state(), 8);
      logentry.log.motor_angle = motor_orientation_get_angle();
      logentry.log.hall_angle = motor_orientation_get_hall_angle();
      logentry.log.assist_level = ui_get_assist_level();
      logentry.log.events = __atomic_fetch_and(&g_log_events, 0, __ATOMIC_ACQUIRE);
      logentry.log.irq_time = motor_get_interrupt_time();
      logentry.log.motor_voltage = motor_get_voltage_abs();
      logentry.log.motor_current = motor_get_current_abs() / 1000.0f;

      logentry.log.checksum = compute_checksum(&logentry);

      uint8_t *dest = (write_next == EVENT_BUF1) ? g_logbuffer1 : g_logbuffer2;
      memcpy(dest + writeptr, &logentry, sizeof(logentry));
      writeptr += sizeof(logentry);
      
      if (writeptr == sizeof(g_logbuffer1))
      {
        // Buffer filled up, save it to SD card
        chEvtSignal(g_logsaver, write_next);
        write_next = (write_next == EVENT_BUF1) ? EVENT_BUF2 : EVENT_BUF1;
        writeptr = 0;
      }
    }

    // Update system state
    g_system_state.prev_voltage_mV = get_battery_voltage_mV();
    int delta_d = wheel_speed_get_distance() - prev_distance;
    g_system_state.total_distance_m += delta_d;
    g_system_state.alltime_distance_m += delta_d;
    prev_distance += delta_d;
    systime_t delta_t = chVTGetSystemTime() - prev_time;
    g_system_state.total_time_ms += ST2MS(delta_t);
    prev_time += delta_t;
    g_system_state.total_energy_mJ += ((int64_t)get_battery_voltage_mV() * get_battery_current_mA() * (int)(ST2MS(delta_t)) + 500000) / 1000000;
  }
}

void start_log()
{
  g_logsaver = chThdCreateStatic(logsaverstack, sizeof(logsaverstack), NORMALPRIO + 1, log_saver_thread, NULL);
  chThdCreateStatic(logwriterstack, sizeof(logwriterstack), NORMALPRIO + 2, log_writer_thread, NULL);
}

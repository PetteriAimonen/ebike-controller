#pragma once

#include <stdint.h>

void start_log();
int log_get_fileindex();

// This struct is stored in a file and keeps e.g. battery status
// over reboots.
typedef struct
{
  int prev_voltage_mV;
  int total_energy_mJ;
  int total_distance_m;
  int total_time_ms;

  int accelerometer_bias_mg;
  int min_voltage_V;
  int max_motor_current_A;
  int max_battery_current_A;
  int accelerometer_invert;
} system_state_t;

extern volatile system_state_t g_system_state;

void load_system_state();
void save_system_state();

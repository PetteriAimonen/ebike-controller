/* Settings and system state */

#pragma once

#include <stdint.h>

// This struct is stored in microcontroller flash and keeps
// settings and battery status over reboots.
typedef struct
{
  uint32_t checksum; // Checksum of data
  uint32_t revision; // Update revision of struct, incremented for every save

  int prev_voltage_mV;
  int total_energy_mJ;
  int total_distance_m;
  int total_time_ms;
  int alltime_distance_m;

  int accelerometer_bias_mg;
  int min_voltage_V;
  int max_motor_current_A;
  int max_battery_current_A;
  int accelerometer_invert;
  int wheel_diameter_inch;
  int bike_weight_kg;
  int wheel_speed_ticks_per_rotation;
  int max_speed_kmh;
  int torque_N_per_A;
  int enable_boost;
  int has_pedal_sensor;
  int max_krpm;
  int accel_time;
  uint32_t sd_log_sector;
  int battery_esr_mohm;
  int max_regen_A;
  int battery_capacity_Wh;
} system_state_t;

// Constant size struct with padding for flash storage
typedef union {
  system_state_t state;
  uint8_t raw[128];
} system_state_store_t;
_Static_assert(sizeof(system_state_store_t) == 128);

extern volatile system_state_t g_system_state;

void load_system_state();
void save_system_state();

// Returns remaining battery percent
static int battery_percent()
{
  int Wh = g_system_state.total_energy_mJ / 3600000;
  return 100 - Wh * 100 / g_system_state.battery_capacity_Wh;
}

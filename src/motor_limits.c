#include "motor_limits.h"
#include "motor_config.h"
#include "motor_sampling.h"
#include "motor_orientation.h"
#include "wheel_speed.h"
#include "settings.h"
#include "log_task.h"

static float g_motor_limit_instant = 1.0f;
static float g_motor_limit_filtered = 1.0f;

static void apply_limit(float *fraction, int limitA, int limitB, int value)
{
  float max;
  if (value <= limitA)
  {
    max = 1.0f;
  }
  else if (value >= limitB)
  {
    max = 0.0f;
  }
  else
  {
    max = 1.0f - (float)(value - limitA) / (limitB - limitA);
  }
  
  if (*fraction > max)
    *fraction = max;
}

void motor_limits_update()
{
  float fraction = 1.0f;
  
  apply_limit(&fraction, g_system_state.max_krpm * 1000, g_system_state.max_krpm * 1200, motor_orientation_get_rpm());

  if (wheel_speed_get_velocity() < ACCEL_LIMIT_SPEED_M_S)
  {
    // Limit acceleration when wheel is not yet spinning fast
    int max_accel = g_system_state.max_krpm * 1000 / g_system_state.accel_time;
    apply_limit(&fraction, max_accel, max_accel * 2, motor_orientation_get_acceleration());
  }

//   apply_limit(&max_duty, MOTOR_MAX_TEMP_A, MOTOR_MAX_TEMP_B, get_motor_temperature_mC());
  apply_limit(&fraction, MOSFET_MAX_TEMP_A, MOSFET_MAX_TEMP_B, get_mosfet_temperature_mC());
  apply_limit(&fraction, -g_system_state.min_voltage_V * 1000, -(g_system_state.min_voltage_V - 3) * 1000, -get_battery_voltage_mV());
  apply_limit(&fraction, g_system_state.max_battery_current_A * 1000, (g_system_state.max_battery_current_A + 2) * 1000, get_battery_current_mA());
  
  float decay; 
  
  if (fraction < g_motor_limit_filtered)
  {
    decay = 1.0f / (CONTROL_FREQ * DUTY_LIMIT_FILTER_TRIP_S);
  }
  else
  {
    decay = 1.0f / (CONTROL_FREQ * DUTY_LIMIT_FILTER_RELAX_S);
  }

  g_motor_limit_instant = fraction;
  g_motor_limit_filtered += (fraction - g_motor_limit_filtered) * decay;

  // if (fraction > g_motor_limit_filtered)
  //   g_motor_limit_filtered += decay;
  // else if (fraction < g_motor_limit_filtered)
  //   g_motor_limit_filtered -= decay;

  if (g_motor_limit_filtered < 0.0f)
    g_motor_limit_filtered = 0.0f;
  
  if (g_motor_limit_filtered > 1.0f)
    g_motor_limit_filtered = 1.0f;
}

float motor_limits_get_fraction()
{
  // return (g_motor_limit_filtered * 3 + g_motor_limit_instant) / 4;
  return g_motor_limit_filtered;
}

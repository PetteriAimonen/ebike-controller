#include "motor_limits.h"
#include "motor_config.h"
#include "motor_sampling.h"
#include "motor_orientation.h"
#include "log_task.h"

static float g_max_duty_filtered = PWM_MAX_DUTY;

static void apply_limit(int *max_duty, int limitA, int limitB, int value)
{
  int max;
  if (value <= limitA)
  {
    max = PWM_MAX_DUTY;
  }
  else if (value >= limitB)
  {
    max = 0;
  }
  else
  {
    max = PWM_MAX_DUTY * (value - limitA) / (limitB - limitA);
  }
  
  if (*max_duty > max)
    *max_duty = max;
}

void motor_limits_update_max_duty()
{
  int max_duty = PWM_MAX_DUTY;
  
  apply_limit(&max_duty, MOTOR_MAX_RPM_A, MOTOR_MAX_RPM_B, motor_orientation_get_rpm());
//   apply_limit(&max_duty, MOTOR_MAX_TEMP_A, MOTOR_MAX_TEMP_B, get_motor_temperature_mC());
  apply_limit(&max_duty, MOSFET_MAX_TEMP_A, MOSFET_MAX_TEMP_B, get_mosfet_temperature_mC());
  apply_limit(&max_duty, -g_system_state.min_voltage_V * 1000, -(g_system_state.min_voltage_V - 3) * 1000, -get_battery_voltage_mV());
  apply_limit(&max_duty, g_system_state.max_battery_current_A * 1000, (g_system_state.max_battery_current_A + 2) * 1000, get_battery_current_mA());
  
  float decay = (float)PWM_MAX_DUTY / (CONTROL_FREQ * DUTY_LIMIT_FILTER_S);
  
  if (max_duty > (int)g_max_duty_filtered)
    g_max_duty_filtered += decay;
  else if (max_duty < (int)g_max_duty_filtered)
    g_max_duty_filtered -= decay;
}

int motor_limits_get_max_duty()
{
  return 500;
  //return (int)g_max_duty_filtered;
}

#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <stdlib.h>
#include <stdbool.h>
#include "debug.h"
#include "wheel_speed.h"
#include "motor_config.h"
#include "settings.h"

static int g_wheel_tickcount;
static int g_wheel_sensor_debounce;
static int g_time_since_rising;
static int g_previous_interval;
static float g_velocity_filtered;
static bool g_previous_state;
static int g_previous_interval_history[6];

static float meters_per_tick()
{
  return g_system_state.wheel_diameter_inch * (0.0254f * 3.1415f) / g_system_state.wheel_speed_ticks_per_rotation;
}

float interval_to_velocity(int interval)
{
  if (interval == 0)
    return 0.0f;
  
  float interval_s = interval / (float)CONTROL_FREQ;
  float speed = meters_per_tick() / interval_s;
  
  return speed;
}

float wheel_speed_get_velocity()
{
  int interval = g_previous_interval;
  if (g_time_since_rising > interval)
    interval = g_time_since_rising;
  
  return interval_to_velocity(interval);
}

float wheel_speed_get_acceleration()
{
  if (g_system_state.wheel_speed_ticks_per_rotation < 6)
  {
    // Can't get reliable acceleration with this few ticks
    return 0.0f;
  }

  float sum_accel = 0.0f;
  float min = 99999.0f;
  float max = -99999.0f;
  for (int i = 0; i < 5; i++)
  {
    float speed1 = interval_to_velocity(g_previous_interval_history[i]);
    float speed2 = interval_to_velocity(g_previous_interval_history[i+1]);
    float time = (g_previous_interval_history[i] + g_previous_interval_history[i + 1]) / (2.0f * CONTROL_FREQ);
    float accel = (time > 0.01f) ? ((speed1 - speed2) / time) : 0.0f;
    sum_accel += accel;

    if (min > accel) min = accel;
    if (max < accel) max = accel;
  }
  
  // Discard min and max values for outliers
  sum_accel -= min;
  sum_accel -= max;

  return sum_accel / 5.0f;
}

int wheel_speed_get_distance()
{
  return g_wheel_tickcount * meters_per_tick();
}

int wheel_speed_get_tickcount()
{
  return g_wheel_tickcount;
}

void wheel_speed_update()
{
  bool state = palReadPad(GPIOC, GPIOC_WHEEL_SPEED);
  const int threshold = 25;
  
  if (state && g_wheel_sensor_debounce < threshold)
  {
    g_wheel_sensor_debounce++;
  }
  else if (!state && g_wheel_sensor_debounce > -threshold)
  {
    g_wheel_sensor_debounce--;
  }
  
  if (g_wheel_sensor_debounce >= threshold && !g_previous_state)
  {
    // Rising edge
    for (int i = 5; i > 0; i--)
    {
      g_previous_interval_history[i] = g_previous_interval_history[i-1];
    }
    
    g_previous_interval_history[0] = g_previous_interval = g_time_since_rising;
    g_wheel_tickcount++;
    g_time_since_rising = 0;
    g_previous_state = true;
  }
  else if (g_wheel_sensor_debounce <= -threshold && g_previous_state)
  {
    // Falling edge
    g_previous_state = false;
  }

  g_time_since_rising++;
}




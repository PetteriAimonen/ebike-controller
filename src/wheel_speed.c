#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <stdlib.h>
#include <stdbool.h>
#include "debug.h"
#include "wheel_speed.h"
#include "motor_config.h"

static int g_wheel_tickcount;
static int g_time_since_rising;
static int g_time_since_falling;
static int g_previous_interval;
static float g_velocity_filtered;
static bool g_previous_state;
static int g_previous_interval_history[6];

float interval_to_velocity(int interval)
{
  if (interval == 0)
    return 0.0f;
  
  float interval_s = interval / (float)CONTROL_FREQ;
  float speed = WHEEL_SPEED_STEP / interval_s;
  
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
  float sum_accel = 0.0f;
  for (int i = 0; i < 5; i++)
  {
    float speed1 = interval_to_velocity(g_previous_interval_history[i]);
    float speed2 = interval_to_velocity(g_previous_interval_history[i+1]);
    float time = (g_previous_interval_history[i] + g_previous_interval_history[i + 1]) / (2.0f * CONTROL_FREQ);
    float accel = (time > 0.01f) ? ((speed1 - speed2) / time) : 0.0f;
    sum_accel += accel;
  }
  
  return sum_accel / 5.0f;
}

int wheel_speed_get_distance()
{
  return g_wheel_tickcount * WHEEL_SPEED_STEP;
}

int wheel_speed_get_tickcount()
{
  return g_wheel_tickcount;
}

void wheel_speed_update()
{
  bool state = palReadPad(GPIOC, GPIOC_WHEEL_SPEED);
  
  if (state && !g_previous_state)
  {
    if (g_time_since_falling > 10) // Debounce
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
  }
  else if (!state && g_previous_state)
  {
    if (g_time_since_rising > 10)
    {
      // Falling edge
      g_time_since_falling = 0;
      g_previous_state = false;
    }
  }
  
  g_time_since_rising++;
  g_time_since_falling++;
}




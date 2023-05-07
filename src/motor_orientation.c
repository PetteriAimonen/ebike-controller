#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <stdlib.h>
#include "debug.h"
#include "motor_config.h"
#include "motor_orientation.h"

static uint32_t g_motor_total_steps;
static int g_motor_prev_sector;
static uint32_t g_motor_timestamp;
static uint32_t g_motor_sector_times[6];
static uint32_t g_motor_sector_counts[6];
static int g_hall_error_count;
static float g_motor_filtered_rpm;

// Table index is H3 | H2 | H1, value is sector 1-6.
// Typical rotation is:
// Hall     Sector
// 001=1    1
// 011=3    2
// 010=2    3
// 110=6    4
// 100=4    5
// 101=5    6
//const int g_hall_table[8] = {-2, 0, 2, 1, 4, 5, 3, -1};
const int g_hall_table[8] = {-2, 5, 3, 4, 1, 0, 2, -1};

static int angle_diff(int a, int b)
{
  int diff = a - b;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return diff;
}

int motor_orientation_get_hall_sector()
{
  uint32_t hall_state =
    (palReadPad(GPIOB, GPIOB_HALL_1) ? 1 : 0) |
    (palReadPad(GPIOB, GPIOB_HALL_2) ? 2 : 0) |
    (palReadPad(GPIOC, GPIOC_HALL_3) ? 4 : 0);
  
  return g_hall_table[hall_state];
}

static int sector_to_angle(float sector)
{
  int hall_angle = sector * 60;
  hall_angle = (hall_angle + HALL_OFFSET + 720) % 360;
  return hall_angle;
}

extern bool g_have_motor;

void motor_orientation_update()
{
  g_motor_timestamp++;

  int sector = motor_orientation_get_hall_sector();
  
  if (sector < 0)
  {
    g_hall_error_count++;
    
    if (g_hall_error_count > 500)
    {
        g_motor_filtered_rpm = 0;
    }
    return;
  }
  else
  {
    g_hall_error_count = 0;
  }
  
  int delta = sector - g_motor_prev_sector;
  if (delta != 0)
  {
    g_motor_prev_sector = sector;

    if (delta > 3) delta -= 6;
    if (delta <= -3) delta += 6;
    g_motor_total_steps += delta;

    g_motor_sector_times[sector] = g_motor_timestamp;
    g_motor_sector_counts[sector] = g_motor_total_steps;
  }

  // Filter the motor RPM estimate for less speed critical uses
  float rpm = motor_orientation_get_fast_rpm();
  float decay = 1.0f / (MOTOR_FILTER_TIME_S * CONTROL_FREQ);
  g_motor_filtered_rpm = g_motor_filtered_rpm * (1 - decay) + rpm * decay;
}

// Based on g_motor_sector_times, fit a linear function:
// f(t) = a * t + b
// which gives the motor sector as function of t, which is
// the time delta relative to reftime.
static inline void motor_angle_linear_fit(float *a, float *b, uint32_t reftime)
{
  // Simple linear regression with
  // X = time delta, Y = motor sector
  uint32_t refsteps = g_motor_sector_counts[0];
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_x2 = 0.0f;
  float sum_y2 = 0.0f;
  float sum_xy = 0.0f;
  for (int i = 0; i < 6; i++)
  {
    float x = (int32_t)(g_motor_sector_times[i] - reftime);
    float y = (int32_t)(g_motor_sector_counts[i] - refsteps);
    sum_x += x;
    sum_y += y;
    sum_x2 += x * x;
    sum_y2 += y * y;
    sum_xy += x * y;
  }

  float x_mean = sum_x / 6;
  float y_mean = sum_y / 6;
  float x_var = sum_x2 / 6 - x_mean * x_mean;
  float cov = sum_xy / 6 - x_mean * y_mean;
  *a = cov / x_var;
  *b = y_mean - (*a) * x_mean;
}

bool motor_spinning()
{
  uint32_t last_time = g_motor_sector_times[g_motor_prev_sector];
  uint32_t max_ticks_per_rotation = (60 * CONTROL_FREQ) / CTRL_MIN_RPM;
  return (uint32_t)(g_motor_timestamp - last_time) < max_ticks_per_rotation;
}

bool motor_orientation_in_sync()
{
  // Check that all sectors have received timestamps recently
  uint32_t max_ticks_per_rotation = (60 * CONTROL_FREQ) / CTRL_MIN_RPM;
  for (int i = 0; i < 6; i++)
  {
    uint32_t delta = (uint32_t)(g_motor_timestamp - g_motor_sector_times[i]);
    if (delta > max_ticks_per_rotation) return false;
  }
  
  return true;
}

int motor_orientation_get_angle()
{
  if (!motor_orientation_in_sync())
  {
    return motor_orientation_get_hall_angle();
  }

  float a, b;
  uint32_t t = g_motor_timestamp;
  motor_angle_linear_fit(&a, &b, t);

  int hall_angle = motor_orientation_get_hall_angle();
  float limit = CTRL_MAX_RPM * 6 / (60.0f * CONTROL_FREQ);
  if (a > -limit && a < limit && b > -6 && b < 12)
  {
    int angle = sector_to_angle(b);
    int diff = angle_diff(angle, hall_angle);
    if (diff > 40)
      angle = hall_angle + 40;
    else if (diff < -40)
      angle = hall_angle - 40;
    
    if (angle < 0) angle += 360;
    if (angle >= 360) angle -= 360;
    
    return angle;
  }
  else
  {
    // Linear fit is clearly wrong
    return hall_angle;
  }
}

int motor_orientation_get_hall_angle()
{
  return sector_to_angle(g_motor_prev_sector + 0.5f);
}

int motor_orientation_get_fast_rpm()
{
  if (!motor_spinning())
  {
    return 0;
  }

  float a, b;
  uint32_t t = g_motor_timestamp;
  motor_angle_linear_fit(&a, &b, t);
  float deg_per_sector = 60.0f;
  float rpm = a * (deg_per_sector * 60.0f * CONTROL_FREQ / 360.0f);

  if (rpm > -CTRL_MAX_RPM && rpm < CTRL_MAX_RPM)
  {
    return rpm;
  }
  else
  {
    return 0;
  }
}

int motor_orientation_get_rpm()
{
  if (!motor_spinning())
  {
    return 0;
  }

  return (int)g_motor_filtered_rpm;
}

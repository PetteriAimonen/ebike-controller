#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <stdlib.h>
#include <math.h>
#include "debug.h"
#include "log_task.h"
#include "settings.h"
#include "motor_config.h"
#include "motor_orientation.h"

static struct {
  uint32_t tickcount;       // Running tick count incremented at CONTROL_FREQ
  uint32_t stepcount;       // Running sector count (increments or decrements)

  int prev_sector;          // Previous accepted hall sector value
  uint32_t prev_time;       // Time when sector last changed
  uint32_t update_time;     // Time when the new sector was accepted to prev_sector
  int prev_direction;       // Rotation direction at last sector change
  int pending_sector;       // Current hall sector value pending glitch filter
  uint32_t pending_time;    // Time when pending_sector last changed

  uint32_t sector_ticks[6]; // Tick count when sector was last entered
  uint32_t sector_steps[6]; // Step count when sector was last entered

  float filter_rpm;         // Filtered motor speed estimate
  float acceleration;       // Acceleration estimate (RPM/s)

  float filter_angle;       // Filtered orientation estimate
  float filter_lag;         // Lag of the orientation filter

  uint32_t reversal_steps;  // Step count when rotation direction changed
} g_hall;

// From main.c, whether motor is connected
extern bool g_have_motor;

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

static int wrap_angle(int a)
{
  return (a + 3600) % 360;
}

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
  return wrap_angle((int)(sector * 60) + HALL_OFFSET);
}

void motor_orientation_update()
{
  g_hall.tickcount++;

  // Check if hall input values have changed
  int sector = motor_orientation_get_hall_sector();
  if (sector != g_hall.pending_sector)
  {
    g_hall.pending_sector = sector;
    g_hall.pending_time = g_hall.tickcount;
  }

  // Detect hall sensor failure
  uint32_t ticks_pending = (uint32_t)(g_hall.tickcount - g_hall.pending_time);
  if (g_hall.pending_sector < 0 && ticks_pending > HALL_TIMEOUT)
  {
    g_hall.filter_rpm = 0;
    g_hall.reversal_steps = g_hall.stepcount;
    log_event(EVENT_HALL_INVALID_LONG);
  }

  if (g_hall.pending_sector < 0)
  {
    log_event(EVENT_HALL_INVALID_SHORT);
  }

  // Filter for short glitches and detect sector change
  int expected_sector = (g_hall.prev_sector + g_hall.prev_direction + 6) % 6;
  if (g_hall.pending_sector >= 0 &&
      g_hall.pending_sector != g_hall.prev_sector &&
      (ticks_pending > HALL_FILTER || g_hall.pending_sector == expected_sector))
  {
    // Figure out number of steps between sectors
    int delta = g_hall.pending_sector - g_hall.prev_sector;
    if (delta < -3) delta += 6;
    if (delta > 3) delta -= 6;

    // Check rotation direction and the angle at sector boundary
    if (delta > 0)
    {
      if (g_hall.prev_direction < 0)
      {
        log_event(EVENT_HALL_REVERSAL);
        g_hall.reversal_steps = g_hall.stepcount;
      }

      g_hall.prev_direction = 1;
    }
    else
    {
      if (g_hall.prev_direction > 0)
      {
        log_event(EVENT_HALL_REVERSAL);
        g_hall.reversal_steps = g_hall.stepcount;
      }

      g_hall.prev_direction = -1;
    }
    
    // Update state variables
    g_hall.update_time = g_hall.tickcount;
    g_hall.prev_sector = g_hall.pending_sector;
    g_hall.prev_time = g_hall.pending_time;
    g_hall.stepcount += delta;

    g_hall.sector_ticks[g_hall.prev_sector] = g_hall.pending_time;
    g_hall.sector_steps[g_hall.prev_sector] = g_hall.stepcount;
  }

  // Filter the angle estimate to reduce noise
  float angle_decay = 0.2f;
  int fast_angle = motor_orientation_get_angle_fast();

  if (g_hall.filter_rpm == 0 || g_hall.filter_angle != g_hall.filter_angle)
  {
    g_hall.filter_angle = fast_angle;
  }
  else
  {
    int delta_angle = angle_diff(fast_angle, (int)g_hall.filter_angle);
    g_hall.filter_angle += delta_angle * angle_decay;
    g_hall.filter_lag += (delta_angle - g_hall.filter_lag) * angle_decay;
    if (g_hall.filter_angle > 360.0f) g_hall.filter_angle -= 360.0f;
    if (g_hall.filter_angle < 0.0f) g_hall.filter_angle += 360.0f;
  }

  // Filter the motor RPM estimate for less speed critical uses
  float rpm = motor_orientation_get_fast_rpm();
  int filter_ticks = (int)(MOTOR_RPM_FILTER_SECTORS * CONTROL_FREQ * 60 / (6 * rpm));
  if (filter_ticks > MOTOR_RPM_FILTER_TIME_MAX * CONTROL_FREQ) filter_ticks = MOTOR_RPM_FILTER_TIME_MAX * CONTROL_FREQ;
  if (filter_ticks < MOTOR_RPM_FILTER_TIME_MIN * CONTROL_FREQ) filter_ticks = MOTOR_RPM_FILTER_TIME_MIN * CONTROL_FREQ;
  float rpm_decay = 1.0f / filter_ticks;
  float rpm_delta = (rpm - g_hall.filter_rpm) * rpm_decay;
  g_hall.filter_rpm += rpm_delta;

  // Estimate acceleration
  float accel = rpm_delta * CONTROL_FREQ;
  g_hall.acceleration += (accel - g_hall.acceleration) * rpm_decay;
}

// Based on g_motor_sector_times, fit a linear function:
// f(t) = a * t + b
// which gives the motor sector as function of t, which is
// the time delta relative to reftime.
static inline void motor_angle_linear_fit(float *a, float *b, uint32_t reftime)
{
  // Simple linear regression with
  // X = time delta, Y = motor sector
  uint32_t refsteps = g_hall.sector_steps[0];
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_x2 = 0.0f;
  float sum_y2 = 0.0f;
  float sum_xy = 0.0f;
  for (int i = 0; i < 6; i++)
  {
    float x = (int32_t)(g_hall.sector_ticks[i] - reftime);
    float y = (int32_t)(g_hall.sector_steps[i] - refsteps);
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
  uint32_t max_ticks_per_sector = (60 * CONTROL_FREQ) / (6 * CTRL_MIN_RPM);
  return (uint32_t)(g_hall.tickcount - g_hall.prev_time) < max_ticks_per_sector;
}

bool motor_orientation_in_sync()
{
  // Check that more than 6 steps have passed since start or last reversal
  uint32_t steps_since_reversal = g_hall.stepcount - g_hall.reversal_steps;
  if (steps_since_reversal <= 6) return false;

  // Check that all sectors have received timestamps recently
  uint32_t max_ticks_per_rotation = (60 * CONTROL_FREQ) / CTRL_MIN_RPM;
  for (int i = 0; i < 6; i++)
  {
    uint32_t delta = (uint32_t)(g_hall.tickcount - g_hall.sector_ticks[i]);
    if (delta > max_ticks_per_rotation) return false;
  }
  
  return true;
}

int motor_orientation_get_angle_fast()
{
  // Skip interpolation if we are not synchronized to rotation yet
  if (!motor_orientation_in_sync())
  {
    log_event(EVENT_HALL_UNSYNCED1);
    return motor_orientation_get_hall_angle();
  }

  float a, b;
  uint32_t t = g_hall.tickcount;
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
    
    return wrap_angle(angle);
  }
  else
  {
    // Linear fit is clearly wrong
    log_event(EVENT_HALL_UNSYNCED2);
    return hall_angle;
  }
}

int motor_orientation_get_angle()
{
  return wrap_angle((int)(g_hall.filter_angle + g_hall.filter_lag));
}

int motor_orientation_get_hall_angle()
{
  return sector_to_angle(g_hall.prev_sector + 0.5f);
}

int motor_orientation_get_fast_rpm()
{
  if (!motor_spinning())
  {
    return 0;
  }

  float a, b;
  uint32_t t = g_hall.tickcount;
  motor_angle_linear_fit(&a, &b, t);
  float deg_per_sector = 60.0f;
  float rpm = a * (deg_per_sector * 60.0f * CONTROL_FREQ / 360.0f);

  if (rpm > -CTRL_MAX_RPM && rpm < CTRL_MAX_RPM)
  {
    return rpm;
  }
  else
  {
    log_event(EVENT_HALL_UNSYNCED3);
    return 0;
  }
}

int motor_orientation_get_rpm()
{
  if (!motor_spinning())
  {
    return 0;
  }

  return (int)g_hall.filter_rpm;
}

int motor_orientation_get_acceleration()
{
  return (int)g_hall.acceleration;
}

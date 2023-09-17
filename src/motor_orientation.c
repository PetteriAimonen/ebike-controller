#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <stdlib.h>
#include <math.h>
#include "debug.h"
#include "settings.h"
#include "motor_config.h"
#include "motor_orientation.h"

static struct {
  uint32_t tickcount;       // Running tick count incremented at CONTROL_FREQ
  int prev_sector;          // Previous accepted hall sector value
  uint32_t prev_time;       // Time when sector last changed
  uint32_t update_time;     // Time when the new sector was accepted to prev_sector
  int prev_angle;           // Angle at previous sector boundary
  int prev_estimate;        // Angle estimate in effect at update_time
  int prev_direction;       // Rotation direction at last sector change
  int pending_sector;       // Current hall sector value pending glitch filter
  uint32_t pending_time;    // Time when pending_sector last changed

  uint32_t last_reversal;   // Last time motor direction changed

  bool valid;               // True if motor is rotating steadily

  int ticks_per_sector;     // Ticks elapsed during last hall sector
  int ticks_per_sector_2;   // Ticks elapsed during second to last sector

  float filter_rpm;         // Filtered motor speed estimate
  float acceleration;       // Acceleration estimate (RPM/s)
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

static int sector_to_angle(int sector)
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
    g_hall.valid = false;
  }

  // Filter for short glitches and detect sector change
  int expected_sector = (g_hall.prev_sector + g_hall.prev_direction + 6) % 6;
  if (g_hall.pending_sector >= 0 &&
      g_hall.pending_sector != g_hall.prev_sector &&
      (ticks_pending > HALL_FILTER || g_hall.pending_sector == expected_sector))
  {
    g_hall.prev_estimate = motor_orientation_get_angle();
    
    // Figure out number of steps between sectors
    int delta = g_hall.pending_sector - g_hall.prev_sector;
    if (delta < -3) delta += 6;
    if (delta > 3) delta -= 6;

    // Check rotation direction and the angle at sector boundary
    if (delta > 0)
    {
      g_hall.prev_direction = 1;
      g_hall.prev_angle = sector_to_angle(g_hall.pending_sector);
    }
    else
    {
      if (g_hall.prev_direction > 0 && g_hall.valid)
      {
        g_hall.last_reversal = g_hall.tickcount;
      }

      g_hall.prev_direction = -1;
      g_hall.prev_angle = sector_to_angle(g_hall.prev_sector);
    }
    
    // Update state variables
    g_hall.update_time = g_hall.tickcount;
    g_hall.ticks_per_sector_2 = g_hall.ticks_per_sector;
    g_hall.ticks_per_sector = g_hall.pending_time - g_hall.prev_time;
    g_hall.prev_sector = g_hall.pending_sector;
    g_hall.prev_time = g_hall.pending_time;
  }

  // Detect when we are synced to rotation
  uint32_t max_ticks_per_sector = (60 * CONTROL_FREQ) / (6 * CTRL_MIN_RPM);
  uint32_t min_ticks_per_sector = (60 * CONTROL_FREQ) / (6 * CTRL_MAX_RPM);
  uint32_t time_since_reversal = g_hall.tickcount - g_hall.last_reversal;
  if (ticks_pending <= max_ticks_per_sector &&
      g_hall.ticks_per_sector >= min_ticks_per_sector &&
      g_hall.ticks_per_sector <= max_ticks_per_sector &&
      g_hall.ticks_per_sector <= g_hall.ticks_per_sector_2 * 2 &&
      g_hall.ticks_per_sector >= g_hall.ticks_per_sector_2 / 2/* &&
      time_since_reversal > CONTROL_FREQ * HALL_BACKOFF_MS / 1000 */)
  {
    g_hall.valid = true;
  }
  else
  {
    g_hall.valid = false;
  }

  // Filter the motor RPM estimate for less speed critical uses
  float rpm = motor_orientation_get_fast_rpm();
  int filter_ticks = g_hall.ticks_per_sector * MOTOR_RPM_FILTER_SECTORS;
  if (filter_ticks > MOTOR_RPM_FILTER_TIME_MAX * CONTROL_FREQ) filter_ticks = MOTOR_RPM_FILTER_TIME_MAX * CONTROL_FREQ;
  if (filter_ticks < MOTOR_RPM_FILTER_TIME_MIN * CONTROL_FREQ) filter_ticks = MOTOR_RPM_FILTER_TIME_MIN * CONTROL_FREQ;
  float rpm_decay = 1.0f / filter_ticks;
  float rpm_delta = (rpm - g_hall.filter_rpm) * rpm_decay;
  g_hall.filter_rpm += rpm_delta;

  // Estimate acceleration
  float accel = rpm_delta * CONTROL_FREQ;
  g_hall.acceleration += (accel - g_hall.acceleration) * rpm_decay;
}

int motor_orientation_get_angle()
{
  // Skip interpolation if we are not synchronized to rotation yet
  if (!g_hall.valid)
  {
    return motor_orientation_get_hall_angle();
  }

  // At each sector change, we get a new estimate for motor speed
  // and a new instant angle value. To avoid any sudden jumps,
  // linearly interpolate between the angle value reported at the
  // previous update and our best estimate for the time when current
  // sector will end.
  int ticks_per_sector = (g_hall.ticks_per_sector + g_hall.ticks_per_sector_2) / 2;
  uint32_t sector_end_time = g_hall.prev_time + ticks_per_sector;
  int32_t time_to_sector_end = (int32_t)(sector_end_time - g_hall.tickcount);
  int32_t sector_total_time = (int32_t)(sector_end_time - g_hall.update_time);
  
  if (time_to_sector_end >= 0)
  {
    // Interpolate from previous estimate to new estimate
    int next_angle = g_hall.prev_angle + g_hall.prev_direction * 60;
    int delta = angle_diff(g_hall.prev_estimate, next_angle);
    int delta_to_apply = delta * time_to_sector_end / sector_total_time;
    return wrap_angle(next_angle + delta_to_apply);
  }
  else if (time_to_sector_end > -ticks_per_sector / 2)
  {
    // Extrapolate up to 30 deg into next sector
    int next_angle = g_hall.prev_angle + g_hall.prev_direction * 60;
    int extra_angle = 60 * (-time_to_sector_end) / ticks_per_sector;
    return wrap_angle(next_angle + extra_angle * g_hall.prev_direction);
  }
  else
  {
    // Hold the estimate, motor is probably stalled
    int next_angle = g_hall.prev_angle + g_hall.prev_direction * 60;
    return wrap_angle(next_angle + 30 * g_hall.prev_direction);
  }
}

int motor_orientation_get_hall_angle()
{
  return wrap_angle(sector_to_angle(g_hall.prev_sector) + 30);
}

int motor_orientation_get_fast_rpm()
{
  uint32_t max_ticks_per_sector = (60 * CONTROL_FREQ) / CTRL_MIN_RPM;
  uint32_t latest_ticks = (g_hall.tickcount - g_hall.update_time);
  int ticks_per_sector = (g_hall.ticks_per_sector + g_hall.ticks_per_sector_2) / 2;
  
  if (latest_ticks > max_ticks_per_sector)
  {
    return 0;
  }
  else if (latest_ticks > ticks_per_sector * 3 / 2)
  {
    ticks_per_sector = latest_ticks;
  }

  float rpm = 10.0f * CONTROL_FREQ / ticks_per_sector * g_hall.prev_direction;

  if (rpm < -CTRL_MAX_RPM)
  {
    return -CTRL_MAX_RPM;
  }
  else if (rpm > CTRL_MAX_RPM)
  {
    return CTRL_MAX_RPM;
  }
  else
  {
    return rpm;
  }
}

int motor_orientation_get_rpm()
{
  return (int)g_hall.filter_rpm;
}

int motor_orientation_get_acceleration()
{
  return (int)g_hall.acceleration;
}

bool motor_orientation_in_sync()
{
  return g_hall.valid;
}
#include <ch.h>
#include <hal.h>
#include <math.h>
#include "bike_control_task.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor_orientation.h"
#include "wheel_speed.h"
#include "sensor_task.h"
#include "ui_task.h"
#include "log_task.h"

static THD_WORKING_AREA(bikestack, 1024);

/* State machine:
 * 
 *  v----------------------------.
 * BOOT -----> BRAKING -----> WAITMOVE ------> POWERED
 *                ^-------------------------------'
 */

static enum { STATE_BOOT = 0, STATE_BRAKING, STATE_WAITMOVE, STATE_POWERED } g_bike_state;
static float g_acceleration; // In m/s^2 along bike axis
static float g_motor_current;
static systime_t g_brake_time;
static int g_brake_pos;

const char* bike_control_get_state()
{
  const char *states[] = {"BOOT", "MOVING", "BRAKING", "POWERED"};
  return states[g_bike_state];
}

int bike_control_get_acceleration()
{
  return 1000 * g_acceleration;
}

int bike_control_get_acceleration_mg()
{
  return (1000.0f/9.81f) * g_acceleration;
}

int bike_control_get_motor_current()
{
  return 1000 * g_motor_current;
}

static void state_boot()
{
  g_motor_current = 0.0f;
  
  if (palReadPad(GPIOB, GPIOB_BRAKE) == 0)
  {
    g_bike_state = STATE_BRAKING;
  }
}

static void state_braking()
{
  g_motor_current = 0.0f;
  
  if (palReadPad(GPIOB, GPIOB_BRAKE) != 0)
  {
    g_bike_state = STATE_WAITMOVE;
    g_brake_time = chVTGetSystemTime();
    g_brake_pos = wheel_speed_get_tickcount();
  }
}

static void state_waitmove()
{
  if (wheel_speed_get_tickcount() - g_brake_pos >= 2)
  {
    g_bike_state = STATE_POWERED;
  }
  else if (chVTGetSystemTime() - g_brake_time > S2ST(2))
  {
    g_bike_state = STATE_BOOT;
  }
}

static void state_powered()
{
  static systime_t prev_time;
  static float max_accel;
  static systime_t max_accel_time;
  static float min_accel;
  static systime_t min_accel_time;
  static float avg_accel;
  static int stall_count;
  
  systime_t time_now = chVTGetSystemTime();
  systime_t delta = time_now - prev_time;
  float delta_s = delta / (float)(S2ST(1));
  prev_time = time_now;
  
  if (delta_s > 0.1f)
  {
    max_accel = min_accel = avg_accel = g_acceleration;
    return; // First timestep
  }
  
  float decay = delta_s / BIKE_MIN_PEDAL_INTERVAL_S;
  
  // Keep track of max and min acceleration during this pedalling period
  if (g_acceleration > max_accel)
  {
    max_accel = g_acceleration;
    max_accel_time = time_now;
  }
  else if (time_now - max_accel_time > S2ST(BIKE_MIN_PEDAL_INTERVAL_S))
  {
    max_accel = g_acceleration * decay + max_accel * (1 - decay);
  }
  
  if (g_acceleration < min_accel)
  {
    min_accel = g_acceleration;
    min_accel_time = time_now;
  }
  else if (time_now - min_accel_time > S2ST(BIKE_MIN_PEDAL_INTERVAL_S))
  {
    min_accel = g_acceleration * decay + min_accel * (1 - decay);
  }
  
  // Keep track of average acceleration
  avg_accel = g_acceleration * decay + avg_accel * (1 - decay);
  
  float wheel_accel = wheel_speed_get_acceleration();
  float velocity = wheel_speed_get_velocity();
  
  if (g_motor_current > 0 && (motor_orientation_get_rpm() < 60 || !motor_orientation_in_sync()))
  {
    stall_count++;
  }
  else
  {
    stall_count = 0;
  }

  if (stall_count > 500 || velocity < BIKE_MIN_VELOCITY)
  {
    // Wheel is stopped
    g_motor_current = 0.0f;
    g_bike_state = STATE_BOOT;
  }
  else if (velocity > BIKE_MAX_VELOCITY)
  {
    // Maximum speed
    float decay = delta_s / 2.0f;
    g_motor_current *= (1.0f - decay);
    
    if (g_motor_current < BIKE_MIN_CURRENT_A)
    {
      g_motor_current = 0;
    }
  }
  else if (min_accel < -BIKE_BRAKE_THRESHOLD_B_M_S2 || wheel_accel < -BIKE_BRAKE_THRESHOLD_M_S2)
  {
    // Soft braking
    g_motor_current = 0.0f;
    max_accel = min_accel = avg_accel = g_acceleration;
  }
  else
  {
    // Decide assist level
    float assist_flat = ui_get_assist_level() / 100.0f;
    float assist_hill = (2 * assist_flat < 1) ? (2 * assist_flat) : 1;
    float fudge = (2 * assist_flat - 0.5f) * 0.1f;
    
    float pedal_accel = (max_accel - min_accel) * 0.5f;
    float hill_accel = avg_accel - wheel_accel;
    
    float target_current = (pedal_accel * assist_flat + hill_accel * assist_hill + fudge) * BIKE_WEIGHT_KG / MOTOR_NEWTON_PER_A;
    if (target_current < BIKE_MIN_CURRENT_A) target_current = 0;

    int max_current = g_system_state.max_motor_current_A;
    if (target_current > max_current) target_current = max_current;
    
    float decay = delta_s / BIKE_TORQUE_FILTER_S;
    
    if (g_motor_current < BIKE_SOFTSTART_A)
    {
      if (g_motor_current < target_current)
      {
        g_motor_current += BIKE_SOFTSTART_A * delta_s / 0.5f;
      }
    }
    else
    {
      g_motor_current = target_current * decay + g_motor_current * (1 - decay);
    }
      
    if (g_motor_current < BIKE_MIN_CURRENT_A && target_current == 0)
    {
      g_motor_current = 0;
    }
  }
}


static void bike_control_thread(void *p)
{
  static event_listener_t el;    
  chEvtRegister(&g_sensor_data_event, &el, 0);
  
  chRegSetThreadName("bike_ctrl");
  
  bool was_stopped = false;

  for (;;)
  {
    /* Wait for a new reading from sensors */
    chEvtWaitAny(ALL_EVENTS);
    int x, y, z;
    sensors_get_accel(&x, &y, &z);
    
    z -= g_system_state.accelerometer_bias_mg;

    if (g_system_state.accelerometer_invert)
      z = -z;

    float acceleration = z * 0.00981f;
    float decay = 1.0f / 5;
    g_acceleration = acceleration * decay + g_acceleration * (1 - decay);
    
    if (palReadPad(GPIOB, GPIOB_BRAKE) == 0)
    {
      g_bike_state = STATE_BRAKING;
    }

    if (x > 700 || x < -700)
    {
      g_bike_state = STATE_BRAKING;
    }

    if (g_bike_state == STATE_BOOT)
    {
      state_boot();
    }
    else if (g_bike_state == STATE_BRAKING)
    {
      state_braking();
    }
    else if (g_bike_state == STATE_WAITMOVE)
    {
      state_waitmove();
    }
    else if (g_bike_state == STATE_POWERED)
    {
      state_powered();
    }
    
    if (g_motor_current > 0)
    {
      was_stopped = false;
      motor_run((int)(g_motor_current * 1000.0f), 0);
    }
    else if (!was_stopped)
    {
      was_stopped = true;
      motor_stop();
    }
  }
}

void start_bike_control()
{
  chThdCreateStatic(bikestack, sizeof(bikestack), NORMALPRIO + 4, bike_control_thread, NULL);
}


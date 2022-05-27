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
  const char *states[] = {"BOOT", "BRAKING", "WAITMOVE", "POWERED"};
  return states[g_bike_state];
}

static float g_accel_history[32];
static uint32_t g_accel_history_index;
static float g_prev_pedal_accel;
static float g_prev_hill_accel;
static float g_pedal_bandpass[8];

// Detect acceleration due to pedalling with a bandpass filter.
// Cycling cadence is 30-100 RPM, peak twice per cycle
// => bandpass filter 1-3 Hz at different offsets and compute amplitude
float get_pedalling_bandpass()
{
  const int filter[15] = {  1809,  134,  253,  3136,  -3360,  -9522,  1982,  13649,  1982,  -9522,  -3360,  3136,  253,  134,  1809 };

  // Bandpass filter past samples
  float bandpass = 0.0f;
  for (int i = 0; i < 15; i++)
  {
    bandpass += g_accel_history[(g_accel_history_index - i) & 31] * filter[i];
  }
  bandpass /= 32768.0f;
  
  // Calculate RMS sum over past 8 bandpass results
  float rms = bandpass * bandpass;
  for (int i = 0; i < 7; i++)
  {
    rms += g_pedal_bandpass[i] * g_pedal_bandpass[i];
    g_pedal_bandpass[i+1] = g_pedal_bandpass[i];
  }
  g_pedal_bandpass[0] = bandpass;
  
  // Calculate peak amplitude: sqrt(sum / 8) * sqrt(2)
  float amplitude = __builtin_sqrtf(rms / 4);
  return amplitude;
}

// Get average acceleration over past 1 second
// Uses low-pass filter at 0.5 Hz to filter out pedalling
float get_avg_accel()
{
  const int filter[11] = {  -53,  1806,  3004,  4560,  5757,  6206,  5757,  4560,  3004,  1806,  -53};

  float result = 0.0f;
  for (int i = 0; i < 11; i++)
  {
    result += g_accel_history[(g_accel_history_index - i) & 31] * filter[i];
  }
  
  return result / 32768.0f;
}

// Acceleration history is collected every 100 ms => samplerate 10 Hz
void update_accel_history()
{
  static systime_t prev_time = 0;
  static float accumulator = 0;
  static int count = 0;
  
  systime_t time_now = chVTGetSystemTime();
  
  count++;
  accumulator += g_acceleration;
  
  if (time_now - prev_time >= MS2ST(100))
  {
    g_accel_history_index = (g_accel_history_index + 1) & 31;
    g_accel_history[g_accel_history_index] = accumulator / count;
    accumulator = 0;
    count = 0;
    prev_time = time_now;
  
    float wheel_accel = wheel_speed_get_acceleration();
    g_prev_pedal_accel = get_pedalling_bandpass();
    g_prev_hill_accel = get_avg_accel() - (wheel_accel > 0 ? wheel_accel : 0);
  }
}

// Brake button gestures allow double-click to request extra power
static struct {systime_t start; systime_t end;} g_prev_brake_times[3];
static void update_brake_gesture()
{
  systime_t time_now = chVTGetSystemTime();
  if (palReadPad(GPIOB, GPIOB_BRAKE) == 0)
  {
    // Brake is down
    if ((time_now - g_prev_brake_times[0].end) > MS2ST(20))
    {
      // Debounce time passed, count as new one
      g_prev_brake_times[2] = g_prev_brake_times[1];
      g_prev_brake_times[1] = g_prev_brake_times[0];
      g_prev_brake_times[0].start = time_now;
    }
    g_prev_brake_times[0].end = time_now;
  }
}
static bool has_brake_doubleclick()
{
  int length1 = g_prev_brake_times[0].end - g_prev_brake_times[0].start;
  int delta = g_prev_brake_times[0].start - g_prev_brake_times[1].end;
  int length2 = g_prev_brake_times[1].end - g_prev_brake_times[1].start;
  
  return (length1 > MS2ST(50) && length1 < MS2ST(800) &&
          delta > MS2ST(50) && delta < MS2ST(800) &&
          length2 > MS2ST(50) && length2 < MS2ST(800));
}

int bike_control_get_acceleration()
{
  return 1000 * g_acceleration;
}

int bike_control_get_pedal_accel()
{
  return 1000 * g_prev_pedal_accel;
}

int bike_control_get_hill_accel()
{
  return 1000 * g_prev_hill_accel;
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
  float wheel_velocity = wheel_speed_get_velocity();
  float wheel_accel = wheel_speed_get_acceleration();

  // Detect if the wheel stops
  static int stall_count;
  if (g_motor_current > 0 && (motor_orientation_get_rpm() < 60 || !motor_orientation_in_sync()))
  {
    stall_count++;
  }
  else
  {
    stall_count = 0;
  }
  
  if (stall_count > 500 || wheel_velocity < BIKE_MIN_VELOCITY)
  {
    // Wheel is stopped
    g_motor_current = 0.0f;
    g_bike_state = STATE_BOOT;
    return;
  }
  
  // Compute timestep
  static systime_t prev_time;
  systime_t time_now = chVTGetSystemTime();
  systime_t delta = time_now - prev_time;
  if (delta > MS2ST(100)) delta = MS2ST(100);
  float delta_s = delta / (float)(S2ST(1));
  prev_time = time_now;
  
  // Lowpass filter will be applied to target current
  float target_current = 0.0f;
  float decay_time = BIKE_TORQUE_FILTER_S;
  float max_current = g_system_state.max_motor_current_A;
  
  if (wheel_velocity > BIKE_MAX_VELOCITY)
  {
    // Maximum speed
    target_current = 0.0f;
    decay_time = 2.0f;
  }
  else if (g_acceleration < -BIKE_BRAKE_THRESHOLD_B_M_S2 || wheel_accel < -BIKE_BRAKE_THRESHOLD_M_S2)
  {
    // Soft braking
    decay_time = 0.2f;
    target_current = 0.0f;
  }
  else
  {
    // Decide assist level
    float assist_flat = 0.25f;
    float assist_hill = 1.0f;
    float fudge = 0.0f;
    float min_pedal_accel = BIKE_MIN_PEDAL_ACCEL;
    if (ui_get_assist_level() <= 25)
    {
        assist_flat = 0.1f;
        assist_hill = 0.75f;
        fudge = -0.1f;
        min_pedal_accel *= 2.0f;
    }
    else if (ui_get_assist_level() >= 75)
    {
        assist_flat = 0.5f;
        assist_hill = 1.5f;
        fudge = 0.1f;
        min_pedal_accel = 0.0f;
    }
    
    if (has_brake_doubleclick() && (time_now - g_brake_time) < S2ST(10))
    {
      // Extra power when brake lever is double-clicked
      assist_hill = 1.5f;
      assist_flat = 0.5f;
      fudge = 0.5f;
      max_current += 5;
      min_pedal_accel = 0.0f;
    }
    
    
    if (g_prev_pedal_accel < min_pedal_accel)
    {
      target_current = 0;
      decay_time = 2.0f;
    }
    else
    {
      target_current = (g_prev_pedal_accel * assist_flat + g_prev_hill_accel * assist_hill + fudge) * BIKE_WEIGHT_KG / MOTOR_NEWTON_PER_A;
    }
  }
  
  if (g_motor_current < BIKE_SOFTSTART_A && g_motor_current < target_current)
  {
    g_motor_current += BIKE_SOFTSTART_A * delta_s / BIKE_SOFTSTART_S;
  }
  else if (g_motor_current < BIKE_MIN_CURRENT_A && target_current < BIKE_MIN_CURRENT_A)
  {
    g_motor_current = 0;
  }
  else
  {
    float current_decay = delta_s / decay_time;
    g_motor_current = target_current * current_decay + g_motor_current * (1 - current_decay);
  }
  
  if (g_motor_current > max_current)
  {
    g_motor_current = max_current;
  }
}


static void bike_control_thread(void *p)
{
  static event_listener_t el;    
  chEvtRegister(&g_sensor_data_event, &el, 0);
  
  chRegSetThreadName("bike_ctrl");
  
  /* Make a boot sound */
  for (int j = 0; j < 2; j++)
  {
    chThdSleepMilliseconds(100);
    for (int i = 0; i < 25; i++)
    {
      motor_run(700, 0);
      chThdSleepMilliseconds(1);
      motor_run(-700, 0);
      chThdSleepMilliseconds(1);
    }
  }
  motor_stop();
  
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
    
    update_accel_history();
    update_brake_gesture();
    
    if (palReadPad(GPIOB, GPIOB_BRAKE) == 0)
    {
      g_bike_state = STATE_BRAKING;
    }

    if (x > 500 || x < -500)
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


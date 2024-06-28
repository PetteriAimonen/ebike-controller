#include <ch.h>
#include <hal.h>
#include <math.h>
#include "bike_control_task.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor_orientation.h"
#include "motor_sampling.h"
#include "wheel_speed.h"
#include "sensor_task.h"
#include "ui_task.h"
#include "settings.h"
#include "ws2812.h"
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
  if (palReadPad(GPIOB, GPIOB_FAULT) == 0)
  {
    return "DRV_FAULT";
  }

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
  
    // Subtract wheel acceleration to counter the effect where
    // right at the start of a hill, speed usually drops a bit
    // which masks the hill from accelerometer.
    // Clamp large values (braking)
    float wheel_accel = wheel_speed_get_acceleration();
    if (wheel_accel < -0.1f) wheel_accel = -0.1f;
    if (wheel_accel > 0.1f) wheel_accel = 0.1f;
    
    g_prev_pedal_accel = get_pedalling_bandpass();
    g_prev_hill_accel = get_avg_accel() - wheel_accel;
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
  if (g_system_state.max_regen_A > 0)
  {
    float kmh = wheel_speed_get_velocity() * 3.6f;
    float regen_ratio = 0.0f;
    if (kmh < BIKE_MIN_VELOCITY * 3.6f)
    {
      regen_ratio = 0.0f;
    }
    else if (kmh < g_system_state.max_speed_kmh)
    {
      regen_ratio = 0.5f * kmh / g_system_state.max_speed_kmh;
    }
    else
    {
      regen_ratio = 0.5f + (kmh - g_system_state.max_speed_kmh) / 4.0f;
    }

    if (regen_ratio > 1.0f) regen_ratio = 1.0f;

    float regen_current = regen_ratio * g_system_state.max_regen_A;
    if (regen_current > BIKE_MIN_CURRENT_A)
    {
      g_motor_current += (-regen_current - g_motor_current) * 0.05f;
    }
    else
    {
      g_motor_current = 0.0f;
    }
  }
  else
  {
    g_motor_current = 0.0f;
  }
  
  if (palReadPad(GPIOB, GPIOB_BRAKE) != 0)
  {
    if (wheel_speed_get_velocity() < BIKE_MIN_VELOCITY)
    {
      g_bike_state = STATE_WAITMOVE;
      g_motor_current = 0.0f;
    }
    else
    {
      g_bike_state = STATE_POWERED;
    }

    g_brake_time = chVTGetSystemTime();
    g_brake_pos = wheel_speed_get_tickcount();
  }
}

static void state_waitmove()
{
  if (wheel_speed_get_tickcount() - g_brake_pos >= 2 || g_system_state.has_pedal_sensor)
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
  static float prev_current;
  static systime_t prev_time;
  systime_t time_now = chVTGetSystemTime();

  // Compute timestep
  systime_t delta = time_now - prev_time;
  if (delta > MS2ST(100)) delta = MS2ST(100);
  float delta_s = delta / (float)(S2ST(1));
  
  /*
  if (delta <= MS2ST(100) && prev_current > g_motor_current)
  {
    // Resume from previous motor power after very brief brake application
    g_motor_current = prev_current;
  }
  */

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
  
  if (stall_count > 500 ||
      (wheel_velocity < BIKE_MIN_VELOCITY && g_system_state.wheel_speed_ticks_per_rotation >= 6) ||
      motor_orientation_get_rpm() < -100)
  {
    // Wheel is stopped or rotating in reverse
    g_motor_current = 0.0f;
    g_bike_state = STATE_BOOT;
    return;
  }
  
  // Lowpass filter will be applied to target current
  float target_current = 0.0f;
  float decay_time = BIKE_TORQUE_FILTER_S;
  float max_current = g_system_state.max_motor_current_A;
  
  if (g_acceleration < -BIKE_BRAKE_THRESHOLD_B_M_S2 || wheel_accel < -BIKE_BRAKE_THRESHOLD_M_S2)
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
    
    if (has_brake_doubleclick() && (time_now - g_brake_time) < S2ST(10) && g_system_state.enable_boost)
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
      float target_accel_m_s2 = (g_prev_pedal_accel * assist_flat + g_prev_hill_accel * assist_hill + fudge);
      float target_force_N = target_accel_m_s2 * g_system_state.bike_weight_kg;
      target_current = target_force_N / g_system_state.torque_N_per_A;
    }
  }

  float speed_kmh = wheel_velocity * 3.6f;
  if (speed_kmh > g_system_state.max_speed_kmh)
  {
    // Softly limit speed within 2km/h
    float exceed = speed_kmh - g_system_state.max_speed_kmh;
    if (exceed > 2)
    {
      target_current = 0.0f;
      decay_time = 2.0f;
    }
    else
    {
      target_current *= exceed / 2.0f;
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

  prev_time = time_now;
  prev_current = g_motor_current;
}

void bike_control_update_leds()
{
  static bool was_brake;

  if (chVTGetSystemTime() < 8000)
  {
    // Let startup battery level indicator stay for a while
    return;
  }

  if (palReadPad(GPIOB, GPIOB_BRAKE) == 0)
  {
    float threshold = -BIKE_BRAKE_THRESHOLD_M_S2;
    if (!was_brake) threshold *= 2.0f;
    if (g_acceleration < threshold)
    {
      was_brake = true;
      log_event(EVENT_BRAKELIGHT);

      for (int i = 0; i < 54; i++)
      {
        // Brake lights
        ws2812_write_led(i, 255, 0, 0);
      }
    }
    else
    {
      was_brake = false;

      for (int i = 0; i < 54; i++)
      {
        // Dimmer brake lights
        ws2812_write_led(i, 64, 0, 0);
      }
    }
  }
  else
  {
    // Rear end lights
    was_brake = false;
    for (int i = 0; i < 17; i++)
    {
      if (i > 2)
      {
        ws2812_write_led(26 - i, 64, 0, 0);
        ws2812_write_led(27 + i, 64, 0, 0);
      }
      else
      {
        ws2812_write_led(26 - i, 0, 0, 0);
        ws2812_write_led(27 + i, 0, 0, 0);
      }
    }

    // Power level indicator lights
    static float actual_current_avg;
    float ratio_target = g_motor_current / g_system_state.max_motor_current_A;
    actual_current_avg += ((motor_get_current_abs() / 1000.0f) - actual_current_avg) * 0.1f;
    float ratio_actual = actual_current_avg / (g_system_state.max_motor_current_A);

    if (fabs(ratio_actual - ratio_target) < 0.1f) ratio_actual = ratio_target;

    for (int i = 0; i < 10; i++)
    {
      if (ratio_actual > i * 0.1f)
      {
        // Actual current delivered to motor
        ws2812_write_led(26 - 17 - i, 64, 32, 0);
        ws2812_write_led(27 + 17 + i, 64, 32, 0);
      }
      else if (ratio_target > i * 0.1f)
      {
        // Target current which is not achieved due to motor/battery limits
        ws2812_write_led(26 - 17 - i, 16, 8, 0);
        ws2812_write_led(27 + 17 + i, 16, 8, 0);
      }
      else
      {
        ws2812_write_led(26 - 17 - i, 0, 0, 0);
        ws2812_write_led(27 + 17 + i, 0, 0, 0);
      }
    }
  }
  
  int battery_mV = get_battery_voltage_mV();
  if (battery_mV <= g_system_state.min_voltage_V * 1000)
  {
    // Turn off some rear leds to indicate very low battery
    for (int i = 0; i <= 9; i += 2)
    {
      ws2812_write_led(26 - i, 0, 0, 0);
      ws2812_write_led(27 + i, 0, 0, 0);
    }
  }
  else if (battery_percent() < BATTERY_LEVEL_WARN)
  {
    // Turn off a few rear leds to indicate low battery
    for (int i = 0; i < 5; i += 2)
    {
      ws2812_write_led(26 - i, 0, 0, 0);
      ws2812_write_led(27 + i, 0, 0, 0);
    }
  }
}


static void bike_control_thread(void *p)
{
  static event_listener_t el;    
  chEvtRegister(&g_sensor_data_event, &el, 0);
  
  chRegSetThreadName("bike_ctrl");
  
  if (battery_percent() < BATTERY_LEVEL_WARN)
  {
    /* Sad sound for low battery */
    chThdSleepMilliseconds(100);
    for (int i = 0; i < 500/4; i++)
    {
      motor_run(2000, 0);
      chThdSleepMilliseconds(2);
      motor_run(-2000, 0);
      chThdSleepMilliseconds(2);
    }
    motor_stop();
    chThdSleepMilliseconds(100);
    for (int i = 0; i < 500/6; i++)
    {
      motor_run(2000, 0);
      chThdSleepMilliseconds(3);
      motor_run(-2000, 0);
      chThdSleepMilliseconds(3);
    }
    motor_stop();
    chThdSleepMilliseconds(100);
    for (int i = 0; i < 500/8; i++)
    {
      motor_run(2000, 0);
      chThdSleepMilliseconds(4);
      motor_run(-2000, 0);
      chThdSleepMilliseconds(4);
    }
    motor_stop();
  }
  else
  {
    /* Normal boot sound */
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
  }
  
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
      log_event(EVENT_BRAKE);
      g_bike_state = STATE_BRAKING;
    }

    if (x > 500 || x < -500)
    {
      log_event(EVENT_TILT);
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
      motor_set_regen_brake(false);
      motor_run((int)(g_motor_current * 1000.0f), 0);
    }
    else if (g_motor_current < 0)
    {
      was_stopped = false;
      motor_set_regen_brake(true);
      motor_run((int)(g_motor_current * 1000.0f), 0);
    }
    else if (!was_stopped)
    {
      was_stopped = true;
      motor_set_regen_brake(false);
      motor_stop();
    }

    bike_control_update_leds();
  }
}

void start_bike_control()
{
  chThdCreateStatic(bikestack, sizeof(bikestack), NORMALPRIO + 4, bike_control_thread, NULL);
}


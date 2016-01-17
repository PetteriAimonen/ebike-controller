#include <ch.h>
#include <hal.h>
#include "bike_control_task.h"
#include "motor_control.h"
#include "motor_config.h"
#include "sensor_task.h"

#define MOTOR_NEWTON_PER_A       5.0f
#define BIKE_MIN_WEIGHT         20.0f
#define BIKE_MAX_WEIGHT         90.0f

static volatile float g_acceleration_level;
static volatile float g_motor_current;
static volatile float g_control_I_accumulator;
static THD_WORKING_AREA(bikestack, 1024);

int bike_control_get_acceleration_level()
{
  return (int)(g_acceleration_level / 0.00981f);
}

int bike_control_get_motor_current()
{
  return (int)(g_motor_current * 1000.0f);
}

static void bike_control_thread(void *p)
{
  static event_listener_t el;    
  chEvtRegister(&g_sensor_data_event, &el, 0);
  
  chRegSetThreadName("bike_ctrl");
  
  systime_t start = chVTGetSystemTime();
  for (;;)
  {
    chEvtWaitAny(ALL_EVENTS);
    
    int x, y, z;
    sensors_get_accel(&x, &y, &z);
    
    // Calculate time delta since previous loop
    systime_t newTime = chVTGetSystemTime();
    systime_t timedelta = newTime - start;
    start = newTime;
    float delta_s = timedelta / (float)(S2ST(1));
    
    // Total acceleration along the bike axis
    // Positive = speed increasing
    float total_accel = - z * 0.00981f;
    
    // Calculate maximum amount of acceleration that might be due to our motor.
    float motor_accel = g_motor_current * MOTOR_NEWTON_PER_A / BIKE_MIN_WEIGHT;
    
    // Calculate how much acceleration is definitely due to cyclist
    float user_accel = total_accel - motor_accel;
    
    // Keep track of the maximum user acceleration within last 5 seconds
    float decay_down = delta_s / 5.0f;
    float decay_up = delta_s / 0.1f;
    if (user_accel > g_acceleration_level)
      g_acceleration_level = g_acceleration_level * (1 - decay_up) + user_accel * decay_up;
    else
      g_acceleration_level = g_acceleration_level * (1 - decay_down) + user_accel * decay_down;
    
    // Detect braking
    if (palReadPad(GPIOB, GPIOB_BRAKE) == 0)
    {
      g_acceleration_level = 0;
      g_control_I_accumulator = 0;
    }
    if (total_accel < -0.1f)
    {
      g_acceleration_level = 0;
      g_control_I_accumulator = 0;
    }
    
    if (g_acceleration_level > 0.05f)
    {
      // Decide new motor current so that total_accel
      // stays at g_acceleration_level.
      float N_error = (g_acceleration_level - total_accel) * BIKE_MAX_WEIGHT;
      float P_term = 0.1f;
      float I_term = 0.05f;
      float current = (P_term * N_error + g_control_I_accumulator) / MOTOR_NEWTON_PER_A;
      int current_mA = (int)(current * 1000.0f);
      
      if (current_mA < 100)
        current_mA = 100;
      
      if (current_mA < MAX_MOTOR_CURRENT)
      {
        g_control_I_accumulator += I_term * N_error;
      }
      else
      {
        current_mA = MAX_MOTOR_CURRENT;
      }
    
      g_motor_current = current_mA / 1000.0f;
      motor_run(current_mA, 0);
    }
    else if (g_motor_current > 0)
    {
      g_motor_current = 0.0f;
      motor_stop();
    }
  }
}

void start_bike_control()
{
  chThdCreateStatic(bikestack, sizeof(bikestack), NORMALPRIO + 4, bike_control_thread, NULL);
}


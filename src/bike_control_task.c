#include <ch.h>
#include <hal.h>
#include "bike_control_task.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor_orientation.h"
#include "sensor_task.h"

static volatile float g_acceleration_level;
static volatile float g_motor_current;
static volatile float g_control_I_accumulator;
static volatile int g_previous_rpm;
static volatile float g_wheel_accel;
static THD_WORKING_AREA(bikestack, 1024);

int bike_control_get_acceleration_level()
{
  return (int)(g_acceleration_level / 0.00981f);
}

int bike_control_get_motor_current()
{
  return (int)(g_motor_current * 1000.0f);
}

int bike_control_get_I_accumulator()
{
  return (int)(g_control_I_accumulator / MOTOR_NEWTON_PER_A * 1000.0f);
}

static void bike_control_thread(void *p)
{
  static event_listener_t el;    
  chEvtRegister(&g_sensor_data_event, &el, 0);
  
  chRegSetThreadName("bike_ctrl");
  
  systime_t start = chVTGetSystemTime();
  bool braking = true;
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
    float total_accel = z * 0.00981f;
    float decay = 0.01f;
    g_acceleration_level = g_acceleration_level * (1-decay) + total_accel * decay;
    
    if (palReadPad(GPIOB, GPIOB_BRAKE) == 0)
    {
      braking = true;
    }
    else if (g_acceleration_level > 0.05f)
    {
      braking = false;
    }
    else if (g_acceleration_level < -0.05f)
    {
      braking = true;
    }
    
    if (!braking)
    {
      float current = g_acceleration_level * 0.3f * BIKE_MAX_WEIGHT / MOTOR_NEWTON_PER_A;
      int current_mA = (int)(current * 1000.0f) + BIKE_STARTUP_CURRENT_MA;
      
      // Do not apply full torque until motor has spun up.
      // This provides for smooth startup, and also protects fingers ;)
      int rpm = motor_orientation_get_rpm();
      if (rpm < BIKE_SOFT_START_RPM)
      {
        current_mA = current_mA * rpm / BIKE_SOFT_START_RPM;
      }
      
      // Monitor the wheel acceleration for anti-slip
//       float rpm_delta = rpm - g_previous_rpm;
//       float wheel_accel_m2_per_s = (rpm_delta / 60 * 2.23f) / delta_s;
//       float decay2 = 0.1f;
//       g_wheel_accel = g_wheel_accel * (1 - decay2) + wheel_accel_m2_per_s * decay2;
//       g_previous_rpm = rpm;
//       
//       if (g_wheel_accel > g_acceleration_level)
//       {
//         current_mA = current_mA * g_acceleration_level / g_wheel_accel;
//       }
//       
      // Always apply atleast this much current so that the motor will reliably start.
      // Getting the motor to start is necessary to get correct RPM readings.
      if (current_mA < BIKE_STARTUP_CURRENT_MA)
        current_mA = BIKE_STARTUP_CURRENT_MA;
      
      // Never apply more than this current to avoid overheating the motor.
      if (current_mA > MAX_MOTOR_CURRENT)
        current_mA = MAX_MOTOR_CURRENT;
      
      current = current_mA / 1000.0f;
      float decay2 = 0.1f;
      g_motor_current = g_motor_current * (1-decay2) + current * decay2;
      motor_run((int)(g_motor_current * 1000.0f), 0);
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


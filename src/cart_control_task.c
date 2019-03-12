#include <ch.h>
#include <hal.h>
#include <math.h>
#include "debug.h"
#include "cart_control_task.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor_orientation.h"
#include "tlv493.h"

static THD_WORKING_AREA(cartstack, 1024);

static float g_control_value_avg;
static float g_motor_current;

float cart_control_get_current()
{
  return g_motor_current;
}

static void cart_control_thread(void *p)
{
  chRegSetThreadName("cart_ctrl");
  
  tlv493_init();

  for (;;)
  {
    chThdSleepMilliseconds(20);
    int x, y, z;
    if (!tlv493_read(&x, &y, &z))
    {
      tlv493_init();
      continue;
    }
    
    float control_value = z / 2048.0f; // -1 to +1

    if (fabsf(control_value) < fabsf(g_control_value_avg) * 0.5f)
    {
      // Fast braking
      g_control_value_avg = control_value;
    }
    else
    {
      float decay = 0.2f;
      g_control_value_avg = control_value * decay + g_control_value_avg * (1 - decay);
    }

    if (fabsf(g_control_value_avg) < 0.1f)
    {
      g_motor_current = 0.0f;
      motor_stop();
    }
    else
    {
      // The control value comes from throttle handle.
      // It controls both motor torque and the max allowed RPM.
      // If max RPM is exceeded by external force (going downhill), the motor
      // will brake.
      float max_rpm = 100 * 100.0f * control_value; // Max RPM = 100 and gearing x100
      float max_current = 10.0f * control_value;
      float current_rpm = motor_orientation_get_rpm();
      float sign = (control_value > 0) ? 1 : -1;

      if (sign * current_rpm > sign * max_rpm * 0.9f && sign * g_motor_current < sign * max_current)
      {
        // Velocity control mode, adjust current to limit RPM
        float error = current_rpm - max_rpm;
        float adjustment = -error * max_current / max_rpm;
        dbg("error %5d, adj %5d, cur %5d, rpm %5d, tgt %5d", (int)(error), (int)(adjustment * 1000), (int)(g_motor_current * 1000), (int)current_rpm, (int)max_rpm);
        g_motor_current += adjustment * 0.01f;
        motor_run((g_motor_current + adjustment) * 1000, 0);
      }
      else
      {
        float decay = 0.2f;
        g_motor_current = decay * max_current + (1 - decay) * g_motor_current;
        motor_run(g_motor_current * 1000, 0);
      }
    }
  }
}

void start_cart_control()
{
  chThdCreateStatic(cartstack, sizeof(cartstack), NORMALPRIO + 4, cart_control_thread, NULL);
}


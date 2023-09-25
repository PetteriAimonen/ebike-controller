#include <ch.h>
#include <hal.h>
#include <math.h>
#include "sensor_task.h"
#include "lsm6ds3.h"
#include "debug.h"
#include "log_task.h"

static volatile float g_accel_x, g_accel_y, g_accel_z;
static volatile float g_gyro_x, g_gyro_y, g_gyro_z;
static THD_WORKING_AREA(sensorstack, 1024);
static thread_t *g_sensor_thread;
event_source_t g_sensor_data_event;

void sensors_get_accel(int* x, int* y, int* z)
{
  chSysLock();
  float xf = g_accel_x;
  float yf = g_accel_y;
  float zf = g_accel_z;
  chSysUnlock();
  
  *x = roundf(xf);
  *y = roundf(yf);
  *z = roundf(zf);
}

void sensors_get_gyro(int* x, int* y, int* z)
{
  chSysLock();
  float xf = g_gyro_x;
  float yf = g_gyro_y;
  float zf = g_gyro_z;
  chSysUnlock();
  
  *x = roundf(xf);
  *y = roundf(yf);
  *z = roundf(zf);
}

static void sensor_thread(void *p)
{
  chRegSetThreadName("sensors");
  
  // Configure accelerometer
  lsm6ds3_init();
  lsm6ds3_write(LSM6DS3_CTRL3_C, 0x01); // Reset the chip
  
  // Enable 3-wire SPI mode
  lsm6ds3_write(LSM6DS3_CTRL3_C, 0x4C); // Block data update, auto-increment read
  lsm6ds3_write(LSM6DS3_CTRL4_C, 0x04);
  
  uint8_t id = lsm6ds3_read(LSM6DS3_WHOAMI);
  if (id != 0x69 && id != 0x6A)
    abort_with_error("ACCEL_FAIL");
  
  // Configure accelerometer
  lsm6ds3_write(LSM6DS3_CTRL1_XL, 0x43); // Accelerometer samplerate 100Hz
  
  // Configure gyroscope
  lsm6ds3_write(LSM6DS3_CTRL2_G, 0x40);
  
  int ticks_since_last_sample = 0;
  for(;;)
  {
    chThdSleepMilliseconds(5);
    
    int x, y, z;
    if (lsm6ds3_read_gyro(&x, &y, &z))
    {
      float decay = 0.2f;
      float scale = 0.00875f;
      float xf = g_gyro_x, yf = g_gyro_y, zf = g_gyro_z;
      xf = (x * scale) * decay + xf * (1 - decay);
      yf = (y * scale) * decay + yf * (1 - decay);
      zf = (z * scale) * decay + zf * (1 - decay);
      
      chSysLock();
      g_gyro_x = xf;
      g_gyro_y = yf;
      g_gyro_z = zf;
      chSysUnlock();
    }
    
    if (lsm6ds3_read_acc(&x, &y, &z))
    {
      float decay = 0.2f;
      float scale = 0.061f;
      float xf = g_accel_x, yf = g_accel_y, zf = g_accel_z;
      xf = (x * scale) * decay + xf * (1 - decay);
      yf = (y * scale) * decay + yf * (1 - decay);
      zf = (z * scale) * decay + zf * (1 - decay);
      
      chSysLock();
      g_accel_x = xf;
      g_accel_y = yf;
      g_accel_z = zf;
      chSysUnlock();
      
      ticks_since_last_sample = 0;
      chEvtBroadcast(&g_sensor_data_event);
    }
    else
    {
      ticks_since_last_sample++;
      
      if (ticks_since_last_sample > 5)
      {
        log_event(EVENT_ACCEL_READ_FAIL);
      }

      if (ticks_since_last_sample > 50)
      {
        abort_with_error("ACC_WDOG");
      }
    }
  }
}

void sensors_start()
{
  chEvtObjectInit(&g_sensor_data_event);
  g_sensor_thread = chThdCreateStatic(sensorstack, sizeof(sensorstack), NORMALPRIO + 5, sensor_thread, NULL);
}


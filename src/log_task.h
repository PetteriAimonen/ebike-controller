#pragma once

#include <stdint.h>

void start_log();

typedef struct {
    uint32_t checksum;
    uint32_t systime;
    uint32_t alltime_distance_m;
    uint32_t trip_distance_m;
    float wheel_velocity; // m/s
    float wheel_accel; // m/s^2
    float motor_rpm;
    float motor_target_current; // A
    float battery_voltage; // V
    float battery_current; // A
    float mosfet_temperature; // C
    float duty_limit; // fraction
    float hill_accel; // m/s²
    float pedal_accel; // m/s²
    char state[8];
    int motor_angle; // degrees
    int hall_angle; // degrees
    int assist_level; // percent
    uint32_t events; // see log_eventtype_t
    uint32_t irq_time; // cpu clock ticks
    float motor_voltage; // voltage applied to motor, 0..1
    float motor_current; // current through motor, amperes
} eventlog_t;

// Constant size struct with padding for flash storage
typedef union {
  eventlog_t log;
  uint32_t raw[64];
} eventlog_store_t;
_Static_assert(sizeof(eventlog_store_t) == 256);


// Event logging, they are latched and added to next log entry
typedef enum {
  EVENT_MOTOR_RUN           = 0x00000001,
  EVENT_MOTOR_STOP          = 0x00000002,
  EVENT_BRAKE               = 0x00000004,
  EVENT_BRAKELIGHT          = 0x00000008,
  EVENT_TILT                = 0x00000010,

  EVENT_HALL_REVERSAL       = 0x00000100,
  EVENT_HALL_INVALID_SHORT  = 0x00000200,
  EVENT_HALL_INVALID_LONG   = 0x00000400,
  EVENT_HALL_UNSYNCED1      = 0x00001000,
  EVENT_HALL_UNSYNCED2      = 0x00002000,
  EVENT_HALL_UNSYNCED3      = 0x00004000,

  EVENT_MOTOR_OVERCURRENT   = 0x00010000,
  EVENT_VOLTAGE_SATURATED   = 0x00020000,
  EVENT_CURRENT_SATURATED   = 0x00040000,
  EVENT_DRV_FAULT           = 0x00080000,

  EVENT_BUTTON_PLUS         = 0x00100000,
  EVENT_BUTTON_MINUS        = 0x00200000,
  EVENT_BUTTON_OK           = 0x00400000,

  EVENT_ACCEL_READ_FAIL     = 0x01000000,
  
} log_eventtype_t;

// Log an event, can be called from interrupts
void log_event(log_eventtype_t event);


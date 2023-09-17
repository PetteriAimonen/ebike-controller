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
    int motor_angle;
    int hall_angle;
    int assist_level;
} eventlog_t;

// Constant size struct with padding for flash storage
typedef union {
  eventlog_t log;
  uint32_t raw[64];
} eventlog_store_t;
_Static_assert(sizeof(eventlog_store_t) == 256);





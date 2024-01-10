#pragma once

void start_bike_control();

// For log/debug
const char *bike_control_get_state();
int bike_control_get_acceleration();
int bike_control_get_pedal_accel();
int bike_control_get_hill_accel();
int bike_control_get_motor_current();
void bike_control_update_leds();

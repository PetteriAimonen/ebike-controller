#pragma once

void start_bike_control();

// For log/debug
const char *bike_control_get_state();
int bike_control_get_acceleration();
int bike_control_get_motor_current();

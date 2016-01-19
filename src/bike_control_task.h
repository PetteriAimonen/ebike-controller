#pragma once

void start_bike_control();

// For logging / debugging
int bike_control_get_motor_current();
int bike_control_get_acceleration_level();
int bike_control_get_I_accumulator();
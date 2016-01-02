#pragma once

int get_hall_sector();

// Called by motor control
void update_motor_orientation(int loop_freq);

// Returns value 0..359
int get_motor_orientation();

// Returns RPM value or 0 if stopped or not synchronized
int get_motor_rpm();

// Get raw (60 deg steps) hall angle
int get_hall_angle();
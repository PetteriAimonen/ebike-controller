/* Keeps track of hall sensor states, and estimates the motor speed and
 * orientation. At start the hall 60 degree estimate is used directly, but
 * after motor starts spinning a phase locked loop provides continuous
 * motor angle estimate.
 */

#pragma once

// Get raw (60 deg steps) hall sector/angle
int motor_orientation_get_hall_sector();
int motor_orientation_get_hall_angle();

// Called by motor control
void motor_orientation_update();

// Returns value 0..359
int motor_orientation_get_angle();

// Returns instantaneous RPM value or 0 if stopped or not synchronized
int motor_orientation_get_fast_rpm();

// Returns filtered RPM value
int motor_orientation_get_rpm();

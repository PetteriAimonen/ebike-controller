/* Keeps track of wheel speed sensor, estimates distance and speed. */

#pragma once

// Get bike velocity in meters per second.
float wheel_speed_get_velocity();

// Get bike acceleration in meters per second squared.
float wheel_speed_get_acceleration();

// Get total distance travelled in meters.
int wheel_speed_get_distance();

// Get total tick count
int wheel_speed_get_tickcount();

// Called by motor control
void wheel_speed_update();
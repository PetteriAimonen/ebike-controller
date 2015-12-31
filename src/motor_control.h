/* Sensored BLDC motor control */

#pragma once

/* Returns value 0.0f <= x < 360.0f */
float get_motor_orientation();

void start_motor_control();
void stop_motor_control();
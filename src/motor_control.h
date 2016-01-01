/* Sensored BLDC motor control */

#pragma once

/* Returns value 1..6 */
int get_hall_sector();

/* Set motor PWM output. Angle 0-359, duty 0..255.  */
void set_motor_pwm(int angle, int duty);

void start_motor_control();
void stop_motor_control();
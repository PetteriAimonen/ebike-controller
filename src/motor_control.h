/* Sensored BLDC motor control */

#pragma once

/* Set motor PWM output. Angle 0-359, duty 0..255.  */
void set_motor_pwm(int angle, int duty);

void motor_run(int duty, int advance);

void start_motor_control();
void stop_motor_control();
/* Sensored BLDC motor control */

#pragma once

#include <complex.h>

/* Set motor PWM output. Angle 0-359, duty 0..255.  */
void set_motor_pwm(int angle, int duty);

void get_foc_debug(float complex *i_vector, float complex *u_vector);

void motor_run(int duty, int advance);

void start_motor_control();
void stop_motor_control();

int motor_get_interrupt_time();
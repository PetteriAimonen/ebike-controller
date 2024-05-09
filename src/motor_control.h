/* Field oriented motor control.
 * This runs a PI control loop to try to keep the stator current
 * 90 degrees ahead of rotor position.
 */

#pragma once

#include <complex.h>

/* Set motor PWM output. Angle 0-359, duty 0..255.  */
void set_motor_pwm(int angle, int duty);

void motor_run(int torque_current_mA, int advance);
void motor_stop();

// Enable regenerative braking when rotation RPM is positive
// and torque current is negative, regardless of brake switch
// value.
void motor_set_regen_brake(bool enable);

void start_motor_control();
void stop_motor_control();

// Debugging information
void get_foc_debug(float complex *i_vector, float complex *u_vector);
int motor_get_interrupt_time();
float motor_get_voltage_abs();
float motor_get_current_abs();

/* Adjustable RPM and temperature limits for the motor. The limits
 * gradually lower the maximum PWM duty cycle when threshold is exceeded.
 */

#pragma once

void motor_limits_update_max_duty();
int motor_limits_get_max_duty();

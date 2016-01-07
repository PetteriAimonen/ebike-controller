/* Adjustable RPM and temperature limits for the motor. The limits
 * gradually lower the maximum PWM duty cycle when threshold is exceeded.
 */

#pragma once

int motor_limits_get_max_duty();

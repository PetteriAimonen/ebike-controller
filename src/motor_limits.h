/* Adjustable RPM and temperature limits for the motor. The limits
 * gradually lower the commanded current when threshold is exceeded.
 */

#pragma once

void motor_limits_update();
float motor_limits_get_fraction();

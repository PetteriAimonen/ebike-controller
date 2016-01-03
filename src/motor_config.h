#pragma once

// Angle offset for hall sensors (degrees)
#define HALL_OFFSET 210

// Base frequency for PWM and control loop.
#define PWM_FREQ 50000
#define CONTROL_FREQ 10000

// PWM period, should be selected so that 168MHz / PWM_FREQ divides nicely.
#define PWM_MAX  840
// #define PWM_MAX_DUTY 800
#define PWM_MAX_DUTY 300

// Maximum motor current (A)
#define MAX_MOTOR_CURRENT 10.0f

// Field oriented control PI loop terms
#define FOC_P_TERM 0.5f
#define FOC_I_TERM 0.05f

#pragma once

void dbg(const char *fmt, ...);

void abort_with_error(const char *fmt, ...);

// ITM ports for outputting data in realtime over SWO pin
#define ITM_PWM_CCR1        10
#define ITM_PWM_CCR2        11
#define ITM_PWM_CCR3        12
#define ITM_CURRENT_PH1     13
#define ITM_CURRENT_PH3     14
#define ITM_BATTVOLTAGE     15
#define ITM_ORIENTATION     16
#define ITM_HALLSECTOR      17
#define ITM_TARGETCURRENT   18
#define ITM_RPM             19
#define ITM_UVECTOR_R       20
#define ITM_UVECTOR_I       21
#define ITM_IVECTOR_R       22
#define ITM_IVECTOR_I       23
#define ITM_ABSCUR          24
#define ITM_ACCEL           25
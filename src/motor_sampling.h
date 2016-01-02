#pragma once
#include <stdint.h>
#include <hal_streams.h>

void motor_sampling_start();
void motor_sampling_store();
void motor_get_currents(int *phase1_mA, int *phase3_mA);
void motor_sampling_print(BaseSequentialStream *stream);
/* Driving external coil for DC-DC converter output. */

#pragma once

extern bool g_dcdc_active;

void start_dcdc_control();
void stop_dcdc_control();

void set_dcdc_mode(int mV, int mA);

int get_dcdc_current_mA();


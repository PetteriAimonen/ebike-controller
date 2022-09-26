/* Driving external coil for DC-DC converter output. */

#pragma once

void start_dcdc_control();
void stop_dcdc_control();

enum DCDC_MODE {
    DCDC_OUTPUT_CCCV, // Output with max current & max voltage limits
    DCDC_INPUT_CCCV, // Input with max current & min voltage limits
    DCDC_INPUT_MPPT // Input with max current & min voltage limits and MPPT tracking
};

void set_dcdc_mode(enum DCDC_MODE mode, int mV, int mA);

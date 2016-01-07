/* Runs the ADC converter to monitor critical motor parameters,
 * like phase current and temperatures. */

#pragma once
#include <stdint.h>
#include <ch.h>
#include <hal_streams.h>

// Initializes the motor sampling, called by motor_control.
void motor_sampling_init();

// Updates value estimates, called by motor_control.
void motor_sampling_update();

// Stores current values into debug ringbuffer, called by motor_control.
void motor_sampling_store();

// Prints out the debug ring buffer. Called from shell_commands.
void motor_sampling_print(BaseSequentialStream *stream);

// Get the current motor phase currents, for FOC and other uses.
void motor_get_currents(int *phase1_mA, int *phase3_mA);

// Get an estimate for the total current being taken out of the battery.
int get_battery_current_mA();

// Get the current battery voltage.
int get_battery_voltage_mV();

// Get motor temperature in millicelcius.
int get_motor_temperature_mC();

// Get mosfet temperature in millicelcius.
int get_mosfet_temperature_mC();

#pragma once

// Angle offset for hall sensors (degrees)
#define HALL_OFFSET 210

// Base frequency for PWM and control loop.
#define PWM_FREQ 25000
#define CONTROL_FREQ 10000

// PWM period, should be selected so that 168MHz / PWM_FREQ divides nicely.
#define PWM_MAX  840
#define PWM_MAX_DUTY 700
// Shunt resistance
#define SHUNT_uA_PER_ADC_VAL 16113

// Maximum motor current (mA)
#define MAX_MOTOR_CURRENT 10000

// Filter time for motor temperature and battery currents
#define MOTOR_FILTER_TIME_S   0.05f
#define DUTY_LIMIT_FILTER_S   3.0f

// Field oriented control PI loop terms
#define FOC_P_TERM 0.5f
#define FOC_I_TERM 0.05f

// Distance in meters per one wheel speed sensor tick.
#define WHEEL_SPEED_STEP (2.17f/6)

// Maximum motor speed, temperatures and minimum battery voltage
// For each, A point gives the start of the linear duty lowering,
// and B point is where the duty drops to 0%.
#define MOTOR_MAX_RPM_A         10000
#define MOTOR_MAX_RPM_B         12000
#define MOTOR_MAX_TEMP_A          50000
#define MOTOR_MAX_TEMP_B          70000
#define MOSFET_MAX_TEMP_A         60000
#define MOSFET_MAX_TEMP_B         90000
#define BATTERY_MIN_VOLTAGE_A     33000
#define BATTERY_MIN_VOLTAGE_B     30000
#define BATTERY_MAX_VOLTAGE_A     43000
#define BATTERY_MAX_VOLTAGE_B     46000
#define BATTERY_MAX_CURRENT_A      3500
#define BATTERY_MAX_CURRENT_B      4500

// Bike control parameters
#define MOTOR_NEWTON_PER_A         5.0f
#define BIKE_WEIGHT_KG            90.0f
#define BIKE_MIN_VELOCITY          0.2f
#define BIKE_MIN_PEDAL_INTERVAL_S    2
#define BIKE_BRAKE_THRESHOLD_M_S2  1.0f
#define BIKE_BRAKE_THRESHOLD_B_M_S2  5.0f
#define BIKE_TORQUE_FILTER_S       0.5f
#define BIKE_MIN_CURRENT_A         0.2f
#define BIKE_SOFTSTART_A           0.5f

#pragma once

// Angle offset for hall sensors (degrees)
#define HALL_OFFSET 210

// Glitch filter count for hall sensors (number of samples)
#define HALL_FILTER 5

// Timeout for invalid hall sensor readings
#define HALL_TIMEOUT 500

// Backoff time after a lost sync.
// 6-step commutation is used during this time.
#define HALL_BACKOFF_MS 1000

// Base frequency for PWM and control loop.
#define PWM_FREQ 25000
#define CONTROL_FREQ 10000

// PWM period, should be selected so that 168MHz / PWM_FREQ divides nicely.
#define PWM_MAX  840
#define PWM_MAX_DUTY 780
// Shunt resistance
#define SHUNT_uA_PER_ADC_VAL 16113

// Maximum motor current (mA)
#define MAX_MOTOR_CURRENT 20000
#define MOTOR_ABSMAX_CURRENT      30000

// Filter time for motor RPM estimate, in number of sectors
#define MOTOR_RPM_FILTER_SECTORS 6
#define MOTOR_RPM_FILTER_TIME_MIN 0.05f
#define MOTOR_RPM_FILTER_TIME_MAX 0.5f

// Filter time for motor temperature and battery currents
#define MOTOR_FILTER_TIME_S   0.1f
#define DUTY_LIMIT_FILTER_TRIP_S   0.1f
#define DUTY_LIMIT_FILTER_RELAX_S   0.5f

// Field oriented control PI loop terms
#define FOC_P_TERM 0.2f
#define FOC_I_TERM 0.05f

// Distance in meters per one wheel speed sensor tick.
// Wheel to motor gear ratio is 1:46 for the 28" wheels, 1:117 for the 20" wheel
// #define WHEEL_SPEED_STEP (2.17f/6)

#define MOTOR_ORIENTATION_FILTER_TIME 0.1f

// Small amount of holding current for stability at low loads (mA)
#define MOTOR_HOLDING_CURRENT 50.0f

// Maximum and minimum RPM for motor control purposes
// Motor speed is assumed to be unknown if it is outside these limits
#define CTRL_MIN_RPM 100
#define CTRL_MAX_RPM 25000

// Warn about battery voltage below this percentage
#define BATTERY_LEVEL_WARN 30

// Maximum motor speed, temperatures and minimum battery voltage
// For each, A point gives the start of the linear duty lowering,
// and B point is where the duty drops to 0%.
// #define MOTOR_MAX_RPM_A         9000
// #define MOTOR_MAX_RPM_B         10000
#define MOTOR_MAX_TEMP_A          50000
#define MOTOR_MAX_TEMP_B          70000
#define MOSFET_MAX_TEMP_A         60000
#define MOSFET_MAX_TEMP_B         90000
#define BATTERY_MIN_VOLTAGE_A     34000
#define BATTERY_MIN_VOLTAGE_B     30000
#define BATTERY_MAX_CURRENT_A      6000
#define BATTERY_MAX_CURRENT_B      8000
#define BATTERY_MAX_REGEN_MV      44000


// Maximum acceleration, as time in seconds from 0 to max RPM
// #define MOTOR_MIN_ACCEL_TIME    5.0f
#define ACCEL_LIMIT_SPEED_M_S      2.0f

// Bike control parameters
// #define MOTOR_NEWTON_PER_A         5.0f
// #define BIKE_WEIGHT_KG           100.0f
#define BIKE_MIN_VELOCITY          0.2f
#define BIKE_MAX_VELOCITY          7.5f
#define BIKE_BRAKE_THRESHOLD_M_S2  1.0f
#define BIKE_BRAKE_THRESHOLD_B_M_S2  5.0f
#define BIKE_TORQUE_FILTER_S       0.5f
#define BIKE_MIN_CURRENT_A         0.2f
#define BIKE_SOFTSTART_A           1.0f
#define BIKE_SOFTSTART_S           0.5f
#define BIKE_MIN_PEDAL_ACCEL       0.1f

#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <math.h>
#include "dcdc_control.h"
#include "motor_sampling.h"

#define PWM_FREQ 25000
#define CONTROL_FREQ 25000
#define PWM_MAX  1680
#define PWM_MAX_DUTY 1500

static enum DCDC_MODE g_dcdc_mode;
static int g_dcdc_mV;
static int g_dcdc_mA;
static int g_dcdc_duty;

void set_dcdc_mode(enum DCDC_MODE mode, int mV, int mA)
{
    g_dcdc_mode = mode;
    g_dcdc_mV = mV;
    g_dcdc_mA = mA;
}

CH_FAST_IRQ_HANDLER(STM32_TIM1_CC_HANDLER)
{
    TIM1->SR &= ~TIM_SR_CC2IF;

    motor_sampling_update();

    // Duty from DC voltage
    int battery_mV = get_battery_voltage_mV();
    int duty = PWM_MAX * g_dcdc_mV / battery_mV;

    // Phase 1 goes through coil to output, phase 3 is ground return
    // The output current can be measured from phase 3, coil current from phase 1
    int phase1_mA, phase3_mA;
    motor_get_currents(&phase1_mA, &phase3_mA);
    int output_mA = -phase3_mA;

    // // If current is outside target range, we simulate 1kohm series resistance
    // // The control loop is simple P-controller
    // int limiter_impedance_ohm = 100;
    // int duty_per_mA_limit = PWM_MAX * limiter_impedance_ohm / battery_mV;

    // if (g_dcdc_mode == DCDC_OUTPUT_CCCV)
    // {
    //     if (output_mA < -1000)
    //     {
    //         duty += duty_per_mA_limit * (-1000 - output_mA);
    //     }
    //     else if (output_mA > g_dcdc_mA)
    //     {
    //         duty -= duty_per_mA_limit * (output_mA - g_dcdc_mA);
    //     }
    // }

    if (duty < 0) duty = 0;
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;

    int temperature_mC = get_mosfet_temperature_mC();

    if (battery_mV < 30000 || battery_mV >= 44000 || temperature_mC > 90000)
    {
        // Disable outputs
        TIM3->CNT = 0;
    }
    else
    {
        TIM1->CCR3 = duty; // CCR3 controls phase 1
        TIM1->BDTR |= TIM_BDTR_MOE;
        TIM3->CCR1 = 1;
        TIM3->CNT = 10;
        TIM3->CR1 |= TIM_CR1_CEN;
    }
}

void start_dcdc_control()
{
    g_dcdc_mode = DCDC_OUTPUT_CCCV;
    g_dcdc_mA = 0;
    g_dcdc_mV = 0;

    // Enable timer clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Timer config for center-aligned PWM
    TIM1->CR1 = TIM_CR1_CMS;
    TIM1->CR2 = 0; // All outputs off when MOE=0.
    TIM1->DIER = 0;
    TIM1->CCMR1 = 0x6868;
    TIM1->CCMR2 = 0x6068;
    TIM1->CCER = 0x1555;
    TIM1->CNT = 0;
    TIM1->PSC = STM32_TIMCLK2 / (2 * PWM_FREQ * PWM_MAX) - 1;
    TIM1->ARR = PWM_MAX - 1;
    TIM1->RCR = (PWM_FREQ * 2) / CONTROL_FREQ - 1;
    TIM1->BDTR = 42; // 0.2µs dead time
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->EGR = TIM_EGR_UG;

    // ADC sampling trigger
    TIM1->CCR4 = PWM_MAX - 5;

    // TIM3 CH1 is used for the gate enable signal. This way the mosfets will be
    // disabled automatically if the code is e.g. stopped in a debugger.
    // It is setup to count downwards at CONTROL_FREQ, if it reaches zero
    // it will disable EN_GATE.
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CR1 = TIM_CR1_OPM | TIM_CR1_DIR;
    TIM3->CCMR1 = 0x0070;
    TIM3->CCR1 = 1;
    TIM3->CCER = TIM_CCER_CC1E;
    TIM3->PSC = STM32_TIMCLK1 / CONTROL_FREQ - 1;
    TIM3->ARR = 1; // Value the timer stops at in one pulse mode
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CNT = CONTROL_FREQ; // For DC offset calibration
    TIM3->CR1 |= TIM_CR1_CEN;

    // Initialize the ADC sampling. This performs DC offset calibration so it
    // has to happen before enabling timer, but after EN_GATE is enabled.
    motor_sampling_init();

    // Enable interrupt for performing DCDC control
    TIM1->DIER = TIM_DIER_CC2IE;
    nvicEnableVector(STM32_TIM1_CC_NUMBER, 0);

    // Start the timer
    TIM3->CNT = 10;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void stop_dcdc_control()
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->DIER = 0;
  nvicDisableVector(STM32_TIM1_CC_NUMBER);
  TIM3->CNT = 0;
}
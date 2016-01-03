#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <math.h>
#include <complex.h>
#include "debug.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor_orientation.h"
#include "motor_sampling.h"

// Some useful constants
static const float f_pi = 3.14159265f;
static const float f_sqrt3 = 1.7320508f;
static const float f_invsqrt3 = 0.57735027f;
static const float complex f_v1 = 1.0f; // 0 deg angle
static const float complex f_v2 = -0.5f + 0.866025fi; // +120 deg angle
static const float complex f_v3 = -0.5f - 0.866025fi; // +240 deg angle

// GCC's cabsf is a bit slowish for some reason..
float cabsf(float complex x)
{
  float i = cimagf(x);
  float r = crealf(x);
  return sqrtf(i*i + r*r);
}

void set_modulation_vector(float complex v)
{
  // Inverse Clark transform
  float u1 = crealf(v) * crealf(f_v1) + cimagf(v) * cimagf(f_v1);
  float u2 = crealf(v) * crealf(f_v2) + cimagf(v) * cimagf(f_v2);
  float u3 = crealf(v) * crealf(f_v3) + cimagf(v) * cimagf(f_v3);
  
  // Choose the smallest value as the PWM term that stays 0
  float min = u1;
  if (u2 < min) min = u2;
  if (u3 < min) min = u3;
  u1 -= min;
  u2 -= min;
  u3 -= min;
  
  // Convert to integers for PWM
  // Note: CCR1 channel is connected to phase3 and CCR3 to phase1.
  float scaler = PWM_MAX_DUTY / f_sqrt3;
  TIM1->CCR3 = (int)roundf(u1 * scaler);
  TIM1->CCR2 = (int)roundf(u2 * scaler);
  TIM1->CCR1 = (int)roundf(u3 * scaler);
}

void set_motor_pwm(int angle, int duty)
{
  float complex v = cexpf(I * f_pi / 180.0f * angle) * duty / 255.0f;
  set_modulation_vector(v);
}

float complex get_current_vector()
{
  int phase1_mA, phase3_mA;
  motor_get_currents(&phase1_mA, &phase3_mA);
  
  float phase1_A = phase1_mA / 1000.0f;
  float phase3_A = phase3_mA / 1000.0f;
  float phase2_A = -(phase1_A + phase3_A);
  
  return f_v1 * phase1_A + f_v2 * phase2_A + f_v3 * phase3_A;
}

static float g_foc_torque_current = -1.0f;
static float complex g_foc_I_accumulator = 0.0f;
static float complex g_debug_latest_I_vector = 0.0f;
static float complex g_debug_latest_U_vector = 0.0f;

void get_foc_debug(float complex *i_vector, float complex *u_vector)
{
  *i_vector = g_debug_latest_I_vector;
  *u_vector = g_debug_latest_U_vector;
}

static void do_field_oriented_control(bool do_modulation)
{
  // Get vector for rotor orientation
  float rotor_angle = get_motor_orientation();
  float complex rotor_vector = cexpf(I * f_pi / 180.0f * rotor_angle);
  
  // Project the current vector to rotor coordinates
  float complex current = get_current_vector();
  current *= conjf(rotor_vector);
  g_debug_latest_I_vector = current;
  
  // Do PI control to match the requested torque
  // Current varies from 0..MAX_MOTOR_CURRENT.
  // The voltage vector length varies 0..1
  float complex reference = I * g_foc_torque_current;
  float complex error = reference - current;
  g_foc_I_accumulator += FOC_I_TERM * error;
  float complex voltage = reference + FOC_P_TERM * error + g_foc_I_accumulator;
  voltage /= MAX_MOTOR_CURRENT;
  
  float vabs = cabsf(voltage);
  if (vabs > 1.0f)
  {
    // Value has saturated
    voltage /= vabs;
  }
  
  float iabs = cabsf(g_foc_I_accumulator);
  if (iabs > MAX_MOTOR_CURRENT)
  {
    // Prevent integrator windup
    g_foc_I_accumulator *= MAX_MOTOR_CURRENT / iabs;
  }
  
  g_debug_latest_U_vector = voltage;
  
  if (do_modulation)
  {
    // Project voltage vector back to stator coordinates
    voltage *= rotor_vector;
    set_modulation_vector(voltage);
  }
}

static int g_interrupt_time;

CH_FAST_IRQ_HANDLER(STM32_TIM1_UP_HANDLER)
{
  TIM1->SR &= ~TIM_SR_UIF;
  
  update_motor_orientation();
  motor_sampling_store();
  motor_sampling_update();
  
  if (g_foc_torque_current >= 0.0f)
  {
    // Do FOC commutation
    do_field_oriented_control(true);
  }
  else
  {
    // Dry-run to get samples of the vectors that would have been set
    do_field_oriented_control(false);
  }
  
  int irq_time = TIM1->CNT;
  if (irq_time > g_interrupt_time)
      g_interrupt_time = irq_time;
}

int motor_get_interrupt_time()
{
  return g_interrupt_time;
}

void motor_run(int duty, int advance)
{
  g_foc_torque_current = MAX_MOTOR_CURRENT * duty / 255.0f;
}

void start_motor_control()
{
  g_foc_torque_current = -1.0f;
  g_foc_I_accumulator = 0.0f;
  
  // Enable timer clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  
  // Disable timer while we configure
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  
  // Timer config for center-aligned PWM
  TIM1->CR1 = TIM_CR1_CMS;
  TIM1->CR2 = 0; // All outputs off when MOE=0.
  TIM1->CCMR1 = 0x6868;
  TIM1->CCMR2 = 0x6068;
  TIM1->CCER = 0x1555;
  TIM1->CNT = 0;
  TIM1->PSC = STM32_TIMCLK2 / (2 * PWM_FREQ * PWM_MAX) - 1;
  TIM1->ARR = PWM_MAX - 1;
  TIM1->RCR = (PWM_FREQ * 2) / CONTROL_FREQ - 1;
  TIM1->BDTR = 84; // 0.5µs dead time
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
  TIM1->EGR = TIM_EGR_UG;
  
  // Uses CC4 to generate a pulse during the off cycle, for current sampling.
  // PWM_MAX_DUTY ensures there is enough off time for the sampling to occur.
  TIM1->CCR4 = PWM_MAX - 20;
  motor_sampling_start();
  
  // Enable interrupt for performing motor control
  TIM1->DIER |= TIM_DIER_UIE;
  nvicEnableVector(STM32_TIM1_UP_NUMBER, 0);
  
  // Start the timer
  TIM1->CR1 |= TIM_CR1_CEN;
  TIM1->BDTR |= TIM_BDTR_MOE;
  palSetPad(GPIOB, GPIOB_EN_GATE);
}

void stop_motor_control()
{
  palClearPad(GPIOB, GPIOB_EN_GATE);
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
}

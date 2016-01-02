#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <math.h>
#include "debug.h"
#include "motor_control.h"
#include "motor_config.h"
#include "motor_orientation.h"
#include "motor_sampling.h"

#define PWM_FREQ 50000
#define PWM_MAX_DUTY 800
#define PWM_MAX  840

const uint8_t g_sine_table[121] = {
    0,   4,   8,  13,  17,  22,  26,  31,  35,  40,  44,  48,
   53,  57,  61,  66,  70,  74,  79,  83,  87,  91,  95, 100,
  104, 108, 112, 116, 120, 124, 127, 131, 135, 139, 143, 146,
  150, 154, 157, 161, 164, 167, 171, 174, 177, 181, 184, 187,
  190, 193, 196, 198, 201, 204, 207, 209, 212, 214, 217, 219,
  221, 223, 226, 228, 230, 232, 233, 235, 237, 238, 240, 242,
  243, 244, 246, 247, 248, 249, 250, 251, 252, 252, 253, 254,
  254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
  254, 254, 253, 252, 252, 251, 250, 249, 248, 247, 246, 244,
  243, 242, 240, 238, 237, 235, 233, 232, 230, 228, 226, 223,
  221
};

const uint8_t g_trapezoid_table[360] = {
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,
   12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,
   24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,
   36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,
   48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
   
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  60,
   
   59,  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48,
   47,  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,
   35,  34,  33,  32,  31,  30,  29,  28,  27,  26,  25,  24,
   23,  22,  21,  20,  19,  18,  17,  16,  15,  14,  13,  12,
   11,  10,   9,   8,   7,   6,   5,   4,   3,   2,   1,   0,
   
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
};

void set_motor_pwm(int angle, int duty)
{
  angle -= 150;
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  if (duty < 0) duty = 0;
  if (duty > 255) duty = 255;
  
  // CCRx = PWM_MAX_DUTY * (sine_table/255) * (duty/255)
  // => CCRx = sine_table * (duty * PWM_MAX * 256 / (255 * 255)) / 256;
  // This precalc avoids divisions later in the computation.
  duty = duty * PWM_MAX_DUTY * 256 / (255 * 255);
 
  // Trapezoidal drive waveforms
  TIM1->CCR1 = g_trapezoid_table[(angle + 240) % 360] * duty / 256;
  TIM1->CCR2 = g_trapezoid_table[(angle + 120) % 360] * duty / 256;
  TIM1->CCR3 = g_trapezoid_table[(angle +   0) % 360] * duty / 256;
  
  // Sinusoidal drive waveforms based on http://www.atmel.com/Images/doc8030.pdf
  if (angle < 120)
  {
    // Phase1 = sin(angle), Phase2 = 0, Phase3 = sin(120-angle)
    TIM1->CCR1 = g_sine_table[angle] * duty / 256;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = g_sine_table[120-angle] * duty / 256;
  }
  else if (angle < 240)
  {
    // Phase1 = sin(240-angle), Phase2 = sin(angle-120), Phase3 = 0
    TIM1->CCR1 = g_sine_table[240-angle] * duty / 256;
    TIM1->CCR2 = g_sine_table[angle-120] * duty / 256;
    TIM1->CCR3 = 0;
  }
  else
  {
    // Phase1 = 0, Phase2 = sin(360-angle), Phase3 = sin(angle-240)
    TIM1->CCR1 = 0;
    TIM1->CCR2 = g_sine_table[360-angle] * duty / 256;
    TIM1->CCR3 = g_sine_table[angle-240] * duty / 256;
  }
}

static int g_motor_run_duty = -1;
static int g_motor_run_advance = 0;

CH_FAST_IRQ_HANDLER(STM32_TIM1_UP_HANDLER)
{
  // In center-aligned PWM mode, update events occur twice every period.
  TIM1->SR &= ~TIM_SR_UIF;
  
  if (TIM1->CR1 & TIM_CR1_DIR)
  {
    update_motor_orientation(PWM_FREQ);
    motor_sampling_store();
    
    if (g_motor_run_duty >= 0)
    {
      int angle = get_motor_orientation() + g_motor_run_advance;
      set_motor_pwm(angle, g_motor_run_duty);
    }
  }
}

void motor_run(int duty, int advance)
{
  g_motor_run_advance = advance;
  g_motor_run_duty = duty;
}

void start_motor_control()
{
  g_motor_run_duty = -1;
  g_motor_run_advance = 0;
  
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
  TIM1->BDTR = 84; // 0.5µs dead time
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
  TIM1->EGR = TIM_EGR_UG;
  
  // Uses CC4 to generate a pulse during the off cycle, for current sampling.
  // PWM_MAX_DUTY ensures there is enough off time for the sampling to occur.
  TIM1->CCR4 = PWM_MAX - 2;
  
  // Enable interrupt for performing motor control
  TIM1->DIER |= TIM_DIER_UIE;
  nvicEnableVector(STM32_TIM1_UP_NUMBER, 5);
  
  // Start the timer
  TIM1->CR1 |= TIM_CR1_CEN;
  TIM1->BDTR |= TIM_BDTR_MOE;
  palSetPad(GPIOB, GPIOB_EN_GATE);
  
  motor_sampling_start();
}

void stop_motor_control()
{
  palClearPad(GPIOB, GPIOB_EN_GATE);
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
}

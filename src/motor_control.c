#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <math.h>
#include "motor_control.h"
#include "motor_config.h"

#define PWM_FREQ 50000
#define PWM_MAX  840
#define F_PI 3.1415926535f

// Table index is H3 | H2 | H1, value is sector 1-6.
// Typical rotation is:
// Hall     Sector
// 001=1    1
// 011=3    2
// 010=2    3
// 110=6    4
// 100=4    5
// 101=5    6
const int g_hall_table[8] = {-1, 1, 3, 2, 5, 6, 4, -1};

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

int get_hall_sector()
{
  uint32_t hall_state =
    (palReadPad(GPIOB, GPIOB_HALL_1) ? 1 : 0) |
    (palReadPad(GPIOB, GPIOB_HALL_2) ? 2 : 0) |
    (palReadPad(GPIOC, GPIOC_HALL_3) ? 4 : 0);
  
  return g_hall_table[hall_state];
}

void set_motor_pwm(int angle, int duty)
{
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  if (duty < 0) duty = 0;
  if (duty > 255) duty = 255;
  
  // CCRx = PWM_MAX * (sine_table/255) * (duty/255)
  // => CCRx = sine_table * (duty * PWM_MAX * 256 / (255 * 255)) / 256;
  // This precalc avoids divisions later in the computation.
  duty = duty * PWM_MAX * 256 / (255 * 255);
  
  // Drive waveforms based on http://www.atmel.com/Images/doc8030.pdf
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

void start_motor_control()
{
  // Enable timer clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  
  // Disable timer while we configure
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  
  // Timer config for center-aligned PWM
  TIM1->CR1 = TIM_CR1_CMS;
  TIM1->CR2 = 0; // All outputs off when MOE=0
  TIM1->CCMR1 = 0x6868;
  TIM1->CCMR2 = 0x0068;
  TIM1->CCER = 0x0555;
  TIM1->CNT = 0;
  TIM1->PSC = STM32_TIMCLK2 / (PWM_FREQ * PWM_MAX * 2) - 1;
  TIM1->ARR = PWM_MAX - 1;
  TIM1->BDTR = 84; // 0.5µs dead time
  TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
  TIM1->EGR = TIM_EGR_UG;
  
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

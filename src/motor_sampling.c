#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <math.h>
#include <chprintf.h>
#include <string.h>
#include "debug.h"
#include "motor_sampling.h"
#include "motor_orientation.h"

#define SHUNT_mA_PER_ADC_VAL 8

typedef struct {
  uint8_t motor_orientation;
  uint8_t hall_angle;
  uint8_t ccr1;
  uint8_t ccr2;
  uint8_t ccr3;
  int8_t current_ph1;
  int8_t current_ph3;
} motor_sample_t;

#define MOTOR_SAMPLE_COUNT 8192
static volatile motor_sample_t g_motor_samples[MOTOR_SAMPLE_COUNT] __attribute__((section(".ram4")));
static volatile int g_motor_samples_writeidx;
static volatile bool g_enable_sampling;

void motor_sampling_start()
{
  // Dual ADC mode with simultaneous injected conversion.
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
  ADC->CCR = STM32_ADC_ADCPRE | ADC_CCR_MULTI_2 | ADC_CCR_MULTI_0;
  ADC1->CR1 = ADC2->CR1 = 0;
  ADC1->CR2 = ADC2->CR2 = ADC_CR2_ADON;
  
  
  // 3 cycle sampling for all channels
  ADC1->SMPR1 = ADC2->SMPR1 = 0;
  ADC1->SMPR2 = ADC2->SMPR2 = 0;
  
  // ADC1 sequence:
  // Ch9: BR_SO1, Ch12: AN_IN, Ch3: ADC_TEMP
  ADC1->JSQR = (2 << 20) | (9 << 5) | (12 << 10) | (3 << 15);
  
  // ADC2 sequence:
  // Ch8: BR_SO2, Ch10: TEMP_MOTOR
  ADC2->JSQR = (1 << 20) | (8 << 10) | (10 << 15);
  
  // Injected sequence trigger on TIM1 CC4 falling edge
  ADC1->CR2 = ADC_CR2_JEXTEN_1 | ADC_CR2_ADON;
  
  // Calculate DC offset
  ADC1->JOFR1 = ADC2->JOFR1 = 0;
  palSetPad(GPIOA, GPIOA_DC_CAL);
  for (int i = 0; i < 10; i++)
  {
    ADC1->CR2 |= ADC_CR2_JSWSTART;
    chThdSleepMilliseconds(2);
  }
  ADC1->JOFR1 = ADC1->JDR1;
  ADC2->JOFR1 = ADC2->JDR1;
  palClearPad(GPIOA, GPIOA_DC_CAL);
  
  g_enable_sampling = true;
}

void motor_get_currents(int *phase1_mA, int *phase3_mA)
{
  int16_t br1 = ADC1->JDR1;
  int16_t br2 = ADC2->JDR1;
  *phase1_mA = br2 * SHUNT_mA_PER_ADC_VAL;
  *phase3_mA = br1 * SHUNT_mA_PER_ADC_VAL;
}

void motor_sampling_store()
{
  motor_sample_t sample = {};
  sample.motor_orientation = get_motor_orientation() / 2;
  sample.hall_angle = get_hall_angle() / 2;
  sample.ccr1 = TIM1->CCR1 / 4;
  sample.ccr2 = TIM1->CCR2 / 4;
  sample.ccr3 = TIM1->CCR3 / 4;
  
  int phase1, phase3;
  motor_get_currents(&phase1, &phase3);
  phase1 /= 100; phase3 /= 100;
  if (phase1 > 127) phase1 = 127;
  if (phase1 < -127) phase1 = 127;
  if (phase3 > 127) phase3 = 127;
  if (phase3 < -127) phase3 = -127;
  sample.current_ph1 = phase1;
  sample.current_ph3 = phase3;
  
  if (g_enable_sampling)
  {
    g_motor_samples[g_motor_samples_writeidx++] = sample;
    if (g_motor_samples_writeidx >= MOTOR_SAMPLE_COUNT)
      g_motor_samples_writeidx = 0;
  }
}

void motor_sampling_print(BaseSequentialStream *stream)
{
  g_enable_sampling = false;
  
  chprintf(stream, "#ORI HALL CCR1 CCR2 CCR3 I_Phase1 I_Phase3\r\n");
  for (int i = 0; i < MOTOR_SAMPLE_COUNT; i++)
  {
    motor_sample_t sample = g_motor_samples[(i + g_motor_samples_writeidx) % MOTOR_SAMPLE_COUNT];
    
    char buf[64];
    chsnprintf(buf, sizeof(buf), "%4d %4d %4d %4d %4d %8d %8d\r\n",
               sample.motor_orientation * 2, sample.hall_angle * 2,
               sample.ccr1 * 4, sample.ccr2 * 4, sample.ccr3 * 4,
               sample.current_ph1 * 100, sample.current_ph3 * 100);
    chSequentialStreamWrite(stream, (void*)buf, strlen(buf));
  }
  
  g_enable_sampling = true;
}

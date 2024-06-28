#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <math.h>
#include <chprintf.h>
#include <string.h>
#include "debug.h"
#include "settings.h"
#include "motor_config.h"
#include "motor_control.h"
#include "motor_sampling.h"
#include "motor_orientation.h"

typedef struct {
  uint8_t motor_orientation;
  uint8_t hall_angle;
  uint8_t ph1;
  uint8_t ph2;
  uint8_t ph3;
  int8_t current_ph1;
  int8_t current_ph3;
  int8_t ivector_r;
  int8_t ivector_i;
  int8_t uvector_r;
  int8_t uvector_i;
} motor_sample_t;

#define MOTOR_SAMPLE_COUNT 4096
static volatile motor_sample_t g_motor_samples[MOTOR_SAMPLE_COUNT] __attribute__((section(".ram4")));
static volatile int g_motor_samples_writeidx;
static volatile bool g_enable_sampling;

void motor_sampling_init()
{
  // Dual ADC mode with simultaneous injected conversion.
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
  ADC->CCR = STM32_ADC_ADCPRE | ADC_CCR_MULTI_2 | ADC_CCR_MULTI_0;
  ADC1->CR1 = ADC2->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 = ADC2->CR2 = ADC_CR2_ADON;
  
  // 3 cycle sampling for current measurement, 15 cycle for other channels
  ADC1->SMPR1 = ADC2->SMPR1 = 0;
  ADC1->SMPR2 = ADC2->SMPR2 = 0;
  
  ADC1->SMPR1 |= (1 << 6); // Ch12
  ADC2->SMPR2 |= (1 << 9); // Ch3

  // ADC1 sequence:
  // Ch9: BR_SO1, Ch12: AN_IN
  ADC1->JSQR = (1 << 20) | (9 << 10) | (12 << 15);
  
  // ADC2 sequence:
  // Ch8: BR_SO2, Ch3: ADC_TEMP
  ADC2->JSQR = (1 << 20) | (8 << 10) | (3 << 15);
  
  // Injected sequence trigger on TIM1 CC4 rising edge
  ADC1->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_ADON;
  
  // Calculate DC offset
  ADC1->JOFR1 = ADC2->JOFR1 = 0;
  palSetPad(GPIOA, GPIOA_DC_CAL);
  
  // Enable TIM1 for sampling at the target rate
  uint32_t bdtr_old = TIM1->BDTR;
  uint32_t ccer_old = TIM1->CCER;
  uint32_t dier_old = TIM1->DIER;
  TIM1->BDTR = 42;
  TIM1->DIER = 0;
  TIM1->CCER = TIM_CCER_CC4E;
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
  
  chThdSleepMilliseconds(50);

  // Take average of 100 samples
  int j1 = 0;
  int j2 = 0;
  for (int i = 0; i < 100; i++)
  {
    chThdSleepMilliseconds(1);
    j1 += ADC1->JDR1;
    j2 += ADC2->JDR1;
  }
  
  palClearPad(GPIOA, GPIOA_DC_CAL);
  TIM3->CNT = 0;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->BDTR = bdtr_old;
  TIM1->CCER = ccer_old;
  TIM1->DIER = dier_old;
  chThdSleepMilliseconds(5);
  ADC1->JOFR1 = j1/100;
  ADC2->JOFR1 = j2/100;
  
  g_enable_sampling = true;
}

void motor_get_currents(int *phase1_mA, int *phase3_mA)
{
  if (TIM1->BDTR & TIM_BDTR_MOE)
  {
    // Note: br1 is phase3 and br2 is phase1, phase2 is the sum.
    int16_t br1 = ADC1->JDR1;
    int16_t br2 = ADC2->JDR1;
    *phase1_mA = br2 * SHUNT_uA_PER_ADC_VAL / 1000;
    *phase3_mA = br1 * SHUNT_uA_PER_ADC_VAL / 1000;
  }
  else
  {
    // No info available while braking
    *phase1_mA = 0;
    *phase3_mA = 0;
  }
}

static float g_battery_current = 0.0f;
static float g_battery_voltage = 0.0f;
static float g_motor_temperature = 0.0f;
static float g_mosfet_temperature = 0.0f;

static float ntc_to_millicelsius(float ohms_25C, float beta, float adc_ohms)
{
  float T0 = 273.15f + 25.0f;
  float R0 = ohms_25C;
  float B = beta;
  float R = adc_ohms;
  
  float T = 1.0f / (1 / B * logf(R / R0) + 1 / T0);
  return 1000.0f * (T - 273.15f);
}

void motor_sampling_update()
{
  float decay = 1.0f / (MOTOR_FILTER_TIME_S * CONTROL_FREQ);

  if (!(TIM1->CR1 & TIM_CR1_CEN))
  {
    // Motor control not running
    decay = 0.01f;
    ADC1->CR2 |= ADC_CR2_JSWSTART;
  }
  
  int ph1_mA, ph3_mA;
  motor_get_currents(&ph1_mA, &ph3_mA);

  /* Estimate phase voltages */
  float div = 1.0f / TIM1->ARR;
  float ph3 = TIM1->CCR1 * div;
  float ph2 = TIM1->CCR2 * div;
  float ph1 = TIM1->CCR3 * div;
  float vgnd = (ph1 + ph2 + ph3) * (1 / 3.0f);
  ph1 -= vgnd;
  ph2 -= vgnd;
  ph3 -= vgnd;
  
  /* Estimate battery current */
  int ph2_mA = -(ph1_mA + ph3_mA);
  float battery_mA = ph1_mA * ph1 + ph2_mA * ph2 + ph3_mA * ph3;
  if (battery_mA < -20000) battery_mA = 0; // Fault state disables current reading
  
  motor_sampling_update_raw(battery_mA, ADC1->JDR2, ADC2->JDR2, decay);
}

void motor_sampling_update_raw(int battery_mA, int vbat_adc, int ntc_adc, float decay)
{
  g_battery_current = g_battery_current * (1 - decay) + battery_mA * decay;

  /* Estimate battery voltage */
  float mV = vbat_adc * 3300.0f / 4096 * (39.0f + 2.2f) / 2.2f;
  float esr_drop_mV = (g_battery_current / 1000.0f) * g_system_state.battery_esr_mohm;
  if (esr_drop_mV > 2000) esr_drop_mV = 2000;
  if (esr_drop_mV < -1000) esr_drop_mV = -1000;
  mV += esr_drop_mV;
  if (g_battery_voltage < BATTERY_MIN_VOLTAGE_B) g_battery_voltage = mV;
  g_battery_voltage = g_battery_voltage * (1 - decay) + mV * decay;

  /* Estimate MOSFET temperature */
  // adc_val = 4096 * 10k / (10k + ntc)
  // =>  ntc = (4096 * 10k) / adc_val - 10k
  float ohms = (4096.0f * 10000.0f) / ntc_adc - 10000.0f;
  float mC = ntc_to_millicelsius(10000, 3428, ohms);
  if (g_mosfet_temperature == 0) g_mosfet_temperature = mC;
  g_mosfet_temperature = g_mosfet_temperature * (1 - decay) + mC * decay;
}

int get_battery_current_mA()
{
  return (int)g_battery_current;
}

int get_battery_voltage_mV()
{
  return (int)g_battery_voltage;
}

int get_motor_temperature_mC()
{
  return 20000;
}

int get_mosfet_temperature_mC()
{
  return (int)g_mosfet_temperature;
}

/* Sample buffer for debugging */

static int8_t clamp(float x)
{
  int i = roundf(x);
  if (i > 127) i = 127;
  if (i < -127) i = -127;
  return i;
}

void motor_sampling_store()
{
  motor_sample_t sample = {};
  sample.motor_orientation = motor_orientation_get_angle() / 2;
  sample.hall_angle = motor_orientation_get_hall_angle() / 2;
  sample.ph3 = TIM1->CCR1 / 4;
  sample.ph2 = TIM1->CCR2 / 4;
  sample.ph1 = TIM1->CCR3 / 4;
  
  int phase1, phase3;
  motor_get_currents(&phase1, &phase3);
  sample.current_ph1 = clamp(phase1 / 100.0f);
  sample.current_ph3 = clamp(phase3 / 100.0f);
  
  float complex ivector, uvector;
  get_foc_debug(&ivector, &uvector);
  sample.ivector_r = clamp(crealf(ivector) / 100.0f);
  sample.ivector_i = clamp(cimagf(ivector) / 100.0f);
  sample.uvector_r = clamp(crealf(uvector) * 100.0f);
  sample.uvector_i = clamp(cimagf(uvector) * 100.0f);
  
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
  
  chprintf(stream, "#ORI HALL Ph1 Ph2 Ph3 I_Phase1 I_Phase3 I_real I_imag U_real U_imag\r\n");
  for (int i = 0; i < MOTOR_SAMPLE_COUNT; i++)
  {
    motor_sample_t sample = g_motor_samples[(i + g_motor_samples_writeidx) % MOTOR_SAMPLE_COUNT];
    
    char buf[128];
    chsnprintf(buf, sizeof(buf), "%4d %4d %4d %4d %4d %8d %8d %4d %4d %4d %4d\r\n",
               sample.motor_orientation * 2, sample.hall_angle * 2,
               sample.ph1 * 4, sample.ph2 * 4, sample.ph3 * 4,
               sample.current_ph1 * 100, sample.current_ph3 * 100,
               sample.ivector_r, sample.ivector_i,
               sample.uvector_r, sample.uvector_i);
    chSequentialStreamWrite(stream, (void*)buf, strlen(buf));
  }
  
  g_enable_sampling = true;
}


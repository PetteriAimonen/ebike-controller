#include <ch.h>
#include <hal.h>
#include <stm32f4xx.h>
#include <math.h>
#include "dcdc_control.h"
#include "motor_sampling.h"
#include "motor_config.h"

#undef PWM_FREQ
#undef CONTROL_FREQ
#undef PWM_MAX
#undef PWM_MAX_DUTY

#define PWM_FREQ 100000
#define CONTROL_FREQ 21000
#define PWM_MAX  1680
#define PWM_MAX_DUTY 1600

#define BOOST_RATIO 2.5f

bool g_dcdc_active;
static int g_dcdc_mV;
static int g_dcdc_mA;
static int g_dcdc_duty;
static float g_dcdc_pid_i;
static bool g_dcdc_boost_mode;

#define ADCBUFLEN 32
static uint16_t g_dcdc_adc_buf[ADCBUFLEN * 3];
static int g_dcdc_ph3_offset_mA;

void set_dcdc_mode(int mV, int mA)
{
    g_dcdc_mV = mV;
    g_dcdc_mA = mA;
}

static int dcdc_ph3_current_mA()
{
  int sum = 0;
  for (int i = 0; i < ADCBUFLEN; i++)
  {
    sum += g_dcdc_adc_buf[i * 3];
  }

  int result = sum * SHUNT_uA_PER_ADC_VAL / (1000 * ADCBUFLEN) - g_dcdc_ph3_offset_mA;
  if (result < -20000) result = 0; // Current sense off
  return result;
}

// Get output current (from the PH3 ground return current)
// Positive = power is going out
int get_dcdc_current_mA() { return -dcdc_ph3_current_mA(); }

static int dcdc_vbat_adc()
{
  int sum = 0;
  for (int i = 0; i < ADCBUFLEN; i++)
  {
    sum += g_dcdc_adc_buf[i * 3 + 1];
  }
  return sum / ADCBUFLEN;
}

static int dcdc_ntc_adc()
{
  int sum = 0;
  for (int i = 0; i < ADCBUFLEN; i++)
  {
    sum += g_dcdc_adc_buf[i * 3 + 2];
  }
  return sum / ADCBUFLEN;
}

// Initialize ADC in free-run mode.
// This gets a more accurate average than PWM-synchronized sampling.
static void dcdc_adc_init()
{
  // Single ADC mode with regular conversion.
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC->CCR = STM32_ADC_ADCPRE;
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 = ADC_CR2_ADON;

  // 15 cycle sampling for currents, 56 cycles for voltage & temperature
  ADC1->SMPR1 = 0111111111;
  ADC1->SMPR2 = 0111111111;
  ADC1->SMPR1 |= (3 << 6); // Ch12
  ADC1->SMPR2 |= (3 << 9); // Ch3

  // ADC1 regular sequence: Ch9/BR_SO1, Ch12/AN_IN, Ch3/ADC_TEMP
  // ADC2 regular sequence: Ch8/BR_SO2,
  ADC1->SQR1 = ADC_SQR1_L_1;
  ADC1->SQR3 = (9 << 0) | (12 << 5) | (3 << 10);

  // Circular DMA to memory buffer
  const stm32_dma_stream_t* dma = STM32_DMA2_STREAM4;
  dmaStreamAllocate(dma, 10, NULL, NULL);
  dmaStreamSetPeripheral(dma, &(ADC1->DR));
  dmaStreamSetMemory0(dma, &g_dcdc_adc_buf);
  dmaStreamSetTransactionSize(dma, ADCBUFLEN * 3);
  dmaStreamSetMode(dma,
    STM32_DMA_CR_CHSEL(0) | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD |
    STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(2));
  dmaStreamEnable(dma);

  // Regular sequence free run
  ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
  ADC1->SR = 0;
  ADC1->CR2 |= ADC_CR2_SWSTART;

  // Calculate DC offset
  palSetPad(GPIOA, GPIOA_DC_CAL);
  g_dcdc_ph3_offset_mA = 0;

  chThdSleepMilliseconds(50);

  // Take average of 100 samples
  int ph3_sum = 0;
  for (int i = 0; i < 100; i++)
  {
    ph3_sum += dcdc_ph3_current_mA();
    chThdSleepMilliseconds(1);
  }

  palClearPad(GPIOA, GPIOA_DC_CAL);
  g_dcdc_ph3_offset_mA = ph3_sum / 100;
}

int g_irqcount = 0;

CH_FAST_IRQ_HANDLER(STM32_TIM3_HANDLER)
{
    static bool first = true;

    g_irqcount++;

    TIM3->SR &= ~TIM_SR_CC2IF;

    if (ADC1->SR & ADC_SR_OVR)
    {
      ADC1->SR = 0;
      ADC2->SR = 0;
      ADC1->CR2 |= ADC_CR2_SWSTART;
    }

    float decay = 1.0f / (MOTOR_FILTER_TIME_S * CONTROL_FREQ);
    if (first)
    {
      decay = 1.0f;
      first = false;
    }

    // Estimate battery current
    int output_mA = get_dcdc_current_mA();
    int battery_mA = 0;
    if (!g_dcdc_boost_mode)
    {
      battery_mA = TIM1->CCR3 / (float)TIM1->ARR * output_mA;
    }
    else
    {
      battery_mA = output_mA + TIM1->CCR2 / (float)TIM1->ARR * output_mA / BOOST_RATIO;
    }

    // Update motor sampling data so that it is available to logging
    // and other tasks.
    int vbat_adc = dcdc_vbat_adc();
    int ntc_adc = dcdc_ntc_adc();
    motor_sampling_update_raw(battery_mA, vbat_adc, ntc_adc, decay);

    // Do PI control to limit output current
    int target_mV = g_dcdc_mV;
    if (g_dcdc_mA > 0)
    {
      int error = output_mA - g_dcdc_mA;
      if (error < -10) error = -10;
      target_mV -= g_dcdc_pid_i + error / 2;
      g_dcdc_pid_i += error / 100.0f;
      if (g_dcdc_pid_i < 0) g_dcdc_pid_i = 0;
      if (g_dcdc_pid_i > g_dcdc_mV) g_dcdc_pid_i = g_dcdc_mV;
    }
    else if (g_dcdc_mA < 0)
    {
      int error = -(output_mA - g_dcdc_mA);
      if (error < -10) error = -10;
      target_mV += g_dcdc_pid_i + error / 2;
      g_dcdc_pid_i += error / 100.0f;
      if (g_dcdc_pid_i < 0) g_dcdc_pid_i = 0;
      if (g_dcdc_pid_i > g_dcdc_mV) g_dcdc_pid_i = g_dcdc_mV;
    }

    // Clamp output voltage setting
    if (target_mV < 0) target_mV = 0;
    if (target_mV > 44000) target_mV = 44000;

    // Duty from DC voltage
    int battery_mV = get_battery_voltage_mV();
    int max_mV = battery_mV * PWM_MAX_DUTY / PWM_MAX;
    int phase, duty;

    if (!g_dcdc_boost_mode && target_mV > max_mV + 500)
    {
      g_dcdc_boost_mode = true;
    }
    else if (g_dcdc_boost_mode && target_mV < max_mV - 1000)
    {
      g_dcdc_boost_mode = false;
    }

    if (g_dcdc_boost_mode)
    {
      // Use output boost mode (PH1 stays high and PH2 toggles)
      phase = 2;
      duty = (target_mV - battery_mV) * PWM_MAX * BOOST_RATIO / battery_mV;
      if (duty > PWM_MAX / 2) duty = PWM_MAX / 2;
    }
    else
    {
      // Use in/out buck mode (PH1 toggles, PH2 floats)
      phase = 1;
      duty = PWM_MAX * target_mV / battery_mV;
    }

    if (duty < 0) duty = 0;
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;

    if (output_mA < -100 && output_mA < g_dcdc_mA - 200)
    {
      // Skip pulses to regulate current in input mode
      phase = 0;
    }
    else if (output_mA > 100 && g_dcdc_mA < 0)
    {
      // Skip pulses to avoid output current when configured for input
      phase = 0;
    }

    int temperature_mC = get_mosfet_temperature_mC();

    if (battery_mV < 30000 || battery_mV >= 44000 || temperature_mC > 90000 || output_mA > 10000 || output_mA < -10000)
    {
        // Disable outputs
        TIM1->BDTR &= ~TIM_BDTR_MOE;

        // Retry after 10 ms
        TIM3->CCR1 = 1;
        TIM3->CNT = CONTROL_FREQ / 100;
        TIM3->CR1 |= TIM_CR1_CEN;
    }
    else
    {
        if (phase == 1)
        {
          // Buck mode: phase 1 (CCR3) toggles, phase 2 float, phase 3 GND
          TIM1->CCR1 = 0;
          TIM1->CCR2 = 0;
          TIM1->CCR3 = duty;
          TIM1->CCER = 0x0505;
        }
        else if (phase == 2)
        {
          // Boost mode: phase 1 high, phase 2 toggles, phase 3 ground
          TIM1->CCR1 = 0;
          TIM1->CCR3 = PWM_MAX;
          TIM1->CCR2 = duty;
          TIM1->CCER = 0x0555;
        }
        else
        {
          // Skip cycles when current limit exceeded in current-in mode
          // Disabling phases 1 & 2 (CCR3 & CCR2) output lets the main inductor float at whatever voltage it is at.
          // There will be some initial voltage peak that will be absorbed by the MOSFET diodes.
          TIM1->CCR1 = 0;
          TIM1->CCR2 = 0;
          TIM1->CCR3 = 0;
          TIM1->CCER = 0x0005;
        }

        TIM1->BDTR |= TIM_BDTR_MOE;
        TIM3->CCR1 = 1;
        TIM3->CNT = 10;
        TIM3->CR1 |= TIM_CR1_CEN;
    }
}

void start_dcdc_control()
{
    g_dcdc_active = true;
    g_dcdc_mA = 0;
    g_dcdc_mV = 0;

    // Enable timer clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Timer config for edge-aligned PWM
    TIM1->CR1 = 0;
    TIM1->CR2 = 0; // All outputs off when MOE=0.
    TIM1->DIER = 0;
    TIM1->CCMR1 = 0x6868;
    TIM1->CCMR2 = 0x6068;
    TIM1->CCER = 0x0555;
    TIM1->CNT = 0;
    TIM1->PSC = STM32_TIMCLK2 / (PWM_FREQ * PWM_MAX) - 1;
    TIM1->ARR = PWM_MAX - 1;
    TIM1->RCR = 0;
    TIM1->BDTR = 80; // 0.2µs dead time
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    TIM1->EGR = TIM_EGR_UG;

    // TIM3 CH1 is used for the gate enable signal. This way the mosfets will be
    // disabled automatically if the code is e.g. stopped in a debugger.
    // It is setup to count downwards at CONTROL_FREQ, if it reaches zero
    // it will disable EN_GATE.
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CR1 = TIM_CR1_OPM | TIM_CR1_DIR;
    TIM3->CCMR1 = 0x0070;
    TIM3->CCR1 = 1;
    TIM3->CCR2 = 9; // Triggers IRQ for DC-DC control loop
    TIM3->CCER = TIM_CCER_CC1E;
    TIM3->PSC = STM32_TIMCLK1 / CONTROL_FREQ - 1;
    TIM3->ARR = 1; // Value the timer stops at in one pulse mode
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CNT = CONTROL_FREQ; // For DC offset calibration
    TIM3->DIER = 0;
    TIM3->CR1 |= TIM_CR1_CEN;

    // Enable interrupt for performing DCDC control
    TIM3->DIER = TIM_DIER_CC2IE;
    nvicEnableVector(STM32_TIM3_NUMBER, 0);

    // Start the timer
    TIM3->CNT = 10;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;

    // Initialize the ADC sampling.
    dcdc_adc_init();
}

void stop_dcdc_control()
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->DIER = 0;
  nvicDisableVector(STM32_TIM3_NUMBER);
  TIM3->CNT = 0;
}

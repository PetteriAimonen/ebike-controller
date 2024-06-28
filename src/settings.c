#include "settings.h"
#include <hal.h>
#include <stm32f4xx.h>
#include <string.h>
#include "debug.h"

volatile system_state_t g_system_state;
static const system_state_store_t *g_latest_saved;

extern const system_state_store_t __settings_start__;
extern const system_state_store_t __settings_end__;

static void flash_erase_sector(int sector)
{
  // Disable interrupts and motor control during erase operation
  __disable_irq();
  TIM3->CCR1 = 10000;
  TIM3->CNT = 9999;

  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | (sector << 3);
  FLASH->CR |= FLASH_CR_STRT;
  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->CR |= FLASH_CR_LOCK;

  // Next call to motor_run will re-enable control by setting TIM3->CCR1
  TIM3->CNT = 10;
  __enable_irq();
}

static void flash_write(uint32_t *flash, uint32_t *data, int len)
{
  int num_words = (len + 3) / 4;
  for (int i = 0; i < num_words; i++)
  {
    __disable_irq();
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    
    flash[i] = data[i];
    while (FLASH->SR & FLASH_SR_BSY);

    FLASH->CR |= FLASH_CR_LOCK;
    __enable_irq();
  }
}

uint32_t compute_checksum(const system_state_store_t *state)
{
  uint32_t checksum = 0xAAAA;
  
  for (int i = sizeof(state->state.checksum); i < sizeof(*state); i++)
  {
    checksum ^= state->raw[i];
    checksum ^= checksum << 13;
	  checksum ^= checksum >> 17;
	  checksum ^= checksum << 5;
  }

  return checksum;
}

bool check_empty(const system_state_store_t *location)
{
  const uint32_t *data = (const uint32_t*)location;
  for (int i = 0; i < sizeof(system_state_store_t) / 4; i++)
  {
    if (data[i] != 0xFFFFFFFF) return false;
  }

  return true;
}

void load_system_state()
{
  // Find latest struct revision
  const system_state_store_t *p = &__settings_start__;
  const system_state_store_t *end = &__settings_end__;
  const system_state_store_t *latest = NULL;

  while (p < end)
  {
    if (!latest || p->state.revision > latest->state.revision)
    {
      if (compute_checksum(p) == p->state.checksum)
      {
        latest = p;
      }
    }

    p++;
  }

  if (latest == NULL)
  {
    // Flash is empty
    memset((void*)&g_system_state, 0, sizeof(system_state_t));
    g_system_state.revision = 1;
    g_latest_saved = end - 1;
  }
  else
  {
    memcpy((void*)&g_system_state, &latest->state, sizeof(system_state_t));
    g_latest_saved = latest;
  }

  // Fill in any missing default values
  if (g_system_state.min_voltage_V == 0) g_system_state.min_voltage_V = 33;
  if (g_system_state.max_motor_current_A == 0) g_system_state.max_motor_current_A = 15;
  if (g_system_state.max_battery_current_A == 0) g_system_state.max_battery_current_A = 9;
  if (g_system_state.wheel_diameter_inch == 0) g_system_state.wheel_diameter_inch = 28;
  if (g_system_state.bike_weight_kg == 0) g_system_state.bike_weight_kg = 100;
  if (g_system_state.wheel_speed_ticks_per_rotation == 0) g_system_state.wheel_speed_ticks_per_rotation = 6;
  if (g_system_state.max_speed_kmh == 0) g_system_state.max_speed_kmh = 25;
  if (g_system_state.torque_N_per_A == 0) g_system_state.torque_N_per_A = 5;
  if (g_system_state.max_krpm == 0) g_system_state.max_krpm = 10;
  if (g_system_state.accel_time == 0) g_system_state.accel_time = 5;
  if (g_system_state.battery_esr_mohm == 0) g_system_state.battery_esr_mohm = 100;
  if (g_system_state.battery_capacity_Wh == 0) g_system_state.battery_capacity_Wh = 100;
}

void save_system_state()
{
  system_state_store_t to_save = {};
  __disable_irq();
  to_save.state = g_system_state;
  __enable_irq();

  if (memcmp(&g_latest_saved->state, &to_save.state, sizeof(system_state_t)) == 0)
  {
    // Nothing to do
    return;
  }

  g_system_state.revision++;
  to_save.state.revision++;
  to_save.state.checksum = compute_checksum(&to_save);

  const system_state_store_t *start = &__settings_start__;
  const system_state_store_t *end = &__settings_end__;
  const system_state_store_t *save_pos = g_latest_saved + 1;
  if (save_pos >= end) save_pos = start;

  // Check if we need to erase
  if (!check_empty(save_pos))
  {
    if ((uint32_t)save_pos >= 0x08004000 && (uint32_t)save_pos < 0x08008000)
    {
      flash_erase_sector(1);
    }
    else if ((uint32_t)save_pos >= 0x08008000 && (uint32_t)save_pos < 0x0800C000)
    {
      flash_erase_sector(2);
    }
    else
    {
      abort_with_error("ERASE %08x", (unsigned)save_pos);
    }
  }

  // Program to flash
  flash_write((uint32_t*)save_pos, (uint32_t*)&to_save, sizeof(to_save));
  g_latest_saved = save_pos;
}

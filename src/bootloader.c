/* Simple bootloader that allows loading of new firmware over bluetooth,
 * and backing up a known-good firmware version to second flash bank. */

#include <hal.h>
#include <stm32f4xx.h>
#include "board.h"

void bootloader_crash();
void bootloader_main();
void bootloader_init_usart();
void bootloader_print(const char *str);
char bootloader_get_button();
void bootloader_erase_sector(int sector);
void bootloader_write_flash(uint32_t *flash, uint32_t *data, int len);
void bootloader_write_backup();
void bootloader_restore_backup();
void bootloader_load_firmware();

const uint32_t bootloader_vectors[4] __attribute__((section(".vectors"))) = {
  0x10004000,
  (uint32_t)&bootloader_main,
  (uint32_t)&bootloader_crash,
  (uint32_t)&bootloader_crash
};

void bootloader_crash()
{
  bootloader_print("\r\nCRASH\r\n");
  for(;;);
}

extern uint32_t __backup_start__[];
extern uint32_t __mainprogram_start__[];

void bootloader_init_usart()
{
  RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  
  GPIOC->ODR = VAL_GPIOC_ODR;
  GPIOC->OTYPER = VAL_GPIOC_OTYPER;
  GPIOC->AFRL = VAL_GPIOC_AFRL;
  GPIOC->AFRH = VAL_GPIOC_AFRH;
  GPIOC->MODER = VAL_GPIOC_MODER;
  
  USART6->BRR = 16000000 / 115200;
  USART6->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void bootloader_print(const char *str)
{
  while (*str)
  {
    while (!(USART6->SR & USART_SR_TXE));
    USART6->DR = *str++;
  }
}

char bootloader_get_button()
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SQR3 = 11;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  
  while (!(ADC1->SR & ADC_SR_EOC));
  
  int adc = ADC1->DR;
  
  char buf[4] = {
    '0' + ((adc / 1000) % 10),
    '0' + ((adc / 100) % 10),
    '0' + ((adc / 10) % 10),
    '0' + ((adc / 1) % 10)
  };
  bootloader_print("Button: ");
  bootloader_print(buf);
  
  char button;
  if (adc < 2500)
    button = ' '; // No button pressed
  else if (adc < 2600)
    button = '-'; // Minus button
  else if (adc < 2800)
    button = '+';  // Plus button
  else
    button = 'K'; // OK button
  
  char buf2[2] = {button, 0};
  bootloader_print(" = '");
  bootloader_print(buf2);
  bootloader_print("'\r\n");
  
  return button;
}

void bootloader_erase_sector(int sector)
{
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  
  FLASH->CR = FLASH_CR_SER | (sector << 3);
  FLASH->CR |= FLASH_CR_STRT;
  
  while (FLASH->SR & FLASH_SR_BSY);
  
  FLASH->CR |= FLASH_CR_LOCK;
}

void bootloader_write_flash(uint32_t *flash, uint32_t *data, int len)
{
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  
  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  int num_words = (len + 3) / 4;
  for (int i = 0; i < num_words; i++)
  {
    flash[i] = data[i];
    while (FLASH->SR & FLASH_SR_BSY);
  }
  
  FLASH->CR |= FLASH_CR_LOCK;
}

void bootloader_write_backup()
{
  bootloader_erase_sector(6);
  bootloader_write_flash(__backup_start__, __mainprogram_start__, 128 * 1024);
  bootloader_print("Backup bank written\r\n");
}

void bootloader_restore_backup()
{
  if ((__backup_start__[1] >> 24) != 0x08)
	return;

  bootloader_erase_sector(5);
  bootloader_write_flash(__mainprogram_start__, __backup_start__, 128 * 1024);
  bootloader_print("Backup firmware restored\r\n");
}

void bootloader_load_firmware()
{
  uint8_t *tmp = (uint8_t*)0x20000000;
  
  while (USART6->DR != '\n');
  
  bootloader_print("Ready to receive .bin\r\n");
  
  // Clear any buffered data
  for (int i = 0; i < 10000; i++)
    (void)USART6->DR;
  
  (void)USART6->SR;
  
  int num_bytes = 0;
  bool done = false;
  while (num_bytes < 128 * 1024 && !done)
  {
    int wait = 0;
    while (!(USART6->SR & USART_SR_RXNE))
    {
      wait++;
      if (wait > 1000000 && num_bytes > 0)
      {
        done = true;
        break;
      }
    }
    tmp[num_bytes++] = USART6->DR;
    if (num_bytes % 1024 == 0)
      USART6->DR = '.';
  }
  
  bootloader_print("\r\nFirmware loaded to RAM, starting flash programming..\r\n");
  for (int i = 1; i <= 5; i++)
  {
    bootloader_erase_sector(i);
  }
  bootloader_write_flash(__mainprogram_start__, (uint32_t*)(tmp + 0x4000), num_bytes - 0x4000);
  bootloader_print("Flash written.\r\n");
}

void bootloader_main()
{
  bootloader_init_usart();
  bootloader_print("\r\nBootloader\r\n");

  char button = bootloader_get_button();
  char button2 = bootloader_get_button();

  while (button != button2)
  {
     button = bootloader_get_button();
     button2 = bootloader_get_button();
  }
  
  if (button == '+')
  {
    bootloader_write_backup();
  }
  else if (button == '-')
  {
    bootloader_restore_backup();
  }
  else if (button == 'K')
  {
    // Jump to DFU bootloader
    uint32_t* sysmem = (uint32_t*)0x1FFF0000;
    uint32_t stack_ptr = sysmem[0];
    uint32_t reset_vector = sysmem[1];

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->MEMRMP = 1;

    __asm__(
      "msr msp, %0\n\t"
      "bx %1" : : "r" (stack_ptr),
                  "r" (reset_vector) : "memory");
  }
  
  /* Now continue to the main application */
  SCB->VTOR = (uint32_t)__mainprogram_start__;
  
  __asm__(
    "msr msp, %0\n\t"
    "bx %1" : : "r" (__mainprogram_start__[0]),
                "r" (__mainprogram_start__[1]) : "memory");
}

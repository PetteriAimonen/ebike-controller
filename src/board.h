/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F405xx

#define BOARD_OTG_NOVBUSSENS 1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2U))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2U))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/* X-macro table of all the pins */
#define GPIOA_PINS(X) \
X(GPIOA,  0, SENS3       , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOA,  1, SENS2       , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOA,  2, SENS1       , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOA,  3, ADC_TEMP    , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOA,  4, SPI1_CS     , OUTPUT     , HIGH, PUSHPULL ,  50M , PULLUP ,  0) \
X(GPIOA,  5, SPI1_SCK    , ALTERNATE  , LOW , PUSHPULL ,  50M , PULLUP ,  5) \
X(GPIOA,  6, SPI1_MISO   , ALTERNATE  , LOW , PUSHPULL ,  50M , PULLUP ,  5) \
X(GPIOA,  7, SPI1_MOSI   , ALTERNATE  , LOW , PUSHPULL ,  50M , PULLUP ,  5) \
X(GPIOA,  8, H3          , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  1) \
X(GPIOA,  9, H2          , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  1) \
X(GPIOA, 10, H1          , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  1) \
X(GPIOA, 11, USB_DM      , ALTERNATE  , LOW , PUSHPULL , 100M , FLOATING , 10) \
X(GPIOA, 12, USB_DP      , ALTERNATE  , LOW , PUSHPULL , 100M , FLOATING , 10) \
X(GPIOA, 13, SWDIO       , ALTERNATE  , LOW , PUSHPULL , 100M , FLOATING ,  0) \
X(GPIOA, 14, SWCLK       , ALTERNATE  , LOW , PUSHPULL , 100M , FLOATING ,  0) \
X(GPIOA, 15, DC_CAL      , OUTPUT     , LOW , PUSHPULL ,  50M , FLOATING ,  0)

#define GPIOB_PINS(X) \
X(GPIOB,  0, BR_SO2      , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOB,  1, BR_SO1      , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOB,  2, BOOT1_NC    , INPUT      , LOW , PUSHPULL ,  50M , PULLDOWN ,  0) \
X(GPIOB,  3, TRACESWO    , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOB,  4, EN_GATE     , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  2) \
X(GPIOB,  5, SENS_IRQ    , INPUT      , LOW , PUSHPULL ,  50M , PULLUP   ,  0) \
X(GPIOB,  6, HALL_1      , INPUT      , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOB,  7, HALL_2      , INPUT      , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOB,  8, FAULT       , INPUT      , LOW , PUSHPULL ,  50M , PULLUP ,  0) \
X(GPIOB,  9, NC          , INPUT      , LOW , PUSHPULL ,  50M , PULLUP   ,  0) \
X(GPIOB, 10, I2C_SCL     , ALTERNATE  , HIGH, OPENDRAIN,  50M , FLOATING ,  4) \
X(GPIOB, 11, I2C_SDA     , ALTERNATE  , HIGH, OPENDRAIN,  50M , FLOATING ,  4) \
X(GPIOB, 12, BRAKE       , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  1) \
X(GPIOB, 13, L3          , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  1) \
X(GPIOB, 14, L2          , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  1) \
X(GPIOB, 15, L1          , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  1)

#define GPIOC_PINS(X) \
X(GPIOC,  0, WHEEL_SPEED , INPUT      , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOC,  1, BUTTONS     , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOC,  2, AN_IN       , ANALOG     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOC,  3, EXT1        , INPUT      , LOW , PUSHPULL ,  50M , PULLUP   ,  0) \
X(GPIOC,  4, LED_GREEN   , OUTPUT     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOC,  5, LED_RED     , OUTPUT     , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOC,  6, BT_RX       , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  8) \
X(GPIOC,  7, BT_TX       , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  8) \
X(GPIOC,  8, EXT2        , INPUT      , LOW , PUSHPULL ,  50M , PULLUP   ,  0) \
X(GPIOC,  9, EXT3        , OUTPUT     , HIGH , OPENDRAIN ,   2M , FLOATING ,  0) \
X(GPIOC, 10, SPI3_SCK    , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  6) \
X(GPIOC, 11, SPI3_MISO   , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  6) \
X(GPIOC, 12, SPI3_MOSI   , ALTERNATE  , LOW , PUSHPULL ,  50M , FLOATING ,  6) \
X(GPIOC, 13, HALL_3      , INPUT      , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOC, 14, NC1         , INPUT      , LOW , PUSHPULL ,  50M , PULLUP   ,  0) \
X(GPIOC, 15, NC2         , INPUT      , LOW , PUSHPULL ,  50M , PULLUP   ,  0)

#define GPIOD_PINS(X) \
X(GPIOD,  2, SPI3_CS     , OUTPUT     , LOW , PUSHPULL ,  50M , FLOATING ,  0)

#define GPIOE_PINS(X)
#define GPIOF_PINS(X)
#define GPIOG_PINS(X)

#define GPIOH_PINS(X) \
X(GPIOH,  0, OSC1        , INPUT      , LOW , PUSHPULL ,  50M , FLOATING ,  0) \
X(GPIOH,  1, OSC2        , INPUT      , LOW , PUSHPULL ,  50M , FLOATING ,  0)

#define GPIOI_PINS(X)

/* Create pin number constants */
enum GPIO_PIN_NUMBERS {
#define pindef(port, pin, name, mode, odr, otype, ospeed, pupdr, af) \
  port ## _ ## name = pin,
GPIOA_PINS(pindef)
GPIOB_PINS(pindef)
GPIOC_PINS(pindef)
GPIOD_PINS(pindef)
GPIOE_PINS(pindef)
GPIOF_PINS(pindef)
GPIOG_PINS(pindef)
GPIOH_PINS(pindef)
GPIOI_PINS(pindef)
#undef pindef
};


/* Create macros for GPIO port register config */
#define GPIO_MODER_VALUE(port, pin, name, mode, odr, otype, ospeed, pupdr, af)  | PIN_MODE_ ## mode(pin)
#define GPIO_ODR_VALUE(port, pin, name, mode, odr, otype, ospeed, pupdr, af)    | PIN_ODR_ ## odr(pin)
#define GPIO_OTYPE_VALUE(port, pin, name, mode, odr, otype, ospeed, pupdr, af)  | PIN_OTYPE_ ## otype(pin)
#define GPIO_OSPEED_VALUE(port, pin, name, mode, odr, otype, ospeed, pupdr, af) | PIN_OSPEED_ ## ospeed(pin)
#define GPIO_PUPDR_VALUE(port, pin, name, mode, odr, otype, ospeed, pupdr, af)  | PIN_PUPDR_ ## pupdr(pin)
#define GPIO_AFRL_VALUE(port, pin, name, mode, odr, otype, ospeed, pupdr, af)   | ((pin < 8) ? PIN_AFIO_AF(pin, af) : 0)
#define GPIO_AFRH_VALUE(port, pin, name, mode, odr, otype, ospeed, pupdr, af)   | ((pin >= 8) ? PIN_AFIO_AF(pin, af) : 0)

#define VAL_GPIOA_MODER   (0 GPIOA_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOA_ODR     (0 GPIOA_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOA_OTYPER  (0 GPIOA_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOA_OSPEEDR (0 GPIOA_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOA_PUPDR   (0 GPIOA_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOA_AFRL    (0 GPIOA_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOA_AFRH    (0 GPIOA_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOB_MODER   (0 GPIOB_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOB_ODR     (0 GPIOB_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOB_OTYPER  (0 GPIOB_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOB_OSPEEDR (0 GPIOB_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOB_PUPDR   (0 GPIOB_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOB_AFRL    (0 GPIOB_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOB_AFRH    (0 GPIOB_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOC_MODER   (0 GPIOC_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOC_ODR     (0 GPIOC_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOC_OTYPER  (0 GPIOC_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOC_OSPEEDR (0 GPIOC_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOC_PUPDR   (0 GPIOC_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOC_AFRL    (0 GPIOC_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOC_AFRH    (0 GPIOC_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOD_MODER   (0 GPIOD_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOD_ODR     (0 GPIOD_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOD_OTYPER  (0 GPIOD_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOD_OSPEEDR (0 GPIOD_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOD_PUPDR   (0 GPIOD_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOD_AFRL    (0 GPIOD_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOD_AFRH    (0 GPIOD_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOE_MODER   (0 GPIOE_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOE_ODR     (0 GPIOE_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOE_OTYPER  (0 GPIOE_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOE_OSPEEDR (0 GPIOE_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOE_PUPDR   (0 GPIOE_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOE_AFRL    (0 GPIOE_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOE_AFRH    (0 GPIOE_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOF_MODER   (0 GPIOF_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOF_ODR     (0 GPIOF_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOF_OTYPER  (0 GPIOF_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOF_OSPEEDR (0 GPIOF_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOF_PUPDR   (0 GPIOF_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOF_AFRL    (0 GPIOF_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOF_AFRH    (0 GPIOF_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOG_MODER   (0 GPIOG_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOG_ODR     (0 GPIOG_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOG_OTYPER  (0 GPIOG_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOG_OSPEEDR (0 GPIOG_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOG_PUPDR   (0 GPIOG_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOG_AFRL    (0 GPIOG_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOG_AFRH    (0 GPIOG_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOH_MODER   (0 GPIOH_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOH_ODR     (0 GPIOH_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOH_OTYPER  (0 GPIOH_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOH_OSPEEDR (0 GPIOH_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOH_PUPDR   (0 GPIOH_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOH_AFRL    (0 GPIOH_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOH_AFRH    (0 GPIOH_PINS(GPIO_AFRH_VALUE))

#define VAL_GPIOI_MODER   (0 GPIOI_PINS(GPIO_MODER_VALUE))
#define VAL_GPIOI_ODR     (0 GPIOI_PINS(GPIO_ODR_VALUE))
#define VAL_GPIOI_OTYPER  (0 GPIOI_PINS(GPIO_OTYPE_VALUE))
#define VAL_GPIOI_OSPEEDR (0 GPIOI_PINS(GPIO_OSPEED_VALUE))
#define VAL_GPIOI_PUPDR   (0 GPIOI_PINS(GPIO_PUPDR_VALUE))
#define VAL_GPIOI_AFRL    (0 GPIOI_PINS(GPIO_AFRL_VALUE))
#define VAL_GPIOI_AFRH    (0 GPIOI_PINS(GPIO_AFRH_VALUE))

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */

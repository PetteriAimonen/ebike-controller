/* From https://github.com/joewa/WS2812-LED-Driver_ChibiOS */

/**
 * @file    ws2812.c
 * @author  Austin Glaser <austin.glaser@gmail.com>, Joerg Wangemann <joerg.wangemann@gmail.com>
 * @brief   WS2812 LED driver
 *
 * Copyright (C) 2016 Austin Glaser, 2017 Joerg Wangemann
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 *
 * @todo    Put in names and descriptions of variables which need to be defined to use this file
 *
 * @addtogroup WS2812
 * @{
 */

/* --- PRIVATE DEPENDENCIES ------------------------------------------------- */

// This Driver
#include "ws2812.h"

// Standard
#include <stdint.h>

// ChibiOS
#include "ch.h"
#include "hal.h"

// Application
#include "board.h"

#define CONCAT_SYMBOLS(s1, s2)     s1##s2
#define CONCAT_EXPANDED_SYMBOLS(s1, s2)  CONCAT_SYMBOLS(s1, s2)

// TODO: Add these #define's to the headers of your project.
// Pin, timer and dma are all connected, check them all if you change one.
// Tested with STM32F4, working at 144 or 168 MHz.
#define WS2812_LED_N    54 // Number of LEDs
#define PORT_WS2812     GPIOC
#define PIN_WS2812      9
#define WS2812_TIM_N    8  // timer, 1-11
#define WS2812_TIM_CH   3  // timer channel, 0-3
#define WS2812_DMA_STREAM STM32_DMA2_STREAM1  // DMA stream for TIMx_UP (look up in reference manual under DMA Channel selection)
#define WS2812_DMA_CHANNEL 7                  // DMA channel for TIMx_UP
#define WS2812_COLOR_N  3  // 3 for RGB, 4 for RGBW (e.g. SK6812)
// The WS2812 expects 5V signal level (or at least 0.7 * VDD). Sometimes it works
// with a 3V signal level, otherwise the easiest way to get the signal level to 5V
// is to add an external pullup resistor from the DI pin to 5V (10k will do) and
// configure the pin as open drain.
// (An SMD resistor is easily soldered on the connections of a light strip)
// Uncomment the next line if an external pullup resistor is used.
#define WS2812_EXTERNAL_PULLUP

/* --- CONFIGURATION CHECK -------------------------------------------------- */

#if !defined(WS2812_LED_N)
    #error WS2812 LED chain length not specified
#elif WS2812_LED_N <= 0
    #error WS2812 LED chain length set to invalid value
#endif

#if !defined(WS2812_TIM_N)
    #error WS2812 timer not specified
#endif
#if defined(STM32F2XX) || defined(STM32F4XX) || defined(STM32F7XX)
    #if WS2812_TIM_N <= 2
        #define WS2812_AF 1
    #elif WS2812_TIM_N <= 5
        #define WS2812_AF 2
    #elif WS2812_TIM_N <= 11
        #define WS2812_AF 3
    #endif
#elif !defined(WS2812_AF)
    #error WS2812_AF timer alternate function not specified
#endif

#if !defined(WS2812_TIM_CH)
    #error WS2812 timer channel not specified
#elif WS2812_TIM_CH >= 4
    #error WS2812 timer channel set to invalid value
#endif

/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define WS2812_PWM_FREQUENCY    (STM32_SYSCLK/2)                /**< Clock frequency of PWM, must be valid with respect to system clock! */
#define WS2812_PWM_PERIOD       (WS2812_PWM_FREQUENCY/800000)   /**< Clock period in ticks. 1 / 800kHz = 1.25 uS */

/**
 * @brief   Number of bit-periods to hold the data line low at the end of a frame
 *
 * The reset period for each frame must be at least 50 uS; so we add in 50 bit-times
 * of zeroes at the end. (50 bits)*(1.25 uS/bit) = 62.5 uS, which gives us some
 * slack in the timing requirements
 * 
 * NOTE: Adapted for WS2812B, reset time 280 µs, 256 bits
 */
#define WS2812_COLOR_BITS       (WS2812_COLOR_N*8)
#define WS2812_RESET_BIT_N      (500)
#define WS2812_COLOR_BIT_N      (WS2812_LED_N*WS2812_COLOR_BITS)            /**< Number of data bits */
#define WS2812_BIT_N            (WS2812_COLOR_BIT_N + WS2812_RESET_BIT_N)   /**< Total number of bits in a frame */

/**
 * @brief   High period for a zero, in ticks
 *
 * Per the datasheet:
 * WS2812:
 * - T0H: 200 nS to 500 nS, inclusive
 * - T0L: 650 nS to 950 nS, inclusive
 * WS2812B:
 * - T0H: 200 nS to 500 nS, inclusive
 * - T0L: 750 nS to 1050 nS, inclusive
 * SK6812:
 * - T0H: 150 nS to 450 nS, inclusive
 * - T0L: 750 nS to 1050 nS, inclusive
 *
 * The duty cycle is calculated for a high period of 350 nS.
 */
#define WS2812_DUTYCYCLE_0      (WS2812_PWM_FREQUENCY/(1000000000/350))

/**
 * @brief   High period for a one, in ticks
 *
 * Per the datasheet:
 * WS2812:
 * - T1H: 550 nS to 850 nS, inclusive
 * - T1L: 450 nS to 750 nS, inclusive
 * WS2812B:
 * - T1H: 750 nS to 1050 nS, inclusive
 * - T1L: 200 nS to 500 nS, inclusive
 * SK6812:
 * - T1H: 450 nS to 750 nS, inclusive
 * - T1L: 450 nS to 750 nS, inclusive
 *
 * The duty cycle is calculated for a high period of 800 nS.
 * This is in the middle of the specifications of the WS2812 and WS2812B.
 */
#define WS2812_DUTYCYCLE_1      (WS2812_PWM_FREQUENCY/(1000000000/800))

/* --- PRIVATE MACROS ------------------------------------------------------- */

/**
 * @brief   Generates a reference to a numbered PWM driver
 *
 * @param[in] n:            The driver (timer) number
 *
 * @return                  A reference to the driver
 */
#define PWMD(n)                             CONCAT_EXPANDED_SYMBOLS(PWMD, n)

#define WS2812_PWMD                         PWMD(WS2812_TIM_N)      /**< The PWM driver to use for the LED chain */

/**
 * @brief   Determine the index in @ref ws2812_frame_buffer "the frame buffer" of a given bit
 *
 * @param[in] led:                  The led index [0, @ref WS2812_LED_N)
 * @param[in] byte:                 The byte number [0, 2]
 * @param[in] bit:                  The bit number [0, 7]
 *
 * @return                          The bit index
 */
#define WS2812_BIT(led, byte, bit)          (WS2812_COLOR_BITS*(led) + 8*(byte) + (7 - (bit)))

/**
 * @brief   Determine the index in @ref ws2812_frame_buffer "the frame buffer" of a given red bit
 *
 * @note    The red byte is the middle byte in the color packet
 *
 * @param[in] led:                  The led index [0, @ref WS2812_LED_N)
 * @param[in] bit:                  The bit number [0, 7]
 *
 * @return                          The bit index
 */
#define WS2812_RED_BIT(led, bit)            WS2812_BIT((led), 1, (bit))

/**
 * @brief   Determine the index in @ref ws2812_frame_buffer "the frame buffer" of a given green bit
 *
 * @note    The green byte is the first byte in the color packet
 *
 * @param[in] led:                  The led index [0, @ref WS2812_LED_N)
 * @param[in] bit:                  The bit number [0, 7]
 *
 * @return                          The bit index
 */
#define WS2812_GREEN_BIT(led, bit)          WS2812_BIT((led), 0, (bit))

/**
 * @brief   Determine the index in @ref ws2812_frame_buffer "the frame buffer" of a given blue bit
 *
 * @note    The blue byte is the last byte in the color packet
 *
 * @param[in] led:                  The led index [0, @ref WS2812_LED_N)
 * @param[in] bit:                  The bit index [0, 7]
 *
 * @return                          The bit index
 */
#define WS2812_BLUE_BIT(led, bit)           WS2812_BIT((led), 2, (bit))

/**
 * @brief   Determine the index in @ref ws2812_frame_buffer "the frame buffer" of a given white bit
 *
 * @note    The white byte is the last byte in the color packet
 *
 * @param[in] led:                  The led index [0, @ref WS2812_LED_N)
 * @param[in] bit:                  The bit index [0, 7]
 *
 * @return                          The bit index
 */
#define WS2812_WHITE_BIT(led, bit)           WS2812_BIT((led), 3, (bit))

/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static uint32_t ws2812_frame_buffer[WS2812_BIT_N + 1];                             /**< Buffer for a frame */

/* --- PUBLIC FUNCTIONS ----------------------------------------------------- */
/*
 * Gedanke: Double-buffer type transactions: double buffer transfers using two memory pointers for
the memory (while the DMA is reading/writing from/to a buffer, the application can
write/read to/from the other buffer).
 */

void ws2812_init(void)
{
    // Initialize led frame buffer
    uint32_t i;
    for (i = 0; i < WS2812_COLOR_BIT_N; i++) ws2812_frame_buffer[i]                       = WS2812_DUTYCYCLE_0;   // All color bits are zero duty cycle
    for (i = 0; i < WS2812_RESET_BIT_N; i++) ws2812_frame_buffer[i + WS2812_COLOR_BIT_N]  = 0;                    // All reset bits are zero

    // PWM Configuration
    #pragma GCC diagnostic ignored "-Woverride-init"                                        // Turn off override-init warning for this struct. We use the overriding ability to set a "default" channel config
    static const PWMConfig ws2812_pwm_config = {
        .frequency          = WS2812_PWM_FREQUENCY,
        .period             = WS2812_PWM_PERIOD, //Mit dieser Periode wird UDE-Event erzeugt und ein neuer Wert (Länge WS2812_BIT_N) vom DMA ins CCR geschrieben
        .callback           = NULL,
        .channels = {
            [0 ... 3]       = {.mode = PWM_OUTPUT_DISABLED,     .callback = NULL},          // Channels default to disabled
            [WS2812_TIM_CH] = {.mode = PWM_OUTPUT_ACTIVE_HIGH,  .callback = NULL},          // Turn on the channel we care about
        },
        .cr2                = 0,
        .dier               = TIM_DIER_UDE,                                                 // DMA on update event for next period
    };
    #pragma GCC diagnostic pop                                                              // Restore command-line warning options

    // Configure DMA
    //dmaInit(); // Joe added this
    const stm32_dma_stream_t* dma = STM32_DMA2_STREAM1;
    dmaStreamAllocate(dma, 10, NULL, NULL);
    dmaStreamSetPeripheral(dma, &(WS2812_PWMD.tim->CCR[WS2812_TIM_CH]));  // Ziel ist der An-Zeit im Cap-Comp-Register
    dmaStreamSetMemory0(dma, ws2812_frame_buffer);
    dmaStreamSetTransactionSize(dma, WS2812_BIT_N);
    dmaStreamSetMode(dma,
      STM32_DMA_CR_CHSEL(WS2812_DMA_CHANNEL) | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD |
      STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(3));
    // M2P: Memory 2 Periph; PL: Priority Level

    // Configure PWM
    // NOTE: It's required that preload be enabled on the timer channel CCR register. This is currently enabled in the
    // ChibiOS driver code, so we don't have to do anything special to the timer. If we did, we'd have to start the timer,
    // disable counting, enable the channel, and then make whatever configuration changes we need.
    pwmStart(&WS2812_PWMD, &ws2812_pwm_config);
    pwmEnableChannel(&WS2812_PWMD, WS2812_TIM_CH, 0);     // Initial period is 0; output will be low until first duty cycle is DMA'd in

    // Configure pin as AF output. If there's an external pull up resistor the signal level is brought to 5V using open drain mode.
#ifdef WS2812_EXTERNAL_PULLUP
    palSetPadMode(PORT_WS2812, PIN_WS2812, PAL_MODE_ALTERNATE(WS2812_AF) | PAL_STM32_OTYPE_OPENDRAIN);
#else
    palSetPadMode(PORT_WS2812, PIN_WS2812, PAL_MODE_ALTERNATE(WS2812_AF));
#endif

    // Start DMA
    dmaStreamEnable(dma);
}

ws2812_err_t ws2812_write_led(uint32_t led_number, uint8_t r, uint8_t g, uint8_t b)
{
    // Check for valid LED
    if (led_number >= WS2812_LED_N) return WS2812_LED_INVALID;

    // Write color to frame buffer
    uint32_t bit;
    for (bit = 0; bit < 8; bit++) {
        ws2812_frame_buffer[WS2812_RED_BIT(led_number, bit)]      = ((r >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
        ws2812_frame_buffer[WS2812_GREEN_BIT(led_number, bit)]    = ((g >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
        ws2812_frame_buffer[WS2812_BLUE_BIT(led_number, bit)]     = ((b >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
    }

    // Success
    return WS2812_SUCCESS;
}

ws2812_err_t ws2812_write_led_rgbw(uint32_t led_number, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    // Check for valid LED
    if (led_number >= WS2812_LED_N) return WS2812_LED_INVALID;

    // Write color to frame buffer
    uint32_t bit;
    for (bit = 0; bit < 8; bit++) {
        ws2812_frame_buffer[WS2812_RED_BIT(led_number, bit)]      = ((r >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
        ws2812_frame_buffer[WS2812_GREEN_BIT(led_number, bit)]    = ((g >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
        ws2812_frame_buffer[WS2812_BLUE_BIT(led_number, bit)]     = ((b >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
#if WS2812_COLOR_N >= 4
        ws2812_frame_buffer[WS2812_WHITE_BIT(led_number, bit)]    = ((w >> bit) & 0x01) ? WS2812_DUTYCYCLE_1 : WS2812_DUTYCYCLE_0;
#endif
    }

    // Success
    return WS2812_SUCCESS;
}

/** @} addtogroup WS2812 */

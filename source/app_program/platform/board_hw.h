/** @file board_hw.h
 *
 * @brief Robo Pico board services built on the Pico SDK: peripheral reset
 *        release, the microsecond time base, the START button and the
 *        GPIO bank-0 interrupt registers.
 *
 * @par
 * Owner: platform / integration.  This is a hardware module (board_hw.c
 * includes Pico SDK headers only); this header is plain C so kernel-side
 * modules can call it.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BOARD_HW_H
#define BOARD_HW_H

#include <stdint.h>
#include <stdbool.h>

// RP2040 NVIC number of IO_IRQ_BANK0 (checked against the SDK in board_hw.c).
#define BOARD_HW_GPIO_IRQ_NUM       13u

// Number of user GPIOs in bank 0 (GP0..GP29).
#define BOARD_HW_GPIO_COUNT         30u

/*!
 * @brief Pad pull resistor selection for a digital input.
 */
typedef enum
{
    BOARD_HW_PULL_NONE = 0,
    BOARD_HW_PULL_UP,
    BOARD_HW_PULL_DOWN
} board_hw_pull_t;

void     board_hw_init(void);
uint32_t board_hw_time_us(void);
void     board_hw_delay_us(uint32_t delay_us);
bool     board_hw_is_start_pressed(void);
void     board_hw_gpio_input(uint32_t pin, board_hw_pull_t pull);
void     board_hw_gpio_output(uint32_t pin, bool b_level);
void     board_hw_gpio_write(uint32_t pin, bool b_level);
bool     board_hw_gpio_read(uint32_t pin);
uint32_t board_hw_gpio_levels(void);
void     board_hw_gpio_irq_enable(uint32_t pin, bool b_rise, bool b_fall);
void     board_hw_gpio_irq_take(uint32_t * p_rise_mask,
                                uint32_t * p_fall_mask);

#endif /* BOARD_HW_H */

/*** end of file ***/


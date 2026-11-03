/** @file board_hw.c
 *
 * @brief Robo Pico board services built on the Pico SDK.
 *
 * @par
 * micro T-Kernel releases only the peripherals it needs itself (IO bank,
 * pads, UART0 and, on radio builds, PIO and DMA).  This module releases the
 * rest of what the robot uses, starts the 1 MHz timer tick, configures the
 * START button and gives the kernel-side GPIO dispatcher access to the bank-0
 * interrupt registers without including any SDK header itself.
 *
 * @par
 * Reference: RP2040 Datasheet, sections 2.14 (Subsystem Resets), 2.19 (GPIO)
 * and 4.6 (Timer).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/timer.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/padsbank0.h"
#include "hardware/structs/watchdog.h"

#include "robot_cfg.h"
#include "board_hw.h"

// clk_ref runs from the 12 MHz crystal; dividing by 12 gives the 1 MHz tick.
#define TICK_CYCLES_PER_US      12u

// Each IO bank-0 interrupt register covers 8 pins with 4 event bits each.
#define PINS_PER_IRQ_REG        8u
#define BITS_PER_PIN            4u
#define IRQ_REG_COUNT           4u
#define EVENT_EDGE_LOW_BIT      2u
#define EVENT_EDGE_HIGH_BIT     3u

_Static_assert(BOARD_HW_GPIO_IRQ_NUM == IO_IRQ_BANK0,
               "BOARD_HW_GPIO_IRQ_NUM must match the SDK IO_IRQ_BANK0");
_Static_assert((PINS_PER_IRQ_REG * IRQ_REG_COUNT) >= BOARD_HW_GPIO_COUNT,
               "interrupt register geometry must cover every GPIO");

static void disable_pad_input(uint32_t pin);


/*!
 * @brief Release the robot's peripherals from reset and configure the
 *        board-level pins.
 *
 * @par
 * Must run once, from usermain(), before any driver init function.
 */
void
board_hw_init (void)
{
    // Step 1: start the 1 us tick that drives the TIMER block.
    watchdog_hw->tick = WATCHDOG_TICK_ENABLE_BITS | TICK_CYCLES_PER_US;

    // Step 2: release the blocks the kernel leaves in reset.
    unreset_block_wait(RESETS_RESET_TIMER_BITS | RESETS_RESET_PWM_BITS |
                       RESETS_RESET_ADC_BITS | RESETS_RESET_I2C0_BITS);

    // Step 3: START button, active low with the internal pull-up.
    board_hw_gpio_input(PIN_START_BUTTON, BOARD_HW_PULL_UP);

    // Step 4: the IR modules' AO wires land on non-ADC pins; keep those
    // pins from drawing current with an analogue voltage on the input.
    disable_pad_input(PIN_IR1_AO_UNUSED);
    disable_pad_input(PIN_IR3_AO_UNUSED);
}


/*!
 * @brief Read the free-running microsecond counter.
 *
 * @return Microseconds since boot, modulo 2^32 (wraps every 71.6 minutes).
 *         Callers compare times by unsigned subtraction, which is wrap-safe.
 */
uint32_t
board_hw_time_us (void)
{
    return (time_us_32());
}


/*!
 * @brief Busy-wait for a short interval.
 *
 * @param[in] delay_us  Microseconds to wait.  Intended for pulses of tens of
 *                      microseconds only; longer waits must use the kernel.
 */
void
board_hw_delay_us (uint32_t delay_us)
{
    uint32_t const start_us = time_us_32();

    while ((time_us_32() - start_us) < delay_us)
    {
        // Spin: the interval is far shorter than one kernel tick.
    }
}


/*!
 * @brief Report whether the Robo Pico START button (GP20) is held down.
 *
 * @return true while the button is pressed.
 */
bool
board_hw_is_start_pressed (void)
{
    return (!gpio_get(PIN_START_BUTTON));
}


/*!
 * @brief Configure a pin as a digital input.
 *
 * @param[in] pin   GPIO number.
 * @param[in] pull  Pad pull resistor.
 */
void
board_hw_gpio_input (uint32_t pin, board_hw_pull_t pull)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_pulls(pin, (BOARD_HW_PULL_UP == pull),
                   (BOARD_HW_PULL_DOWN == pull));
}


/*!
 * @brief Configure a pin as a digital output.
 *
 * @param[in] pin      GPIO number.
 * @param[in] b_level  Initial output level.
 */
void
board_hw_gpio_output (uint32_t pin, bool b_level)
{
    gpio_init(pin);
    gpio_put(pin, b_level);
    gpio_set_dir(pin, GPIO_OUT);
}


/*!
 * @brief Drive an output pin.
 *
 * @param[in] pin      GPIO number configured as an output.
 * @param[in] b_level  Level to drive.
 */
void
board_hw_gpio_write (uint32_t pin, bool b_level)
{
    gpio_put(pin, b_level);
}


/*!
 * @brief Read a pin's input level.
 *
 * @param[in] pin  GPIO number.
 *
 * @return true if the pin is high.
 */
bool
board_hw_gpio_read (uint32_t pin)
{
    return (gpio_get(pin));
}


/*!
 * @brief Snapshot the input level of every bank-0 GPIO.
 *
 * @return Bit n is the current level of GPn.
 */
uint32_t
board_hw_gpio_levels (void)
{
    return (gpio_get_all());
}


/*!
 * @brief Enable edge interrupts for one pin on this core.
 *
 * @param[in] pin     GPIO number.
 * @param[in] b_rise  true to interrupt on a rising edge.
 * @param[in] b_fall  true to interrupt on a falling edge.
 */
void
board_hw_gpio_irq_enable (uint32_t pin, bool b_rise, bool b_fall)
{
    uint32_t events = 0u;

    if (b_rise)
    {
        events |= GPIO_IRQ_EDGE_RISE;
    }

    if (b_fall)
    {
        events |= GPIO_IRQ_EDGE_FALL;
    }

    gpio_acknowledge_irq(pin, events);
    gpio_set_irq_enabled(pin, events, true);
}


/*!
 * @brief Collect and acknowledge every pending bank-0 edge event.
 *
 * @param[out] p_rise_mask  Bit n set if GPn saw a rising edge.
 * @param[out] p_fall_mask  Bit n set if GPn saw a falling edge.
 *
 * @par
 * Called only from the GPIO interrupt handler.  Edges are acknowledged
 * before they are reported, so an edge arriving during dispatch raises the
 * interrupt again instead of being lost.
 */
void
board_hw_gpio_irq_take (uint32_t * p_rise_mask, uint32_t * p_fall_mask)
{
    uint32_t rise_mask = 0u;
    uint32_t fall_mask = 0u;

    for (uint32_t reg = 0u; reg < IRQ_REG_COUNT; reg++)
    {
        uint32_t const status = iobank0_hw->proc0_irq_ctrl.ints[reg];

        if (0u != status)
        {
            iobank0_hw->intr[reg] = status;

            for (uint32_t slot = 0u; slot < PINS_PER_IRQ_REG; slot++)
            {
                uint32_t const pin    = (reg * PINS_PER_IRQ_REG) + slot;
                uint32_t const events = status >> (slot * BITS_PER_PIN);

                if (0u != (events & (1u << EVENT_EDGE_HIGH_BIT)))
                {
                    rise_mask |= (1u << pin);
                }

                if (0u != (events & (1u << EVENT_EDGE_LOW_BIT)))
                {
                    fall_mask |= (1u << pin);
                }
            }
        }
    }

    *p_rise_mask = rise_mask;
    *p_fall_mask = fall_mask;
}


/*!
 * @brief Disconnect a pad's digital input buffer.
 *
 * @param[in] pin  GPIO number whose pad input is disabled.
 */
static void
disable_pad_input (uint32_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_NULL);
    hw_clear_bits(&padsbank0_hw->io[pin], PADS_BANK0_GPIO0_IE_BITS);
}

/*** end of file ***/


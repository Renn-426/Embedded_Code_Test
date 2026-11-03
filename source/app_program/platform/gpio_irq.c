/** @file gpio_irq.c
 *
 * @brief Single GPIO bank-0 interrupt handler and per-pin dispatch.
 *
 * @par
 * NOTE (Barr-C 6.5.a): micro T-Kernel enters every interrupt through its own
 * assembly entry point and calls a TA_HLNG handler as an ordinary AAPCS
 * function, so the Cortex-M0+ needs no compiler interrupt keyword.  The
 * handler is static and placed at the end of the module (rule 6.5.c).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "board_hw.h"
#include "gpio_irq.h"

// NVIC priority level for the GPIO interrupt.  Level 1 is the highest the
// port allows for external interrupts; edges must be timestamped promptly.
#define GPIO_IRQ_LEVEL          1

// Handler attached to each pin; NULL when the pin is not used.
static gpio_irq_handler_t gp_handlers[BOARD_HW_GPIO_COUNT];

// Interrupts serviced since boot (diagnostic).
static uint32_t volatile  g_irq_count = 0u;

static void gpio_irq_bank0_isr(UINT int_number);


/*!
 * @brief Attach a handler to a pin and enable its edge interrupt.
 *
 * @param[in] pin        GPIO number.
 * @param[in] p_handler  Handler called in interrupt context.
 * @param[in] b_rise     Interrupt on rising edges.
 * @param[in] b_fall     Interrupt on falling edges.
 *
 * @return true if attached; false for an invalid pin or a pin that already
 *         has a handler.
 *
 * @par
 * Call during initialisation, before gpio_irq_start().
 */
bool
gpio_irq_attach (uint32_t pin, gpio_irq_handler_t p_handler, bool b_rise,
                 bool b_fall)
{
    bool b_attached = false;

    if ((pin < BOARD_HW_GPIO_COUNT) && (NULL != p_handler) &&
        (NULL == gp_handlers[pin]))
    {
        gp_handlers[pin] = p_handler;
        board_hw_gpio_irq_enable(pin, b_rise, b_fall);
        b_attached = true;
    }

    return (b_attached);
}


/*!
 * @brief Register the bank-0 handler with the kernel and enable it.
 *
 * @return E_OK (0) on success, or a negative kernel error code.
 */
int32_t
gpio_irq_start (void)
{
    T_DINT const dint =
    {
        .intatr = TA_HLNG,
        .inthdr = gpio_irq_bank0_isr
    };
    ER const     ercd = tk_def_int(BOARD_HW_GPIO_IRQ_NUM, &dint);

    if (E_OK == ercd)
    {
        EnableInt(BOARD_HW_GPIO_IRQ_NUM, GPIO_IRQ_LEVEL);
    }

    return (ercd);
}


/*!
 * @brief Number of GPIO interrupts serviced since boot.
 *
 * @return Interrupt count.
 */
uint32_t
gpio_irq_count (void)
{
    return (g_irq_count);
}


/*!
 * @brief IO bank-0 interrupt handler.
 *
 * @param[in] int_number  Interrupt number (always IO_IRQ_BANK0; unused).
 *
 * @par
 * Takes one timestamp and one level snapshot, acknowledges every pending
 * edge, then calls each affected pin's handler.  Pins without a handler are
 * acknowledged and ignored.
 */
static void
gpio_irq_bank0_isr (UINT int_number)
{
    uint32_t const time_us   = board_hw_time_us();
    uint32_t const levels    = board_hw_gpio_levels();
    uint32_t       rise_mask = 0u;
    uint32_t       fall_mask = 0u;

    (void) int_number;  // Unused: this handler serves one interrupt only.

    board_hw_gpio_irq_take(&rise_mask, &fall_mask);
    g_irq_count++;

    for (uint32_t pin = 0u; pin < BOARD_HW_GPIO_COUNT; pin++)
    {
        uint32_t const bit   = 1u << pin;
        uint32_t       edges = 0u;

        if (0u != (rise_mask & bit))
        {
            edges |= GPIO_IRQ_EDGE_RISING;
        }

        if (0u != (fall_mask & bit))
        {
            edges |= GPIO_IRQ_EDGE_FALLING;
        }

        if ((0u != edges) && (NULL != gp_handlers[pin]))
        {
            gp_handlers[pin](pin, edges, time_us, levels);
        }
    }
}

/*** end of file ***/


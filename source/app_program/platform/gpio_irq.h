/** @file gpio_irq.h
 *
 * @brief Single GPIO bank-0 interrupt handler shared by the encoders, the
 *        barcode sensor and the ultrasonic ECHO input (Week 6 Section 4).
 *
 * @par
 * The RP2040 has one interrupt for every bank-0 pin.  This module owns it,
 * timestamps each interrupt once, and dispatches every pending edge to the
 * handler attached to that pin.  Handlers run in interrupt context: they
 * must only record data and signal a task (tk_wup_tsk, tk_sig_sem).
 *
 * @par
 * Owner: platform / integration.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <stdint.h>
#include <stdbool.h>

// Edge bits passed to a handler.
#define GPIO_IRQ_EDGE_RISING        0x01u
#define GPIO_IRQ_EDGE_FALLING       0x02u

/*!
 * @brief Pin handler, called in interrupt context.
 *
 * @param pin      GPIO number that saw the edge(s).
 * @param edges    GPIO_IRQ_EDGE_* bits; both set means two edges were
 *                 coalesced since the last interrupt.
 * @param time_us  Interrupt entry time from board_hw_time_us().
 * @param levels   Snapshot of every pin level at interrupt entry.
 */
typedef void (* gpio_irq_handler_t)(uint32_t pin, uint32_t edges,
                                    uint32_t time_us, uint32_t levels);

bool     gpio_irq_attach(uint32_t pin, gpio_irq_handler_t p_handler,
                         bool b_rise, bool b_fall);
int32_t  gpio_irq_start(void);
uint32_t gpio_irq_count(void);

#endif /* GPIO_IRQ_H */

/*** end of file ***/


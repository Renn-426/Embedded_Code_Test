/** @file ultrasonic.c
 *
 * @brief HC-SR04 ranging with interrupt-timed ECHO pulses.
 *
 * @par
 * The ECHO interrupt records the rising-edge time and, on the falling edge,
 * the pulse width, then signals sem_echo.  Distance is width x 343 m/s / 2.
 * Reference: HC-SR04 datasheet (10 us trigger, echo width proportional to
 * range, about 38 ms when nothing returns).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "board_hw.h"
#include "gpio_irq.h"
#include "ultrasonic.h"

#define TRIGGER_US              10u
#define SOUND_MM_PER_MS         343u
#define ROUND_TRIP_DIVISOR      2000u

// ECHO widths beyond this are "nothing in range" (SONAR_MAX_MM round trip).
#define MAX_ECHO_US             ((SONAR_MAX_MM * ROUND_TRIP_DIVISOR) / \
                                 SOUND_MM_PER_MS)

// Written only by the ECHO interrupt handler.
static uint32_t volatile g_rise_us    = 0u;
static uint32_t volatile g_width_us   = 0u;
static uint32_t volatile g_rise_count = 0u;
static uint32_t volatile g_fall_count = 0u;

static ID                gh_sem_echo  = 0;

static void ultrasonic_echo_isr(uint32_t pin, uint32_t edges,
                                uint32_t time_us, uint32_t levels);


/*!
 * @brief Configure TRIG and ECHO and create sem_echo.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
ultrasonic_init (void)
{
    T_CSEM const csem   =
    {
        .exinf = NULL, .sematr = TA_TFIFO, .isemcnt = 0, .maxsem = 1
    };
    int32_t      result = 0;

    board_hw_gpio_output(PIN_SONAR_TRIG, false);
    board_hw_gpio_input(PIN_SONAR_ECHO, BOARD_HW_PULL_DOWN);
    gh_sem_echo = tk_cre_sem(&csem);

    if ((gh_sem_echo <= 0) ||
        (!gpio_irq_attach(PIN_SONAR_ECHO, ultrasonic_echo_isr, true, true)))
    {
        result = -1;
    }

    return (result);
}


/*!
 * @brief Take one range measurement (blocks for up to about 50 ms).
 *
 * @param[out] p_distance_mm  Distance on ULTRASONIC_OK.
 *
 * @return ULTRASONIC_OK, ULTRASONIC_NO_ECHO or ULTRASONIC_FAULT.
 *
 * @par
 * Called only by the Obstacle Scanning task.
 */
ultrasonic_result_t
ultrasonic_measure (uint32_t * p_distance_mm)
{
    uint32_t const      rises_before = g_rise_count;
    ultrasonic_result_t result       = ULTRASONIC_FAULT;
    ER                  ercd         = E_OK;

    // Discard a stale signal left over from an earlier late echo.
    while (E_OK == tk_wai_sem(gh_sem_echo, 1, TMO_POL))
    {
        // Nothing to do: each successful poll consumes one stale signal.
    }

    board_hw_gpio_write(PIN_SONAR_TRIG, true);
    board_hw_delay_us(TRIGGER_US);
    board_hw_gpio_write(PIN_SONAR_TRIG, false);

    ercd = tk_wai_sem(gh_sem_echo, 1, SONAR_TIMEOUT_MS);

    if (E_OK == ercd)
    {
        uint32_t const width_us = g_width_us;

        if (width_us > MAX_ECHO_US)
        {
            result = ULTRASONIC_NO_ECHO;
        }
        else
        {
            *p_distance_mm = (width_us * SOUND_MM_PER_MS) /
                             ROUND_TRIP_DIVISOR;
            result         = ULTRASONIC_OK;
        }
    }
    else if (rises_before != g_rise_count)
    {
        // ECHO went high but never came back down within the timeout.
        result = ULTRASONIC_NO_ECHO;
    }
    else
    {
        // ECHO never went high: no sensor response at all.
        result = ULTRASONIC_FAULT;
    }

    return (result);
}


/*!
 * @brief GPIO handler for ECHO (interrupt context).
 *
 * @param[in] pin      PIN_SONAR_ECHO.
 * @param[in] edges    GPIO_IRQ_EDGE_* bits.
 * @param[in] time_us  Interrupt entry time.
 * @param[in] levels   Pin levels at interrupt entry.
 */
static void
ultrasonic_echo_isr (uint32_t pin, uint32_t edges, uint32_t time_us,
                     uint32_t levels)
{
    (void) pin;     // Unused: this handler serves one pin.
    (void) levels;  // Unused: the edge bits give the direction.

    if ((GPIO_IRQ_EDGE_RISING | GPIO_IRQ_EDGE_FALLING) == edges)
    {
        // Both edges inside one interrupt: the width is unknown, so no
        // distance is reported (the measurement becomes NO_ECHO).
        g_rise_count++;
    }
    else if (0u != (edges & GPIO_IRQ_EDGE_RISING))
    {
        g_rise_us = time_us;
        g_rise_count++;
    }
    else
    {
        g_width_us = time_us - g_rise_us;
        g_fall_count++;
        (void) tk_sig_sem(gh_sem_echo, 1);  // E_QOVR: already signalled.
    }
}

/*** end of file ***/


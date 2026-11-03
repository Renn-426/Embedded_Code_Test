/** @file sdk_stub_hw.c
 *
 * @brief Pico SDK runtime hooks for the radio-less bench build.
 *
 * @par
 * The robot links only a few Pico SDK driver sources, not the SDK runtime.
 * In the radio build the port's cyw43_utk_compat.c supplies the runtime
 * hooks those drivers call (clock_get_hz, time_us_64).  This module supplies
 * the same two hooks when the firmware is built without the radio
 * (WIFI=none), so the I2C and timeout code links in both profiles.
 *
 * @par
 * NOTE (Barr-C 6.1.i): clock_get_hz() and time_us_64() keep the names the
 * Pico SDK calls them by; they are the only public functions in the
 * application without a module prefix.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "sdk_stub_hw.h"

#ifndef TM_WIFI_CYW43
#define TM_WIFI_CYW43           0
#endif

#if !TM_WIFI_CYW43

#include "hardware/clocks.h"
#include "hardware/timer.h"

// clk_sys and clk_peri both run at 125 MHz (port cpu_clock.c).
#define CLK_SYS_HZ              125000000u
#define HIGH_WORD_SHIFT         32u


/*!
 * @brief Frequency of any clock the SDK drivers ask for.
 *
 * @param[in] clock  Clock identifier (all used clocks run at 125 MHz).
 *
 * @return 125000000.
 */
uint32_t
clock_get_hz (clock_handle_t clock)
{
    (void) clock;  // Unused: every clock used by the drivers is 125 MHz.

    return (CLK_SYS_HZ);
}


/*!
 * @brief 64-bit microsecond time, read without a latch race.
 *
 * @return Microseconds since boot.
 *
 * @par
 * The high word is read before and after the low word and the read is
 * repeated if the high word changed in between.
 */
uint64_t
time_us_64 (void)
{
    uint32_t high = timer_hw->timerawh;
    uint32_t low  = timer_hw->timerawl;

    while (high != timer_hw->timerawh)
    {
        high = timer_hw->timerawh;
        low  = timer_hw->timerawl;
    }

    return ((((uint64_t) high) << HIGH_WORD_SHIFT) | low);  // Widening.
}

#endif  /* !TM_WIFI_CYW43 */

/*!
 * @brief Report whether this build provides its own SDK runtime hooks.
 *
 * @return true in the radio-less build.
 */
bool
sdk_stub_hw_is_active (void)
{
    return (0 == TM_WIFI_CYW43);
}

/*** end of file ***/


/** @file sys_status.c
 *
 * @brief Latest-value status board stored as packed 32-bit words.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>

#include "sys_status.h"

// Used bits of each word, in sys_status_word_t order.  Must match the field
// layouts in the owners' headers and tools/tlm_decode.py.
static uint32_t const g_word_width[SYS_STATUS_WORD_COUNT] =
{
    23u,    // VEHICLE
    26u,    // MOTION
    32u,    // ENC_COUNTS
    16u,    // ODOMETRY
    32u,    // LINE
    27u,    // BARCODE
    27u,    // IMU
    22u,    // HUMP
    24u,    // OBSTACLE
    19u,    // COMMS
    24u     // HEALTH
};

// Single-writer words.  volatile: written and read by different tasks.
static uint32_t volatile g_words[SYS_STATUS_WORD_COUNT];


/*!
 * @brief Publish a complete status word.
 *
 * @param[in] word   Which word; the caller must be its owner.
 * @param[in] value  Fully built packed value.
 */
void
sys_status_write (sys_status_word_t word, uint32_t value)
{
    if (word < SYS_STATUS_WORD_COUNT)
    {
        // One aligned 32-bit store: atomic on the Cortex-M0+.
        g_words[word] = value;
    }
}


/*!
 * @brief Read the latest value of a status word.
 *
 * @param[in] word  Which word.
 *
 * @return The packed value, or 0 for an invalid word.
 */
uint32_t
sys_status_read (sys_status_word_t word)
{
    uint32_t value = 0u;

    if (word < SYS_STATUS_WORD_COUNT)
    {
        value = g_words[word];
    }

    return (value);
}


/*!
 * @brief Number of meaningful bits in a status word.
 *
 * @param[in] word  Which word.
 *
 * @return Width in bits, or 0 for an invalid word.
 */
uint32_t
sys_status_width (sys_status_word_t word)
{
    uint32_t width = 0u;

    if (word < SYS_STATUS_WORD_COUNT)
    {
        width = g_word_width[word];
    }

    return (width);
}


/*!
 * @brief Total meaningful bits across every status word.
 *
 * @return Sum of the word widths (the telemetry frame body size).
 */
uint32_t
sys_status_total_bits (void)
{
    uint32_t total = 0u;

    // The cast is exact: the enumeration count is a small positive constant.
    for (uint32_t idx = 0u; idx < (uint32_t) SYS_STATUS_WORD_COUNT; idx++)
    {
        total += g_word_width[idx];
    }

    return (total);
}

/*** end of file ***/


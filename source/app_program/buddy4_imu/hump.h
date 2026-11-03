/** @file hump.h
 *
 * @brief Hump detector and peak-height estimator (Buddy 4, Week 6
 *        Section 6.5).
 *
 * @par
 * Pure module (no kernel or hardware dependency), unit-tested on the host.
 * A hump is the sequence LEVEL, CLIMBING, DESCENDING, LEVEL.  While on the
 * hump, height is integrated as encoder distance x sin(pitch); the peak of
 * that integral is the hump height.  Levelling out during CLIMBING is a
 * false start and is discarded, and an invalid sample abandons the hump, so
 * invalid data can never update the running maximum (assumption A4).
 *
 * @par
 * NOTE: distance x sin(pitch) assumes both wheels climb together.  A hump
 * crossed by one wheel only is an open hypothesis (Week 6 A4) to be checked
 * against reference measurements.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef HUMP_H
#define HUMP_H

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief Hump states (2 bits).
 */
typedef enum
{
    HUMP_STATE_LEVEL = 0,
    HUMP_STATE_CLIMBING,
    HUMP_STATE_DESCENDING
} hump_state_t;

/*!
 * @brief Detector configuration and state.
 */
typedef struct
{
    int32_t         enter_mdeg;     // Pitch magnitude that starts a phase.
    int32_t         exit_mdeg;      // Pitch magnitude that counts as level.
    uint32_t        hold_ms;        // Time a condition must persist.
    hump_state_t    state;
    uint32_t        timer_ms;       // Time the transition condition held.
    uint32_t        level_ms;       // Time spent level while CLIMBING.
    int32_t         last_distance_mm;
    int32_t         height_um;
    int32_t         peak_um;
    uint32_t        max_mm;
    uint32_t        count;
} hump_t;

void         hump_init(hump_t * p_hump, int32_t enter_mdeg, int32_t exit_mdeg,
                       uint32_t hold_ms);
bool         hump_update(hump_t * p_hump, int32_t pitch_mdeg,
                         int32_t distance_mm, bool b_valid, uint32_t dt_ms,
                         uint32_t * p_height_mm);
hump_state_t hump_state(hump_t const * p_hump);
uint32_t     hump_max_mm(hump_t const * p_hump);
uint32_t     hump_count(hump_t const * p_hump);

#endif /* HUMP_H */

/*** end of file ***/


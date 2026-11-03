/** @file hump.c
 *
 * @brief Hump detector and peak-height estimator.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "fix_math.h"
#include "hump.h"

#define UM_PER_MM               1000
#define MAX_STEP_MM             100

static void    restart(hump_t * p_hump);
static int32_t rise_um(int32_t step_mm, int32_t pitch_mdeg);
static void    step_level(hump_t * p_hump, int32_t pitch_mdeg,
                          int32_t rise, uint32_t dt_ms);
static void    step_climbing(hump_t * p_hump, int32_t pitch_mdeg,
                             uint32_t dt_ms);
static bool    step_descending(hump_t * p_hump, int32_t pitch_mdeg,
                               uint32_t dt_ms, uint32_t * p_height_mm);


/*!
 * @brief Configure a detector and clear its history.
 *
 * @param[out] p_hump      Detector.
 * @param[in]  enter_mdeg  Pitch that starts a climb or descent.
 * @param[in]  exit_mdeg   Pitch below which the robot counts as level.
 * @param[in]  hold_ms     How long a condition must persist.
 */
void
hump_init (hump_t * p_hump, int32_t enter_mdeg, int32_t exit_mdeg,
           uint32_t hold_ms)
{
    p_hump->enter_mdeg       = enter_mdeg;
    p_hump->exit_mdeg        = exit_mdeg;
    p_hump->hold_ms          = hold_ms;
    p_hump->last_distance_mm = 0;
    p_hump->max_mm           = 0u;
    p_hump->count            = 0u;
    restart(p_hump);
}


/*!
 * @brief Process one sample.
 *
 * @param[in,out] p_hump       Detector.
 * @param[in]     pitch_mdeg   Pitch, positive nose-up, bias removed.
 * @param[in]     distance_mm  Odometry distance (any origin).
 * @param[in]     b_valid      false if the sensor data is not trustworthy.
 * @param[in]     dt_ms        Time since the previous sample.
 * @param[out]    p_height_mm  Height of a completed hump.
 *
 * @return true when a hump has just been completed and *p_height_mm set.
 */
bool
hump_update (hump_t * p_hump, int32_t pitch_mdeg, int32_t distance_mm,
             bool b_valid, uint32_t dt_ms, uint32_t * p_height_mm)
{
    int32_t const step     = fix_math_clamp(distance_mm -
                                            p_hump->last_distance_mm,
                                            -MAX_STEP_MM, MAX_STEP_MM);
    int32_t const rise     = rise_um(step, pitch_mdeg);
    bool          b_report = false;

    p_hump->last_distance_mm = distance_mm;

    if (!b_valid)
    {
        // Invalid data abandons the hump and never touches the maximum.
        restart(p_hump);
    }
    else
    {
        switch (p_hump->state)
        {
            case HUMP_STATE_CLIMBING:
                p_hump->height_um += rise;
                step_climbing(p_hump, pitch_mdeg, dt_ms);
            break;

            case HUMP_STATE_DESCENDING:
                p_hump->height_um += rise;
                b_report = step_descending(p_hump, pitch_mdeg, dt_ms,
                                           p_height_mm);
            break;

            default:
                step_level(p_hump, pitch_mdeg, rise, dt_ms);
            break;
        }

        if (p_hump->height_um > p_hump->peak_um)
        {
            p_hump->peak_um = p_hump->height_um;
        }
    }

    return (b_report);
}


/*!
 * @brief Current detector state.
 *
 * @param[in] p_hump  Detector.
 *
 * @return LEVEL, CLIMBING or DESCENDING.
 */
hump_state_t
hump_state (hump_t const * p_hump)
{
    return (p_hump->state);
}


/*!
 * @brief Highest completed hump since start-up.
 *
 * @param[in] p_hump  Detector.
 *
 * @return Height in millimetres.
 */
uint32_t
hump_max_mm (hump_t const * p_hump)
{
    return (p_hump->max_mm);
}


/*!
 * @brief Number of completed humps since start-up.
 *
 * @param[in] p_hump  Detector.
 *
 * @return Hump count.
 */
uint32_t
hump_count (hump_t const * p_hump)
{
    return (p_hump->count);
}


/*!
 * @brief Return to LEVEL with no hump in progress.
 *
 * @param[in,out] p_hump  Detector.
 */
static void
restart (hump_t * p_hump)
{
    p_hump->state     = HUMP_STATE_LEVEL;
    p_hump->timer_ms  = 0u;
    p_hump->level_ms  = 0u;
    p_hump->height_um = 0;
    p_hump->peak_um   = 0;
}


/*!
 * @brief Height gained over one step: distance x sin(pitch).
 *
 * @param[in] step_mm     Distance travelled, at most +/-100 mm.
 * @param[in] pitch_mdeg  Pitch.
 *
 * @return Height change in micrometres.
 */
static int32_t
rise_um (int32_t step_mm, int32_t pitch_mdeg)
{
    // |step| <= 100 and |sin| <= 32768: the product fits easily.
    return ((step_mm * UM_PER_MM * fix_math_sin_q15(pitch_mdeg)) /
            FIX_MATH_Q15_ONE);
}


/*!
 * @brief LEVEL: start CLIMBING once nose-up pitch persists.
 *
 * @param[in,out] p_hump      Detector.
 * @param[in]     pitch_mdeg  Pitch.
 * @param[in]     rise        Height change of this step.
 * @param[in]     dt_ms       Sample interval.
 *
 * @par
 * Height is accumulated while the condition is pending, so the climb made
 * during the hold time is not lost.
 */
static void
step_level (hump_t * p_hump, int32_t pitch_mdeg, int32_t rise,
            uint32_t dt_ms)
{
    if (pitch_mdeg > p_hump->enter_mdeg)
    {
        p_hump->timer_ms  += dt_ms;
        p_hump->height_um += rise;

        if (p_hump->timer_ms >= p_hump->hold_ms)
        {
            p_hump->state    = HUMP_STATE_CLIMBING;
            p_hump->timer_ms = 0u;
            p_hump->level_ms = 0u;
        }
    }
    else
    {
        restart(p_hump);
    }
}


/*!
 * @brief CLIMBING: go to DESCENDING on nose-down pitch, or discard the
 *        hump as a false start if the robot levels out first.
 *
 * @param[in,out] p_hump      Detector.
 * @param[in]     pitch_mdeg  Pitch.
 * @param[in]     dt_ms       Sample interval.
 */
static void
step_climbing (hump_t * p_hump, int32_t pitch_mdeg, uint32_t dt_ms)
{
    p_hump->timer_ms = (pitch_mdeg < -p_hump->enter_mdeg) ?
                       (p_hump->timer_ms + dt_ms) : 0u;
    p_hump->level_ms = (fix_math_abs(pitch_mdeg) < p_hump->exit_mdeg) ?
                       (p_hump->level_ms + dt_ms) : 0u;

    if (p_hump->timer_ms >= p_hump->hold_ms)
    {
        p_hump->state    = HUMP_STATE_DESCENDING;
        p_hump->timer_ms = 0u;
    }
    else if (p_hump->level_ms >= p_hump->hold_ms)
    {
        restart(p_hump);
    }
    else
    {
        // Still climbing.
    }
}


/*!
 * @brief DESCENDING: once level again, report the hump.
 *
 * @param[in,out] p_hump       Detector.
 * @param[in]     pitch_mdeg   Pitch.
 * @param[in]     dt_ms        Sample interval.
 * @param[out]    p_height_mm  Height of the completed hump.
 *
 * @return true if the hump was completed.
 */
static bool
step_descending (hump_t * p_hump, int32_t pitch_mdeg, uint32_t dt_ms,
                 uint32_t * p_height_mm)
{
    bool b_done = false;

    p_hump->timer_ms = (fix_math_abs(pitch_mdeg) < p_hump->exit_mdeg) ?
                       (p_hump->timer_ms + dt_ms) : 0u;

    if (p_hump->timer_ms >= p_hump->hold_ms)
    {
        // peak_um is never negative, so the conversion is exact.
        uint32_t const height = (uint32_t) (p_hump->peak_um / UM_PER_MM);

        *p_height_mm   = height;
        p_hump->max_mm = (height > p_hump->max_mm) ? height : p_hump->max_mm;
        p_hump->count++;
        restart(p_hump);
        b_done = true;
    }

    return (b_done);
}

/*** end of file ***/


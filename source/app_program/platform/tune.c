/** @file tune.c
 *
 * @brief Live-tunable settings, changed over MQTT while the robot runs.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "robot_cfg.h"
#include "tune.h"

/*!
 * @brief Accepted range of one setting.
 */
typedef struct
{
    int32_t min;
    int32_t max;
} tune_range_t;

// Current values, starting from robot_cfg.h.  Every configured value is
// below 2^16, so each cast to int32_t is exact.
static int32_t volatile g_values[TUNE_COUNT] =
{
    (int32_t) MOTOR_MAX_PERMILLE,       // Cast: unsigned constant, exact.
    (int32_t) SPEED_FOLLOW_MM_S,        // Cast: unsigned constant, exact.
    (int32_t) SPEED_SLOW_MM_S,          // Cast: unsigned constant, exact.
    (int32_t) SPEED_TURN_MM_S,          // Cast: unsigned constant, exact.
    STEER_KP_MILLI,
    STEER_KD_MILLI,
    STEER_DIRECTION,
    (int32_t) ENC_UM_PER_EDGE,          // Cast: unsigned constant, exact.
    (int32_t) TRACK_WIDTH_MM,           // Cast: unsigned constant, exact.
    (int32_t) REVERSE_DISTANCE_MM       // Cast: unsigned constant, exact.
};

static tune_range_t const g_ranges[TUNE_COUNT] =
{
    { 250, 1000 },      // TUNE_MOTOR_MAX: below 250 the wheels may not turn.
    { 30, 600 },        // TUNE_SPEED_FOLLOW
    { 30, 600 },        // TUNE_SPEED_SLOW
    { 30, 600 },        // TUNE_SPEED_TURN
    { 0, 3000 },        // TUNE_STEER_KP
    { 0, 3000 },        // TUNE_STEER_KD
    { -1, 1 },          // TUNE_STEER_DIR (0 rejected below)
    { 50, 30000 },      // TUNE_ENC_UM_PER_EDGE
    { 50, 300 },        // TUNE_TRACK_MM
    { 0, 500 }          // TUNE_REVERSE_MM
};


/*!
 * @brief Current value of one setting.
 *
 * @param[in] param  Which setting.
 *
 * @return The value (0 for an unknown setting).
 */
int32_t
tune_get (tune_param_t param)
{
    return ((param < TUNE_COUNT) ? g_values[param] : 0);
}


/*!
 * @brief Change one setting if the value is in range.
 *
 * @param[in] param  Parameter id (tune_param_t value from MQTT).
 * @param[in] value  New value.
 *
 * @return true if applied; false if the id or value was rejected.
 */
bool
tune_set (uint32_t param, int32_t value)
{
    // Cast: compare the unsigned id with the enumeration's count.
    bool b_ok = (param < (uint32_t) TUNE_COUNT);

    if (b_ok)
    {
        b_ok = (value >= g_ranges[param].min) &&
               (value <= g_ranges[param].max) &&
               ((TUNE_STEER_DIR != param) || (0 != value));
    }

    if (b_ok)
    {
        g_values[param] = value;  // One aligned store: atomic.
    }

    return (b_ok);
}

/*** end of file ***/


/** @file obs_profile.h
 *
 * @brief Obstacle profiling and avoidance planning from a servo scan
 *        (Buddy 5, brief stages 3 and 4).
 *
 * @par
 * Pure module, unit-tested on the host.  Each scan point is converted to
 * robot coordinates (x lateral, positive to the left; y forward).  The
 * obstacle is every in-range point no deeper than OBSTACLE_DEPTH_MM behind
 * the closest one; its edges give the width, and the free lateral space
 * beyond each edge gives the clearance.  The plan passes on the side that
 * has room for the vehicle plus margin and deviates least from the line
 * (NFR9); with no room on either side it asks to reverse and retry.
 * Following Week 6 Section 6.6, a missing echo means nothing within
 * SONAR_MAX_MM; samples where the sensor did not respond at all (FAULT) are
 * left out of the scan by the caller, so a dead sensor never looks clear.
 *
 * @par
 * Profile message on mbf_obs_profile, 6 bytes, LSB first:
 *   [7:0] closest distance cm, [15:8] width cm, [21:16] closest angle / 5,
 *   [29:22] left clearance cm, [37:30] right clearance cm, [40:38] plan,
 *   [41] valid.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef OBS_PROFILE_H
#define OBS_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

#define OBS_PROFILE_MSG_BYTES       6u
#define OBS_PROFILE_MAX_POINTS      40u

/*!
 * @brief Avoidance plan (3 bits), brief stage 4.
 */
typedef enum
{
    OBS_PLAN_CONTINUE = 0,
    OBS_PLAN_STOP,
    OBS_PLAN_PASS_LEFT,
    OBS_PLAN_PASS_RIGHT,
    OBS_PLAN_REVERSE
} obs_profile_plan_t;

/*!
 * @brief One scan sample.
 */
typedef struct
{
    uint32_t    angle_deg;      // Servo angle, 90 = straight ahead.
    uint32_t    distance_mm;    // Valid only if b_echo is true.
    bool        b_echo;         // false: nothing within SONAR_MAX_MM.
} obs_profile_point_t;

/*!
 * @brief Obstacle profile (brief stage 3) and plan (stage 4).
 */
typedef struct
{
    uint32_t           distance_mm;     // Closest point.
    uint32_t           angle_deg;       // Servo angle of the closest point.
    uint32_t           width_mm;
    uint32_t           left_clear_mm;
    uint32_t           right_clear_mm;
    obs_profile_plan_t plan;
    bool               b_valid;
} obs_profile_t;

void obs_profile_compute(obs_profile_point_t const * p_points,
                         uint32_t count, obs_profile_t * p_profile);
void obs_profile_pack(obs_profile_t const * p_profile, uint8_t * p_msg);
void obs_profile_unpack(uint8_t const * p_msg, obs_profile_t * p_profile);

#endif /* OBS_PROFILE_H */

/*** end of file ***/


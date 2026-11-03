/** @file obstacle.h
 *
 * @brief Obstacle Scanning Task: forward monitoring, staged coarse and
 *        fine scanning, and profiling (Buddy 5, Week 6 Section 6.6).
 *
 * @par
 * MONITOR pings straight ahead every MONITOR_PERIOD_MS.  Two readings in a
 * row below OBSTACLE_DETECT_MM set ROBOT_EVT_OBSTACLE.  On SCAN_REQ
 * (obstacle_request_scan) the task runs a coarse scan at 30, 60, 90, 120
 * and 150 degrees, a fine 5-degree scan across the obstacle, computes the
 * profile and plan, sends it on mbf_obs_profile and sets
 * ROBOT_EVT_OBS_PROFILE.  Three sensor faults in a row enter ECHO FAULT
 * (ROBOT_FAULT_NO_ECHO and ROBOT_EVT_ECHO_FAULT).
 *
 * @par
 * Status word SYS_STATUS_OBSTACLE (LSB first): [7:0] forward distance cm
 * (255 = nothing in range), [10:8] state, [13:11] last plan,
 * [19:14] profiles made (saturating), [23:20] echo faults (saturating).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>
#include <stdbool.h>

#include "obs_profile.h"

#define OBSTACLE_DIST_SHIFT         0u
#define OBSTACLE_DIST_WIDTH         8u
#define OBSTACLE_STATE_SHIFT        8u
#define OBSTACLE_STATE_WIDTH        3u
#define OBSTACLE_PLAN_SHIFT         11u
#define OBSTACLE_PLAN_WIDTH         3u
#define OBSTACLE_PROFILES_SHIFT     14u
#define OBSTACLE_PROFILES_WIDTH     6u
#define OBSTACLE_FAULTS_SHIFT       20u
#define OBSTACLE_FAULTS_WIDTH       4u

/*!
 * @brief Obstacle Scanning states (3 bits).
 */
typedef enum
{
    OBSTACLE_STATE_MONITOR = 0,
    OBSTACLE_STATE_DETECTED,
    OBSTACLE_STATE_COARSE_SCAN,
    OBSTACLE_STATE_FINE_SCAN,
    OBSTACLE_STATE_PROFILE,
    OBSTACLE_STATE_ECHO_FAULT
} obstacle_state_t;

int32_t obstacle_start(void);
void    obstacle_request_scan(void);
bool    obstacle_get_profile(obs_profile_t * p_profile);

#endif /* OBSTACLE_H */

/*** end of file ***/


/** @file vehicle.h
 *
 * @brief Vehicle Control Task: the system-level statechart that coordinates
 *        every subsystem (Week 6 Section 5).
 *
 * @par
 * Owner: integration (shared; named owner to be confirmed by the team).
 * Vehicle Control never drives hardware: it commands Motion Control, asks
 * Obstacle Scanning for profiles and owns the barcode map (FR10).  It is
 * the only task waiting on flg_vehicle_evt.
 *
 * @par
 * Simultaneous events are handled in this order (Tribunal 1 decision):
 * safety (impact, stall, fatal fault) first, then obstacle, then commands,
 * then barcodes, then junction and line events.  A latched barcode command
 * survives an obstacle bypass and is used at the next junction.  Network
 * loss never causes a transition (NFR1, NFR17).
 *
 * @par
 * Status word SYS_STATUS_VEHICLE (LSB first): [3:0] state, [5:4] latched
 * action, [6] action latched, [10:7] map version, [16:11] junctions done
 * (saturating), [20:17] bypasses done (saturating), [22:21] retries.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdint.h>

#define VEHICLE_STATE_SHIFT         0u
#define VEHICLE_STATE_WIDTH         4u
#define VEHICLE_ACTION_SHIFT        4u
#define VEHICLE_ACTION_WIDTH        2u
#define VEHICLE_LATCHED_SHIFT       6u
#define VEHICLE_MAP_SHIFT           7u
#define VEHICLE_MAP_WIDTH           4u
#define VEHICLE_JUNCTIONS_SHIFT     11u
#define VEHICLE_JUNCTIONS_WIDTH     6u
#define VEHICLE_BYPASSES_SHIFT      17u
#define VEHICLE_BYPASSES_WIDTH      4u
#define VEHICLE_RETRIES_SHIFT       21u
#define VEHICLE_RETRIES_WIDTH       2u

/*!
 * @brief System states (4 bits), Week 6 Figure 5.
 */
typedef enum
{
    VEHICLE_INITIALISING = 0,
    VEHICLE_READY,
    VEHICLE_CALIBRATING,
    VEHICLE_LINE_FOLLOWING,
    VEHICLE_JUNCTION_ACTION,
    VEHICLE_OBSTACLE_SCAN,
    VEHICLE_BYPASS,
    VEHICLE_LINE_REACQUIRE,
    VEHICLE_REVERSE_RETRY,
    VEHICLE_SAFE_STOP,
    VEHICLE_MISSION_COMPLETE
} vehicle_state_t;

int32_t vehicle_start(void);

#endif /* VEHICLE_H */

/*** end of file ***/


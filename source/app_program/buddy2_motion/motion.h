/** @file motion.h
 *
 * @brief Motion Control Task and the reusable motion API (Buddy 2).
 *
 * @par
 * Other subsystems never drive the motors.  They call the functions below,
 * which pack a command (Change Requirement 1: 29 bits in one 32-bit word)
 * and queue it on mbf_motion_cmd.  A new command pre-empts the current one.
 * When a MOVE, TURN or STOP finishes, the task sets ROBOT_EVT_MOVE_DONE; on
 * a stall it stops the motors and sets ROBOT_EVT_STALL.
 *
 * @par
 * Command word (LSB first): [2:0] type, [12:3] speed mm/s,
 * [28:13] signed argument (excess-32768): distance in mm for MOVE, angle in
 * degrees for TURN (positive = left / anticlockwise).
 *
 * @par
 * Status word SYS_STATUS_MOTION (LSB first): [2:0] mode, [13:3] left speed
 * mm/s (excess-1024), [24:14] right speed mm/s (excess-1024), [25] stalled.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>

// Command word layout.
#define MOTION_CMD_TYPE_SHIFT       0u
#define MOTION_CMD_TYPE_WIDTH       3u
#define MOTION_CMD_SPEED_SHIFT      3u
#define MOTION_CMD_SPEED_WIDTH      10u
#define MOTION_CMD_ARG_SHIFT        13u
#define MOTION_CMD_ARG_WIDTH        16u

// Status word layout.
#define MOTION_MODE_SHIFT           0u
#define MOTION_MODE_WIDTH           3u
#define MOTION_LEFT_SHIFT           3u
#define MOTION_RIGHT_SHIFT          14u
#define MOTION_SPEED_WIDTH          11u
#define MOTION_STALLED_SHIFT        25u

/*!
 * @brief Command types (3 bits).
 */
typedef enum
{
    MOTION_CMD_STOP = 0,
    MOTION_CMD_FOLLOW,
    MOTION_CMD_MOVE,
    MOTION_CMD_TURN
} motion_cmd_type_t;

/*!
 * @brief Motion Control states (3 bits), Week 6 Section 6.2.
 */
typedef enum
{
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_FOLLOWING,
    MOTION_MODE_MOVING,
    MOTION_MODE_TURNING,
    MOTION_MODE_BRAKING,
    MOTION_MODE_STALLED
} motion_mode_t;

int32_t  motion_start(void);

bool     motion_move_forward(uint32_t distance_mm, uint32_t speed_mm_s);
bool     motion_move_backward(uint32_t distance_mm, uint32_t speed_mm_s);
bool     motion_turn_left(uint32_t angle_deg);
bool     motion_turn_right(uint32_t angle_deg);
bool     motion_follow_line(uint32_t speed_mm_s);
bool     motion_stop(void);

uint32_t motion_pack_command(motion_cmd_type_t type, uint32_t speed_mm_s,
                             int32_t argument);

#endif /* MOTION_H */

/*** end of file ***/


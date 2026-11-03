/** @file line_follow.h
 *
 * @brief Line Following Task: line position, junction, line-lost and
 *        line-found detection (Buddy 3, Week 6 Section 6.3).
 *
 * @par
 * Sensors: left IR1 digital (GP3) and centre IR2 analogue (GP27 / ADC1).
 * The centre sensor tracks the right-hand edge of the black line, so its
 * normalised reading (0 = white, 1000 = black) minus 500 is a signed
 * offset: positive means the robot has drifted left onto the line and must
 * steer right.  The left sensor seeing black alone means the line has moved
 * well to the left.
 *
 * @par
 * line_state word (SYS_STATUS_LINE, LSB first, Change Requirement 1):
 *   [10:0]  offset, -1000..+1000 (excess-1024)
 *   [22:11] centre ADC raw value, 0..4095
 *   [23]    left sensor sees black
 *   [26:24] line state (line_follow_state_t)
 *   [31:27] timestamp in 10 ms ticks, modulo 32 (freshness check)
 * The word is built locally and published with one aligned store, so the
 * Motion Control reader always sees a consistent set of fields.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>
#include <stdbool.h>

#define LINE_OFFSET_SHIFT           0u
#define LINE_OFFSET_WIDTH           11u
#define LINE_CENTRE_SHIFT           11u
#define LINE_CENTRE_WIDTH           12u
#define LINE_LEFT_SHIFT             23u
#define LINE_STATE_SHIFT            24u
#define LINE_STATE_WIDTH            3u
#define LINE_STAMP_SHIFT            27u
#define LINE_STAMP_WIDTH            5u

// Normalised offset range.
#define LINE_OFFSET_LIMIT           1000

/*!
 * @brief Line Following states (3 bits).
 */
typedef enum
{
    LINE_STATE_CALIBRATING = 0,
    LINE_STATE_TRACKING,
    LINE_STATE_JUNCTION,
    LINE_STATE_LOST,
    LINE_STATE_SEARCHING
} line_follow_state_t;

int32_t             line_follow_start(void);
uint32_t            line_follow_read(void);
int32_t             line_follow_offset(uint32_t word);
line_follow_state_t line_follow_state(uint32_t word);
uint32_t            line_follow_age_ms(uint32_t word);
void                line_follow_begin_calibration(void);
bool                line_follow_end_calibration(void);
void                line_follow_clear_junction(void);

#endif /* LINE_FOLLOW_H */

/*** end of file ***/


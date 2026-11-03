/** @file barcode.h
 *
 * @brief Barcode Task: edge capture from the dedicated barcode IR sensor
 *        (GP28) and Code 39 decoding (Buddy 3, Week 6 Section 6.4).
 *
 * @par
 * The GPIO interrupt only stores each element's duration in a bounded FIFO
 * and wakes the task.  The task ends a capture after a quiet zone, decodes
 * it, and sends only the decoded letter to Vehicle Control, which owns the
 * letter-to-action map (FR10).  Any doubt is a REJECT: nothing is sent, so
 * a misread cannot turn the robot the wrong way.
 *
 * @par
 * Navigation message on mbf_nav_cmd: 1 byte, LSB first:
 *   [4:0] letter 0=A..25=Z, [5] read backwards, [7:6] sequence number.
 *
 * @par
 * Status word SYS_STATUS_BARCODE (LSB first): [4:0] last letter,
 * [5] reversed, [7:6] sequence, [15:8] reads (saturating), [23:16] rejects
 * (saturating), [26:24] task state.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>

#define BARCODE_MSG_WIDTH           8u
#define BARCODE_LETTER_SHIFT        0u
#define BARCODE_LETTER_WIDTH        5u
#define BARCODE_REVERSED_SHIFT      5u
#define BARCODE_SEQ_SHIFT           6u
#define BARCODE_SEQ_WIDTH           2u
#define BARCODE_READS_SHIFT         8u
#define BARCODE_REJECTS_SHIFT       16u
#define BARCODE_COUNT_WIDTH         8u
#define BARCODE_STATE_SHIFT         24u
#define BARCODE_STATE_WIDTH         3u

// DECODE_FAIL event data: reject reason, then the element count.
#define BARCODE_REJECT_REASON_WIDTH 3u
#define BARCODE_REJECT_COUNT_SHIFT  3u
#define BARCODE_REJECT_COUNT_WIDTH  6u

/*!
 * @brief Barcode Task states (3 bits).
 */
typedef enum
{
    BARCODE_STATE_IDLE = 0,
    BARCODE_STATE_CAPTURING,
    BARCODE_STATE_DECODING,
    BARCODE_STATE_REPORT,
    BARCODE_STATE_REJECT
} barcode_state_t;

/*!
 * @brief Reasons a capture was rejected (3 bits, telemetry DECODE_FAIL).
 */
typedef enum
{
    BARCODE_REJECT_OVERFLOW = 0,
    BARCODE_REJECT_COUNT,
    BARCODE_REJECT_RATIO,
    BARCODE_REJECT_NO_MATCH,
    BARCODE_REJECT_FRAME,
    BARCODE_REJECT_START
} barcode_reject_t;

int32_t  barcode_start(void);
uint32_t barcode_msg_letter(uint8_t msg);
void     barcode_debug_levels(uint32_t * p_dark, uint32_t * p_white,
                              uint32_t * p_black);

#endif /* BARCODE_H */

/*** end of file ***/


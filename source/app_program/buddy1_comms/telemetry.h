/** @file telemetry.h
 *
 * @brief Telemetry Task: periodic status frames, heartbeat, event and
 *        summary messages, queued for the Communication side (Buddy 1).
 *
 * @par
 * Producers report one-off events with telemetry_post_event().  The call
 * never blocks (zero timeout, Week 6 NFR17): when mbf_tlm_event is full the
 * event is dropped and counted (telemetry_dropped_events), so producers
 * have nothing to check.  Each event is one 32-bit word, LSB first:
 * [3:0] event type, [31:4] type-specific data (layouts below).
 *
 * @par
 * Event data layouts (bits of the 28-bit data field):
 *   BARCODE      [4:0] letter 0=A..25=Z, [5] reversed, [7:6] sequence
 *   DECODE_FAIL  [2:0] reason (barcode_reject_t), [8:3] element count
 *   HUMP         [7:0] hump height mm, [15:8] running maximum mm
 *   IMPACT       [7:0] peak acceleration in units of 16 mg
 *   OBSTACLE     [7:0] distance cm
 *   PROFILE      [2:0] plan, [10:3] width cm, [18:11] distance cm
 *   STATE        [3:0] new vehicle state, [7:4] previous state
 *   FAULT        [7:0] fault bitmask
 *   JUNCTION     [1:0] action executed
 *   MAP_ACK      [4:0] letter, [6:5] action, [10:7] map version, [11] ok
 *   LINE         [0] 1 = line lost, 0 = line found
 *
 * @par
 * HEALTH status word (owned by this task, LSB first): [7:0] missed periods
 * (saturating), [15:8] kernel errors (saturating), [23:16] fault bitmask.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_EVT_TYPE_WIDTH    4u
#define TELEMETRY_EVT_DATA_SHIFT    4u
#define TELEMETRY_EVT_DATA_WIDTH    28u

/*!
 * @brief Event types (4 bits).
 */
typedef enum
{
    TELEMETRY_EVT_BARCODE = 0,
    TELEMETRY_EVT_DECODE_FAIL,
    TELEMETRY_EVT_HUMP,
    TELEMETRY_EVT_IMPACT,
    TELEMETRY_EVT_OBSTACLE,
    TELEMETRY_EVT_PROFILE,
    TELEMETRY_EVT_STATE,
    TELEMETRY_EVT_FAULT,
    TELEMETRY_EVT_JUNCTION,
    TELEMETRY_EVT_MAP_ACK,
    TELEMETRY_EVT_LINE,
    TELEMETRY_EVT_TUNE
} telemetry_evt_t;

int32_t  telemetry_start(void);
void     telemetry_post_event(telemetry_evt_t type, uint32_t data);
uint32_t telemetry_dropped_events(void);

#endif /* TELEMETRY_H */

/*** end of file ***/


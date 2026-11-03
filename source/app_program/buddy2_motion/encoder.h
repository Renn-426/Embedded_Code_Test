/** @file encoder.h
 *
 * @brief Wheel encoders: edge counting in the GPIO interrupt and the
 *        periodic Encoder Task that publishes speed and distance
 *        (Week 6 "enc_state").
 *
 * @par
 * Owner: Buddy 2 (Motion Control).  Readers: Motion Control, IMU (distance
 * for hump height), Telemetry.
 *
 * @par
 * Status words written (sys_status, LSB first):
 *   ENC_COUNTS  [15:0] left edge count, [31:16] right edge count (wrapping)
 *   ODOMETRY    [15:0] distance travelled in cm (wrapping)
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#define ENCODER_COUNT_WIDTH         16u
#define ENCODER_LEFT_SHIFT          0u
#define ENCODER_RIGHT_SHIFT         16u
#define ENCODER_DISTANCE_WIDTH      16u

/*!
 * @brief Snapshot of both wheels (the shared enc_state).
 */
typedef struct
{
    int32_t     left_count;         // Signed edge count since boot.
    int32_t     right_count;
    int32_t     left_speed_mm_s;    // Signed wheel speed.
    int32_t     right_speed_mm_s;
    int32_t     distance_mm;        // Mean signed travel since boot.
    uint32_t    left_edge_age_ms;   // Time since the last edge.
    uint32_t    right_edge_age_ms;
    uint32_t    time_us;            // When this snapshot was taken.
} encoder_state_t;

int32_t encoder_start(void);
void    encoder_set_direction(int32_t left_sign, int32_t right_sign);
void    encoder_get_state(encoder_state_t * p_state);

#endif /* ENCODER_H */

/*** end of file ***/


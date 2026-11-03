/** @file codec_vectors.c
 *
 * @brief Prints known MQTT frames for tests/test_decode.py, which checks
 *        that tools/tlm_decode.py decodes exactly what the firmware sends.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "bitpack.h"
#include "sys_status.h"
#include "tlm_codec.h"
#include "vehicle.h"
#include "motion.h"
#include "line_follow.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "comms.h"
#include "telemetry.h"
#include "nav_map.h"

static void print_frame(char const * p_name, uint8_t const * p_frame,
                        uint32_t length);


/*!
 * @brief Emit one telemetry frame and one event frame as hex.
 *
 * @return 0.
 */
int
main (void)
{
    uint32_t words[SYS_STATUS_WORD_COUNT] = { 0u };
    uint8_t  frame[TLM_CODEC_MAX_FRAME];
    uint32_t length = 0u;
    uint32_t data   = 0u;

    words[SYS_STATUS_VEHICLE] = bitpack_word_set(0u, VEHICLE_STATE_SHIFT,
                                                 VEHICLE_STATE_WIDTH,
                                                 VEHICLE_LINE_FOLLOWING);
    words[SYS_STATUS_VEHICLE] = bitpack_word_set(words[SYS_STATUS_VEHICLE],
                                                 VEHICLE_MAP_SHIFT,
                                                 VEHICLE_MAP_WIDTH, 3u);
    words[SYS_STATUS_MOTION]  = bitpack_word_set(0u, MOTION_LEFT_SHIFT,
                                                 MOTION_SPEED_WIDTH,
                                                 bitpack_from_signed(-150,
                                                 MOTION_SPEED_WIDTH));
    words[SYS_STATUS_ODOMETRY] = 1234u;
    words[SYS_STATUS_LINE]    = bitpack_word_set(0u, LINE_OFFSET_SHIFT,
                                                 LINE_OFFSET_WIDTH,
                                                 bitpack_from_signed(320,
                                                 LINE_OFFSET_WIDTH));
    words[SYS_STATUS_BARCODE] = bitpack_word_set(0u, BARCODE_LETTER_SHIFT,
                                                 BARCODE_LETTER_WIDTH, 3u);
    words[SYS_STATUS_IMU]     = bitpack_word_set(
                                    bitpack_word_set(0u, IMU_PITCH_SHIFT,
                                        IMU_PITCH_WIDTH,
                                        bitpack_from_signed(12,
                                                            IMU_PITCH_WIDTH)),
                                    IMU_HEADING_SHIFT, IMU_HEADING_WIDTH,
                                    270u);
    words[SYS_STATUS_HUMP]    = bitpack_word_set(0u, IMU_HUMP_MAX_SHIFT,
                                                 IMU_HUMP_MM_WIDTH, 17u);
    words[SYS_STATUS_OBSTACLE] = bitpack_word_set(0u, OBSTACLE_PLAN_SHIFT,
                                                  OBSTACLE_PLAN_WIDTH,
                                                  OBS_PLAN_PASS_LEFT);
    words[SYS_STATUS_COMMS]   = bitpack_word_set(0u, COMMS_LINK_SHIFT,
                                                 COMMS_LINK_WIDTH,
                                                 COMMS_LINK_ONLINE);
    words[SYS_STATUS_HEALTH]  = bitpack_word_set(0u, 16u, 8u, 0x04u);

    length = tlm_codec_telemetry(42u, words, frame, sizeof(frame));
    print_frame("telemetry", frame, length);

    data   = bitpack_word_set(0u, 0u, 5u, 1u);
    data   = bitpack_word_set(data, 5u, 2u, NAV_ACTION_UTURN);
    data   = bitpack_word_set(data, 7u, 4u, 2u);
    data   = bitpack_word_set(data, 11u, 1u, 1u);
    data   = bitpack_word_set(0u, 0u, 4u, TELEMETRY_EVT_MAP_ACK) |
             (data << 4);
    length = tlm_codec_event(9u, data, frame, sizeof(frame));
    print_frame("event", frame, length);

    return (0);
}


/*!
 * @brief Print a frame as "name hex...".
 *
 * @param[in] p_name   Label.
 * @param[in] p_frame  Bytes.
 * @param[in] length   Byte count.
 */
static void
print_frame (char const * p_name, uint8_t const * p_frame, uint32_t length)
{
    printf("%s ", p_name);

    for (uint32_t idx = 0u; idx < length; idx++)
    {
        printf("%02x", p_frame[idx]);
    }

    printf("\n");
}

/*** end of file ***/

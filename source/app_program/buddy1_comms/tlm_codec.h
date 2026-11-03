/** @file tlm_codec.h
 *
 * @brief Compact binary MQTT payloads (Buddy 1, Change Requirement 1).
 *
 * @par
 * Pure module, unit-tested on the host and mirrored by tools/tlm_decode.py.
 * Every payload starts with a header byte: [3:0] frame type, [7:4] protocol
 * version.  Fields follow least-significant bit first with no padding.
 *
 * @par
 * Outgoing frames:
 *   TELEMETRY  seq(8), then every sys_status word at its used width
 *              (sys_status_width), in sys_status_word_t order.
 *   HEARTBEAT  seq(8), uptime s(16), vehicle state(4), faults(8).
 *   EVENT      seq(8), event word(32) (telemetry.h layouts).
 *   SUMMARY    max hump mm(8), humps(6), barcodes(8), rejects(8),
 *              obstacles(6), distance cm(16), run time s(16), faults(8).
 *   ACK        command type(4), accepted(1).
 *
 * @par
 * Incoming command frame (header frame type 8):
 *   byte 1: [3:0] command (tlm_codec_cmd_t), [7:4] map version.
 *   byte 2 (MAP_SET only): [4:0] letter 0..25, [6:5] action.
 *   TUNE: byte 1 [7:4] is the setting id (tune.h) instead of the map
 *   version; bytes 2-3 are the new value, signed 16-bit little-endian.
 * START, STOP, RESET and SUMMARY are 2 bytes; MAP_SET is 3; TUNE is 4.
 *
 * @par
 * Command word on mbf_ctrl_cmd (LSB first): [3:0] command, [8:4] letter,
 * [10:9] action, [14:11] map version.  TUNE word: [3:0] command,
 * [7:4] setting id, [23:8] value (excess-32768).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef TLM_CODEC_H
#define TLM_CODEC_H

#include <stdint.h>
#include <stdbool.h>

#define TLM_CODEC_VERSION           1u
#define TLM_CODEC_MAX_FRAME         39u

#define TLM_CMD_TYPE_SHIFT          0u
#define TLM_CMD_TYPE_WIDTH          4u
#define TLM_CMD_LETTER_SHIFT        4u
#define TLM_CMD_LETTER_WIDTH        5u
#define TLM_CMD_ACTION_SHIFT        9u
#define TLM_CMD_ACTION_WIDTH        2u
#define TLM_CMD_VERSION_SHIFT       11u
#define TLM_CMD_VERSION_WIDTH       4u
#define TLM_CMD_PARAM_SHIFT         4u
#define TLM_CMD_PARAM_WIDTH         4u
#define TLM_CMD_VALUE_SHIFT         8u
#define TLM_CMD_VALUE_WIDTH         16u

/*!
 * @brief MQTT topics (suffixes are defined in comms_net.c).
 */
typedef enum
{
    TLM_TOPIC_TELEMETRY = 0,
    TLM_TOPIC_HEARTBEAT,
    TLM_TOPIC_EVENT,
    TLM_TOPIC_SUMMARY,
    TLM_TOPIC_ACK,
    TLM_TOPIC_COUNT
} tlm_codec_topic_t;

/*!
 * @brief Inbound command types (4 bits).
 */
typedef enum
{
    TLM_CMD_START = 0,
    TLM_CMD_STOP,
    TLM_CMD_RESET,
    TLM_CMD_SUMMARY,
    TLM_CMD_MAP_SET,
    TLM_CMD_TUNE,
    TLM_CMD_INVALID
} tlm_codec_cmd_t;

/*!
 * @brief Run summary (FR5, FR14).
 */
typedef struct
{
    uint32_t    max_hump_mm;
    uint32_t    humps;
    uint32_t    barcodes;
    uint32_t    rejects;
    uint32_t    obstacles;
    uint32_t    distance_cm;
    uint32_t    run_time_s;
    uint32_t    faults;
} tlm_codec_summary_t;

uint32_t        tlm_codec_telemetry(uint32_t seq, uint32_t const * p_words,
                                    uint8_t * p_out, uint32_t capacity);
uint32_t        tlm_codec_heartbeat(uint32_t seq, uint32_t uptime_s,
                                    uint32_t state, uint32_t faults,
                                    uint8_t * p_out, uint32_t capacity);
uint32_t        tlm_codec_event(uint32_t seq, uint32_t event_word,
                                uint8_t * p_out, uint32_t capacity);
uint32_t        tlm_codec_summary(tlm_codec_summary_t const * p_summary,
                                  uint8_t * p_out, uint32_t capacity);
uint32_t        tlm_codec_ack(uint32_t command, bool b_accepted,
                              uint8_t * p_out, uint32_t capacity);
tlm_codec_cmd_t tlm_codec_parse_command(uint8_t const * p_in,
                                        uint32_t length, uint32_t * p_word);

#endif /* TLM_CODEC_H */

/*** end of file ***/


/** @file tlm_codec.c
 *
 * @brief Compact binary MQTT payloads.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "bitpack.h"
#include "sys_status.h"
#include "tlm_codec.h"

#define FRAME_TELEMETRY         0u
#define FRAME_HEARTBEAT         1u
#define FRAME_EVENT             2u
#define FRAME_SUMMARY           3u
#define FRAME_ACK               4u
#define FRAME_COMMAND           8u
#define TYPE_WIDTH              4u
#define VERSION_WIDTH           4u
#define SEQ_WIDTH               8u
#define UPTIME_WIDTH            16u
#define STATE_WIDTH             4u
#define FAULT_WIDTH             8u
#define EVENT_WIDTH             32u
#define HUMP_WIDTH              8u
#define COUNT6_WIDTH            6u
#define COUNT8_WIDTH            8u
#define WORD16_WIDTH            16u
#define LETTER_COUNT            26u
#define COMMAND_BYTES           2u
#define MAP_SET_BYTES           3u
#define TUNE_BYTES              4u
#define VALUE_SIGN_BIT          0x8000u
#define WORD_COUNT              ((uint32_t) SYS_STATUS_WORD_COUNT)  // Exact.

static void            start_frame(bitpack_t * p_pack, uint8_t * p_out,
                                   uint32_t capacity, uint32_t type);
static uint32_t        finish_frame(bitpack_t const * p_pack);
static tlm_codec_cmd_t command_type(uint32_t type, uint32_t letter,
                                    uint32_t length);


/*!
 * @brief Build a TELEMETRY frame from the status words.
 *
 * @param[in]  seq       Frame sequence number (low 8 bits used).
 * @param[in]  p_words   SYS_STATUS_WORD_COUNT packed status words.
 * @param[out] p_out     Output buffer.
 * @param[in]  capacity  Output buffer size.
 *
 * @return Frame length in bytes, or 0 if it did not fit.
 */
uint32_t
tlm_codec_telemetry (uint32_t seq, uint32_t const * p_words, uint8_t * p_out,
                     uint32_t capacity)
{
    bitpack_t pack;

    start_frame(&pack, p_out, capacity, FRAME_TELEMETRY);
    bitpack_put(&pack, seq, SEQ_WIDTH);

    for (uint32_t idx = 0u; idx < WORD_COUNT; idx++)
    {
        // idx is below the enumeration count, so the cast is exact.
        bitpack_put(&pack, p_words[idx],
                    sys_status_width((sys_status_word_t) idx));
    }

    return (finish_frame(&pack));
}


/*!
 * @brief Build a HEARTBEAT frame.
 *
 * @param[in]  seq       Frame sequence number.
 * @param[in]  uptime_s  Seconds since boot (low 16 bits).
 * @param[in]  state     Vehicle Control state.
 * @param[in]  faults    Fault bitmask.
 * @param[out] p_out     Output buffer.
 * @param[in]  capacity  Output buffer size.
 *
 * @return Frame length in bytes, or 0 if it did not fit.
 */
uint32_t
tlm_codec_heartbeat (uint32_t seq, uint32_t uptime_s, uint32_t state,
                     uint32_t faults, uint8_t * p_out, uint32_t capacity)
{
    bitpack_t pack;

    start_frame(&pack, p_out, capacity, FRAME_HEARTBEAT);
    bitpack_put(&pack, seq, SEQ_WIDTH);
    bitpack_put(&pack, uptime_s, UPTIME_WIDTH);
    bitpack_put(&pack, state, STATE_WIDTH);
    bitpack_put(&pack, faults, FAULT_WIDTH);

    return (finish_frame(&pack));
}


/*!
 * @brief Build an EVENT frame.
 *
 * @param[in]  seq         Frame sequence number.
 * @param[in]  event_word  Event (telemetry.h layout).
 * @param[out] p_out       Output buffer.
 * @param[in]  capacity    Output buffer size.
 *
 * @return Frame length in bytes, or 0 if it did not fit.
 */
uint32_t
tlm_codec_event (uint32_t seq, uint32_t event_word, uint8_t * p_out,
                 uint32_t capacity)
{
    bitpack_t pack;

    start_frame(&pack, p_out, capacity, FRAME_EVENT);
    bitpack_put(&pack, seq, SEQ_WIDTH);
    bitpack_put(&pack, event_word, EVENT_WIDTH);

    return (finish_frame(&pack));
}


/*!
 * @brief Build a SUMMARY frame.
 *
 * @param[in]  p_summary  Run summary; each value saturates to its field.
 * @param[out] p_out      Output buffer.
 * @param[in]  capacity   Output buffer size.
 *
 * @return Frame length in bytes, or 0 if it did not fit.
 */
uint32_t
tlm_codec_summary (tlm_codec_summary_t const * p_summary, uint8_t * p_out,
                   uint32_t capacity)
{
    bitpack_t pack;

    start_frame(&pack, p_out, capacity, FRAME_SUMMARY);
    bitpack_put(&pack, bitpack_saturate(p_summary->max_hump_mm, HUMP_WIDTH),
                HUMP_WIDTH);
    bitpack_put(&pack, bitpack_saturate(p_summary->humps, COUNT6_WIDTH),
                COUNT6_WIDTH);
    bitpack_put(&pack, bitpack_saturate(p_summary->barcodes, COUNT8_WIDTH),
                COUNT8_WIDTH);
    bitpack_put(&pack, bitpack_saturate(p_summary->rejects, COUNT8_WIDTH),
                COUNT8_WIDTH);
    bitpack_put(&pack, bitpack_saturate(p_summary->obstacles, COUNT6_WIDTH),
                COUNT6_WIDTH);
    bitpack_put(&pack, bitpack_saturate(p_summary->distance_cm,
                                        WORD16_WIDTH), WORD16_WIDTH);
    bitpack_put(&pack, bitpack_saturate(p_summary->run_time_s, WORD16_WIDTH),
                WORD16_WIDTH);
    bitpack_put(&pack, p_summary->faults, FAULT_WIDTH);

    return (finish_frame(&pack));
}


/*!
 * @brief Build an ACK frame (command received, accepted or refused).
 *
 * @param[in]  command     Command type field of the received command.
 * @param[in]  b_accepted  true if the command was valid.
 * @param[out] p_out       Output buffer.
 * @param[in]  capacity    Output buffer size.
 *
 * @return Frame length in bytes, or 0 if it did not fit.
 */
uint32_t
tlm_codec_ack (uint32_t command, bool b_accepted, uint8_t * p_out,
               uint32_t capacity)
{
    bitpack_t pack;

    start_frame(&pack, p_out, capacity, FRAME_ACK);
    bitpack_put(&pack, command, TLM_CMD_TYPE_WIDTH);
    bitpack_put(&pack, b_accepted ? 1u : 0u, 1u);

    return (finish_frame(&pack));
}


/*!
 * @brief Validate an inbound command and convert it to a command word.
 *
 * @param[in]  p_in    Payload bytes.
 * @param[in]  length  Payload length.
 * @param[out] p_word  Command word for mbf_ctrl_cmd (valid commands only).
 *
 * @return The command type, or TLM_CMD_INVALID for a malformed payload
 *         (wrong length, frame type, version, command, letter or action).
 */
tlm_codec_cmd_t
tlm_codec_parse_command (uint8_t const * p_in, uint32_t length,
                         uint32_t * p_word)
{
    tlm_codec_cmd_t result = TLM_CMD_INVALID;

    if ((length >= COMMAND_BYTES) &&
        (FRAME_COMMAND == bitpack_word_get(p_in[0], 0u, TYPE_WIDTH)) &&
        (TLM_CODEC_VERSION == bitpack_word_get(p_in[0], TYPE_WIDTH,
                                               VERSION_WIDTH)))
    {
        uint32_t const type    = bitpack_word_get(p_in[1], 0u,
                                                  TLM_CMD_TYPE_WIDTH);
        uint32_t const version = bitpack_word_get(p_in[1],
                                                  TLM_CMD_TYPE_WIDTH,
                                                  TLM_CMD_VERSION_WIDTH);
        uint32_t const detail  = (MAP_SET_BYTES == length) ? p_in[2] : 0u;
        uint32_t const letter  = bitpack_word_get(detail, 0u,
                                                  TLM_CMD_LETTER_WIDTH);
        uint32_t const action  = bitpack_word_get(detail,
                                                  TLM_CMD_LETTER_WIDTH,
                                                  TLM_CMD_ACTION_WIDTH);
        uint32_t       word    = 0u;

        word = bitpack_word_set(word, TLM_CMD_TYPE_SHIFT, TLM_CMD_TYPE_WIDTH,
                                type);
        word = bitpack_word_set(word, TLM_CMD_LETTER_SHIFT,
                                TLM_CMD_LETTER_WIDTH, letter);
        word = bitpack_word_set(word, TLM_CMD_ACTION_SHIFT,
                                TLM_CMD_ACTION_WIDTH, action);
        word = bitpack_word_set(word, TLM_CMD_VERSION_SHIFT,
                                TLM_CMD_VERSION_WIDTH, version);
        if ((TLM_CMD_TUNE == type) && (TUNE_BYTES == length))
        {
            // Signed 16-bit value to excess-32768: flipping the sign bit
            // maps -32768..32767 onto 0..65535.  Bytes are 8-bit values.
            uint32_t const raw = ((uint32_t) p_in[2]) |
                                 (((uint32_t) p_in[3]) << 8u);

            word = bitpack_word_set(0u, TLM_CMD_TYPE_SHIFT,
                                    TLM_CMD_TYPE_WIDTH, type);
            word = bitpack_word_set(word, TLM_CMD_PARAM_SHIFT,
                                    TLM_CMD_PARAM_WIDTH, version);
            word = bitpack_word_set(word, TLM_CMD_VALUE_SHIFT,
                                    TLM_CMD_VALUE_WIDTH,
                                    raw ^ VALUE_SIGN_BIT);
        }

        result  = command_type(type, letter, length);
        *p_word = word;
    }

    return (result);
}


/*!
 * @brief Check a command's length and fields against its type.
 *
 * @param[in] type    Command type field.
 * @param[in] letter  Letter field (MAP_SET only).
 * @param[in] length  Payload length.
 *
 * @return The command type if consistent, otherwise TLM_CMD_INVALID.
 */
static tlm_codec_cmd_t
command_type (uint32_t type, uint32_t letter, uint32_t length)
{
    tlm_codec_cmd_t result = TLM_CMD_INVALID;

    switch (type)
    {
        case TLM_CMD_START:
            result = (COMMAND_BYTES == length) ? TLM_CMD_START :
                                                 TLM_CMD_INVALID;
        break;

        case TLM_CMD_STOP:
            result = (COMMAND_BYTES == length) ? TLM_CMD_STOP :
                                                 TLM_CMD_INVALID;
        break;

        case TLM_CMD_RESET:
            result = (COMMAND_BYTES == length) ? TLM_CMD_RESET :
                                                 TLM_CMD_INVALID;
        break;

        case TLM_CMD_SUMMARY:
            result = (COMMAND_BYTES == length) ? TLM_CMD_SUMMARY :
                                                 TLM_CMD_INVALID;
        break;

        case TLM_CMD_MAP_SET:
            result = ((MAP_SET_BYTES == length) && (letter < LETTER_COUNT)) ?
                     TLM_CMD_MAP_SET : TLM_CMD_INVALID;
        break;

        case TLM_CMD_TUNE:
            result = (TUNE_BYTES == length) ? TLM_CMD_TUNE :
                                              TLM_CMD_INVALID;
        break;

        default:
            result = TLM_CMD_INVALID;
        break;
    }

    return (result);
}


/*!
 * @brief Clear the buffer and write the header byte.
 *
 * @param[out] p_pack    Stream cursor.
 * @param[out] p_out     Output buffer.
 * @param[in]  capacity  Output buffer size.
 * @param[in]  type      Frame type.
 */
static void
start_frame (bitpack_t * p_pack, uint8_t * p_out, uint32_t capacity,
             uint32_t type)
{
    bitpack_init(p_pack, p_out, capacity);
    bitpack_put(p_pack, type, TYPE_WIDTH);
    bitpack_put(p_pack, TLM_CODEC_VERSION, VERSION_WIDTH);
}


/*!
 * @brief Length of a finished frame.
 *
 * @param[in] p_pack  Stream cursor.
 *
 * @return Bytes used, or 0 if any field did not fit.
 */
static uint32_t
finish_frame (bitpack_t const * p_pack)
{
    uint32_t length = 0u;

    if (!bitpack_is_overflow(p_pack))
    {
        length = bitpack_bytes_used(p_pack);
    }

    return (length);
}

/*** end of file ***/


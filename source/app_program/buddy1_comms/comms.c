/** @file comms.c
 *
 * @brief Communication, kernel side: outgoing queue, command routing and
 *        link status.
 *
 * @par
 * Functions called from the radio-owner task (comms_take, comms_on_command,
 * comms_on_link) are the only writers of the link state and of the
 * SYS_STATUS_COMMS word.  The drop counter has two writers (Telemetry and
 * the radio task) and is updated with interrupts disabled.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_ipc.h"
#include "sys_status.h"
#include "bitpack.h"
#include "tlm_codec.h"
#include "telemetry.h"
#include "tune.h"
#include "comms_net.h"
#include "comms.h"

#ifndef TM_WIFI_MQTT
#define TM_WIFI_MQTT            0
#endif

#define BYTE_MASK               0xFFu
#define BITS_PER_BYTE           8u
#define TOPIC_BYTES             1u

static uint32_t volatile g_dropped    = 0u;     // Telemetry and radio task.
static uint32_t          g_reconnects = 0u;     // Radio task only.
static comms_link_t      g_link       = COMMS_LINK_WIFI_INIT;

static void publish_status(void);
static bool route_command(tlm_codec_cmd_t command, uint32_t word);
static bool apply_tune(uint32_t word);


/*!
 * @brief Start the network side (when built with MQTT=1).
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
comms_start (void)
{
    int32_t result = 0;

    if (0 != TM_WIFI_MQTT)
    {
        result = comms_net_start();
    }
    else
    {
        g_link = COMMS_LINK_DISABLED;
    }

    publish_status();

    return (result);
}


/*!
 * @brief Queue a payload for publishing, without waiting.
 *
 * @param[in] topic      Destination topic.
 * @param[in] p_payload  Payload bytes.
 * @param[in] length     Payload length, at most TLM_CODEC_MAX_FRAME.
 *
 * @return true if queued; false if dropped (queue full, too long, or no
 *         network in this build).
 */
bool
comms_queue (tlm_codec_topic_t topic, uint8_t const * p_payload,
             uint32_t length)
{
    bool b_queued = false;

    if ((0 != TM_WIFI_MQTT) && (length > 0u) &&
        (length <= TLM_CODEC_MAX_FRAME))
    {
        uint8_t msg[ROBOT_IPC_MQTT_MSG_BYTES];

        // The topic enumeration is below 8, so the cast is exact.
        msg[0] = (uint8_t) topic;

        for (uint32_t idx = 0u; idx < length; idx++)
        {
            msg[idx + TOPIC_BYTES] = p_payload[idx];
        }

        b_queued = robot_ipc_send(ROBOT_IPC_MBF_MQTT_TX, msg,
                                  length + TOPIC_BYTES, ROBOT_IPC_POLL);
    }

    if (!b_queued)
    {
        comms_count_drop();
    }

    return (b_queued);
}


/*!
 * @brief Take the next queued payload (radio-owner task only).
 *
 * @param[out] p_topic    Destination topic.
 * @param[out] p_payload  Payload bytes.
 * @param[in]  capacity   Size of p_payload, at least TLM_CODEC_MAX_FRAME.
 *
 * @return Payload length, or 0 if the queue is empty.
 */
uint32_t
comms_take (uint32_t * p_topic, uint8_t * p_payload, uint32_t capacity)
{
    uint8_t        msg[ROBOT_IPC_MQTT_MSG_BYTES];
    uint32_t const length  = robot_ipc_receive(ROBOT_IPC_MBF_MQTT_TX, msg,
                                               sizeof(msg), ROBOT_IPC_POLL);
    uint32_t       payload = 0u;

    if ((length > TOPIC_BYTES) && ((length - TOPIC_BYTES) <= capacity))
    {
        payload  = length - TOPIC_BYTES;
        *p_topic = msg[0];

        for (uint32_t idx = 0u; idx < payload; idx++)
        {
            p_payload[idx] = msg[idx + TOPIC_BYTES];
        }
    }

    publish_status();

    return (payload);
}


/*!
 * @brief Validate and route an inbound command (radio-owner task only).
 *
 * @param[in]  p_payload     Command payload.
 * @param[in]  length        Payload length.
 * @param[out] p_ack         ACK frame to publish (TLM_CODEC_MAX_FRAME).
 * @param[out] p_ack_length  ACK frame length.
 *
 * @return true if the command was valid and handed to its task.
 *
 * @par
 * The ACK reports receipt and validation only.  Vehicle Control reports
 * when a map update has actually been applied (MAP_ACK event), so
 * application is acknowledged separately from receipt (Version A, Buddy 1).
 */
bool
comms_on_command (uint8_t const * p_payload, uint32_t length,
                  uint8_t * p_ack, uint32_t * p_ack_length)
{
    uint32_t              word     = 0u;
    tlm_codec_cmd_t const command  = tlm_codec_parse_command(p_payload,
                                                             length, &word);
    bool const            b_routed = (TLM_CMD_INVALID != command) &&
                                     (route_command(command, word));

    *p_ack_length = tlm_codec_ack(bitpack_word_get(word, TLM_CMD_TYPE_SHIFT,
                                                   TLM_CMD_TYPE_WIDTH),
                                  b_routed, p_ack, TLM_CODEC_MAX_FRAME);

    return (b_routed);
}


/*!
 * @brief Record a link state change (radio-owner task only).
 *
 * @param[in] link           New link state.
 * @param[in] b_reconnected  true if this is a recovery after a loss.
 *
 * @par
 * Link loss is reported as a fault for telemetry only; Vehicle Control
 * never changes state because of the network (Week 6, NFR1 and NFR17).
 */
void
comms_on_link (comms_link_t link, bool b_reconnected)
{
    comms_link_t const previous = g_link;

    g_link = link;

    if (b_reconnected)
    {
        g_reconnects++;
    }

    if (COMMS_LINK_ONLINE == link)
    {
        robot_ipc_flag_clear(ROBOT_IPC_FLAG_FAULT,
                             ROBOT_FAULT_WIFI_LOSS | ROBOT_FAULT_MQTT_LOSS);
        robot_ipc_flag_set(ROBOT_IPC_FLAG_COMM, ROBOT_COMM_LINK_UP);
    }
    else if ((COMMS_LINK_ONLINE == previous) &&
             (COMMS_LINK_LOST == link))
    {
        robot_ipc_raise_fault(ROBOT_FAULT_MQTT_LOSS);
        robot_ipc_flag_set(ROBOT_IPC_FLAG_COMM, ROBOT_COMM_LINK_DOWN);
    }
    else if ((COMMS_LINK_WIFI_CONNECTING == link) &&
             (COMMS_LINK_WIFI_INIT != previous))
    {
        robot_ipc_raise_fault(ROBOT_FAULT_WIFI_LOSS);
    }
    else
    {
        // Intermediate connection step: nothing to report.
    }

    publish_status();
}


/*!
 * @brief Count a message dropped anywhere on the outgoing path.
 */
void
comms_count_drop (void)
{
    uint32_t intsts = 0u;

    DI(intsts);
    g_dropped++;
    EI(intsts);
}


/*!
 * @brief Publish the SYS_STATUS_COMMS word.
 */
static void
publish_status (void)
{
    uint32_t word = 0u;

    // The enumeration value is below 8, so the cast is exact.
    word = bitpack_word_set(word, COMMS_LINK_SHIFT, COMMS_LINK_WIDTH,
                            (uint32_t) g_link);
    word = bitpack_word_set(word, COMMS_RECONNECT_SHIFT, COMMS_COUNT_WIDTH,
                            bitpack_saturate(g_reconnects,
                                             COMMS_COUNT_WIDTH));
    word = bitpack_word_set(word, COMMS_DROPPED_SHIFT, COMMS_COUNT_WIDTH,
                            bitpack_saturate(g_dropped, COMMS_COUNT_WIDTH));
    sys_status_write(SYS_STATUS_COMMS, word);
}


/*!
 * @brief Hand a valid command to the task that owns it.
 *
 * @param[in] command  Command type.
 * @param[in] word     Packed command word.
 *
 * @return true if delivered.
 */
static bool
route_command (tlm_codec_cmd_t command, uint32_t word)
{
    bool b_routed = true;

    if (TLM_CMD_TUNE == command)
    {
        b_routed = apply_tune(word);
    }
    else if (TLM_CMD_SUMMARY == command)
    {
        // Summary requests go to Telemetry (flg_comm).
        robot_ipc_flag_set(ROBOT_IPC_FLAG_COMM, ROBOT_COMM_SUMMARY_REQ);
    }
    else
    {
        uint8_t msg[ROBOT_IPC_CTRL_MSG_BYTES];

        for (uint32_t idx = 0u; idx < ROBOT_IPC_CTRL_MSG_BYTES; idx++)
        {
            // Little-endian; each byte is masked to 8 bits.
            msg[idx] = (uint8_t) ((word >> (idx * BITS_PER_BYTE)) &
                                  BYTE_MASK);
        }

        // START, STOP, RESET and MAP_SET go to Vehicle Control.
        b_routed = robot_ipc_send(ROBOT_IPC_MBF_CTRL_CMD, msg, sizeof(msg),
                                  ROBOT_IPC_POLL);

        if (b_routed)
        {
            robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_CTRL_CMD);
        }
    }

    return (b_routed);
}


/*!
 * @brief Apply a TUNE command and report it as a TUNE event.
 *
 * @param[in] word  TUNE command word (tlm_codec.h layout).
 *
 * @return true if the setting accepted the value.
 *
 * @par
 * Runs in the radio-owner task, the only writer of the tunable settings.
 */
static bool
apply_tune (uint32_t word)
{
    uint32_t const param = bitpack_word_get(word, TLM_CMD_PARAM_SHIFT,
                                            TLM_CMD_PARAM_WIDTH);
    uint32_t const field = bitpack_word_get(word, TLM_CMD_VALUE_SHIFT,
                                            TLM_CMD_VALUE_WIDTH);
    bool const     b_ok  = tune_set(param,
                                    bitpack_to_signed(field,
                                                      TLM_CMD_VALUE_WIDTH));

    if (b_ok)
    {
        // Event data: setting id (4 bits), then the value field (16 bits).
        telemetry_post_event(TELEMETRY_EVT_TUNE,
                             param | (field << TLM_CMD_PARAM_WIDTH));
    }

    return (b_ok);
}

/*** end of file ***/


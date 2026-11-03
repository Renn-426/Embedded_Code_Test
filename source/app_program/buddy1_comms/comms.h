/** @file comms.h
 *
 * @brief Communication, kernel side (Buddy 1): the outgoing MQTT queue
 *        (mbf_mqtt_tx), inbound command routing and link status.
 *
 * @par
 * The network itself (WiFi rejoin, MQTT client, publishing) lives in
 * comms_net.c and runs inside the port's CYW43 radio-owner task, which is
 * the only task allowed to touch lwIP (Week 6: "Communication Task = the
 * port's radio task, extended").  The two halves talk only through the
 * plain-C functions below.  No control task ever waits on the network: the
 * queue is filled with zero timeout and a full queue drops the message.
 *
 * @par
 * Status word SYS_STATUS_COMMS (LSB first): [2:0] link state,
 * [10:3] reconnections (saturating), [18:11] dropped messages (saturating).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>
#include <stdbool.h>

#include "tlm_codec.h"

#define COMMS_LINK_SHIFT            0u
#define COMMS_LINK_WIDTH            3u
#define COMMS_RECONNECT_SHIFT       3u
#define COMMS_DROPPED_SHIFT         11u
#define COMMS_COUNT_WIDTH           8u

/*!
 * @brief Link states (3 bits), Week 6 Section 6.1.
 */
typedef enum
{
    COMMS_LINK_WIFI_INIT = 0,
    COMMS_LINK_WIFI_CONNECTING,
    COMMS_LINK_MQTT_CONNECTING,
    COMMS_LINK_ONLINE,
    COMMS_LINK_LOST,
    COMMS_LINK_RADIO_FAULT,
    COMMS_LINK_DISABLED
} comms_link_t;

int32_t  comms_start(void);
bool     comms_queue(tlm_codec_topic_t topic, uint8_t const * p_payload,
                     uint32_t length);
uint32_t comms_take(uint32_t * p_topic, uint8_t * p_payload,
                    uint32_t capacity);
bool     comms_on_command(uint8_t const * p_payload, uint32_t length,
                          uint8_t * p_ack, uint32_t * p_ack_length);
void     comms_on_link(comms_link_t link, bool b_reconnected);
void     comms_count_drop(void);

#endif /* COMMS_H */

/*** end of file ***/


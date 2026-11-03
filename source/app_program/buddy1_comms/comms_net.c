/** @file comms_net.c
 *
 * @brief Communication, network side: WiFi recovery and the lwIP MQTT
 *        client, run inside the CYW43 radio-owner task.
 *
 * @par
 * lwIP runs with NO_SYS=1 in the radio-owner task, so every lwIP and cyw43
 * call, including the MQTT callbacks, happens in that one task.  The poll
 * function is registered with cyw43_utk_set_app_poll() and runs once per
 * radio poll.  Reference: lwIP 2.2 apps/mqtt (lwip/apps/mqtt.h).
 *
 * @par
 * Built only with MQTT=1 (TM_WIFI_MQTT); otherwise comms_net_start()
 * reports that the network is not available.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "robot_cfg.h"
#include "comms.h"
#include "comms_net.h"

#ifndef TM_WIFI_MQTT
#define TM_WIFI_MQTT            0
#endif

#if TM_WIFI_MQTT

#include <string.h>

#include "cyw43.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "cyw43_utk.h"
#include "wifi_credentials.h"

#define MQTT_CONNECT_TIMEOUT_MS 5000u
#define WIFI_REJOIN_MS          15000u
#define BACKOFF_FIRST_MS        1000u
#define BACKOFF_LAST_MS         4000u
#define SENDS_PER_POLL          4u
#define COMMAND_MAX_BYTES       8u
#define LWT_OFFLINE_FRAME       0x15
#define QOS_AT_MOST_ONCE        0u
#define NO_RETAIN               0u

/*!
 * @brief Network states (Week 6 Section 6.1).
 */
typedef enum
{
    NET_WAIT_WIFI = 0,
    NET_MQTT_CONNECTING,
    NET_ONLINE,
    NET_BACKOFF
} net_state_t;

/*!
 * @brief One outgoing message held while offline or while lwIP is busy.
 */
typedef struct
{
    uint8_t     data[TLM_CODEC_MAX_FRAME];
    uint32_t    length;
} held_msg_t;

static char const * const gp_topics[TLM_TOPIC_COUNT] =
{
    MQTT_TOPIC_ROOT "tlm",
    MQTT_TOPIC_ROOT "hb",
    MQTT_TOPIC_ROOT "evt",
    MQTT_TOPIC_ROOT "sum",
    MQTT_TOPIC_ROOT "ack"
};
static char const         g_command_topic[] = MQTT_TOPIC_ROOT "cmd";
static char const         g_will_msg[]      = { LWT_OFFLINE_FRAME, 0 };
static uint8_t const      g_ssid[]          = UTK_WIFI_SSID;
static uint8_t const      g_password[]      = UTK_WIFI_PASSWORD;

// All of the state below is used only inside the radio-owner task.
static mqtt_client_t *    gp_client         = NULL;
static net_state_t        g_state           = NET_WAIT_WIFI;
static uint32_t           g_state_ms        = 0u;
static uint32_t           g_backoff_ms      = BACKOFF_FIRST_MS;
static uint32_t           g_join_ms         = 0u;
static bool               gb_connected      = false;
static bool               gb_refused        = false;
static bool               gb_was_online     = false;
static held_msg_t         g_held[TLM_TOPIC_COUNT];
static uint8_t            g_rx_buf[COMMAND_MAX_BYTES];
static uint32_t           g_rx_length       = 0u;
static bool               gb_rx_wanted      = false;

static void comms_net_poll(void);
static bool link_is_up(void);
static void try_rejoin(uint32_t now_ms);
static void start_mqtt(uint32_t now_ms);
static void enter_state(net_state_t state, uint32_t now_ms);
static void lose_link(uint32_t now_ms);
static void pump_queue(void);
static bool publish_held(uint32_t topic);
static void on_connection(mqtt_client_t * p_client, void * p_arg,
                          mqtt_connection_status_t status);
static void on_publish(void * p_arg, char const * p_topic,
                       uint32_t total_length);
static void on_data(void * p_arg, uint8_t const * p_data, uint16_t length,
                    uint8_t flags);


/*!
 * @brief Register the network poll with the radio-owner task.
 *
 * @return 0 (the MQTT client itself is created inside the radio task,
 *         which is the only task allowed to use the lwIP heap).
 */
int32_t
comms_net_start (void)
{
    for (uint32_t topic = 0u; topic < TLM_TOPIC_COUNT; topic++)
    {
        g_held[topic].length = 0u;
    }

    cyw43_utk_set_app_poll(comms_net_poll);

    return (0);
}


/*!
 * @brief Network statechart, run once per radio poll.
 */
static void
comms_net_poll (void)
{
    uint32_t const now_ms = sys_now();
    bool const     b_link = link_is_up();

    if (NULL == gp_client)
    {
        gp_client = mqtt_client_new();
    }

    switch (g_state)
    {
        case NET_WAIT_WIFI:
            if (b_link)
            {
                start_mqtt(now_ms);
            }
            else
            {
                try_rejoin(now_ms);
            }
        break;

        case NET_MQTT_CONNECTING:
            if (gb_connected)
            {
                enter_state(NET_ONLINE, now_ms);
                comms_on_link(COMMS_LINK_ONLINE, gb_was_online);
                gb_was_online = true;
                g_backoff_ms  = BACKOFF_FIRST_MS;
            }
            else if ((!b_link) || gb_refused ||
                     ((now_ms - g_state_ms) > MQTT_CONNECT_TIMEOUT_MS))
            {
                lose_link(now_ms);
            }
            else
            {
                // Waiting for the broker's CONNACK.
            }
        break;

        case NET_ONLINE:
            if ((!b_link) || (0u == mqtt_client_is_connected(gp_client)))
            {
                lose_link(now_ms);
            }
        break;

        default:
            if ((now_ms - g_state_ms) >= g_backoff_ms)
            {
                g_backoff_ms = (g_backoff_ms < BACKOFF_LAST_MS) ?
                               (g_backoff_ms * 2u) : BACKOFF_LAST_MS;
                enter_state(NET_WAIT_WIFI, now_ms);
                comms_on_link(COMMS_LINK_WIFI_CONNECTING, false);
            }
        break;
    }

    pump_queue();
}


/*!
 * @brief Report whether the station link is up with an IPv4 address.
 *
 * @return true when MQTT can be attempted.
 */
static bool
link_is_up (void)
{
    struct netif * const p_netif = &cyw43_state.netif[CYW43_ITF_STA];

    return ((0u != netif_is_link_up(p_netif)) &&
            (!ip4_addr_isany_val(*netif_ip4_addr(p_netif))));
}


/*!
 * @brief Re-join the access point if the link has been down for a while.
 *
 * @param[in] now_ms  Current time.
 *
 * @par
 * The port's radio task makes the first join; this only repeats it after a
 * loss, at most every WIFI_REJOIN_MS, and never while a join is running.
 */
static void
try_rejoin (uint32_t now_ms)
{
    int32_t const status = cyw43_wifi_link_status(&cyw43_state,
                                                  CYW43_ITF_STA);

    if (((now_ms - g_join_ms) > WIFI_REJOIN_MS) &&
        (CYW43_LINK_JOIN != status) && (CYW43_LINK_NOIP != status))
    {
        g_join_ms = now_ms;

        // (void): the join outcome is observed through the link status.
        (void) cyw43_wifi_join(&cyw43_state, sizeof(g_ssid) - 1u, g_ssid,
                               sizeof(g_password) - 1u, g_password,
                               UTK_WIFI_AUTH, NULL, CYW43_CHANNEL_NONE);
    }
}


/*!
 * @brief Open the broker connection.
 *
 * @param[in] now_ms  Current time.
 */
static void
start_mqtt (uint32_t now_ms)
{
    ip_addr_t broker;
    struct mqtt_connect_client_info_t info;

    (void) memset(&info, 0, sizeof(info));  // Returns its argument.
    info.client_id    = MQTT_CLIENT_ID;
    info.keep_alive   = MQTT_KEEPALIVE_S;
    info.will_topic   = gp_topics[TLM_TOPIC_HEARTBEAT];
    info.will_msg     = g_will_msg;
    info.will_msg_len = 1u;
    gb_connected      = false;
    gb_refused        = false;

    if ((NULL != gp_client) &&
        (0 != ipaddr_aton(MQTT_BROKER_IP, &broker)) &&
        (ERR_OK == mqtt_client_connect(gp_client, &broker, MQTT_BROKER_PORT,
                                       on_connection, NULL, &info)))
    {
        enter_state(NET_MQTT_CONNECTING, now_ms);
        comms_on_link(COMMS_LINK_MQTT_CONNECTING, false);
    }
    else
    {
        lose_link(now_ms);
    }
}


/*!
 * @brief Change state and remember when.
 *
 * @param[in] state   New state.
 * @param[in] now_ms  Current time.
 */
static void
enter_state (net_state_t state, uint32_t now_ms)
{
    g_state    = state;
    g_state_ms = now_ms;
}


/*!
 * @brief LINK LOST: drop the session and back off.
 *
 * @param[in] now_ms  Current time.
 */
static void
lose_link (uint32_t now_ms)
{
    // Also needed for an attempt still connecting, or the next connect fails
    // with ERR_ISCONN.  lwIP ignores this call on an idle client.
    if (NULL != gp_client)
    {
        mqtt_disconnect(gp_client);
    }

    gb_connected = false;
    enter_state(NET_BACKOFF, now_ms);
    comms_on_link(COMMS_LINK_LOST, false);
}


/*!
 * @brief Move queued messages to the broker, or hold the newest of each
 *        topic while offline.
 */
static void
pump_queue (void)
{
    bool     b_online = (NET_ONLINE == g_state);
    uint32_t sends    = 0u;

    // Messages held from an earlier poll go first, in topic order.
    for (uint32_t topic = 0u; topic < TLM_TOPIC_COUNT; topic++)
    {
        if (b_online && (0u != g_held[topic].length))
        {
            b_online = publish_held(topic);
        }
    }

    while (b_online && (sends < SENDS_PER_POLL))
    {
        uint32_t topic  = 0u;
        uint8_t  data[TLM_CODEC_MAX_FRAME];
        uint32_t length = comms_take(&topic, data, sizeof(data));

        if ((0u == length) || (topic >= TLM_TOPIC_COUNT))
        {
            b_online = false;
        }
        else
        {
            (void) memcpy(g_held[topic].data, data, length);  // Returns dst.
            g_held[topic].length = length;
            b_online             = publish_held(topic);
            sends++;
        }
    }

    // Offline: keep only the newest message of each topic.
    while (NET_ONLINE != g_state)
    {
        uint32_t topic  = 0u;
        uint8_t  data[TLM_CODEC_MAX_FRAME];
        uint32_t length = comms_take(&topic, data, sizeof(data));

        if ((0u == length) || (topic >= TLM_TOPIC_COUNT))
        {
            break;
        }

        if (0u != g_held[topic].length)
        {
            comms_count_drop();
        }

        (void) memcpy(g_held[topic].data, data, length);  // Returns dst.
        g_held[topic].length = length;
    }
}


/*!
 * @brief Publish the held message of one topic.
 *
 * @param[in] topic  Topic index.
 *
 * @return true if lwIP accepted it (or it was dropped as unsendable);
 *         false if lwIP is out of buffer space and it must be retried.
 */
static bool
publish_held (uint32_t topic)
{
    // The held length is at most TLM_CODEC_MAX_FRAME, so it fits u16_t.
    err_t const result = mqtt_publish(gp_client, gp_topics[topic],
                                      g_held[topic].data,
                                      (uint16_t) g_held[topic].length,
                                      QOS_AT_MOST_ONCE, NO_RETAIN, NULL,
                                      NULL);
    bool        b_done = true;

    if (ERR_MEM == result)
    {
        b_done = false;
    }
    else
    {
        if (ERR_OK != result)
        {
            comms_count_drop();
        }

        g_held[topic].length = 0u;
    }

    return (b_done);
}


/*!
 * @brief MQTT connection callback (radio-owner task).
 *
 * @param[in] p_client  Client.
 * @param[in] p_arg     Unused argument.
 * @param[in] status    Connection outcome.
 */
static void
on_connection (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    (void) p_arg;  // Unused: one client only.

    gb_connected = (MQTT_CONNECT_ACCEPTED == status);
    gb_refused   = !gb_connected;

    if (gb_connected)
    {
        mqtt_set_inpub_callback(p_client, on_publish, on_data, NULL);

        // (void): a failed subscription only means no commands arrive; the
        // robot runs without them (NFR1).
        (void) mqtt_subscribe(p_client, g_command_topic, QOS_AT_MOST_ONCE,
                              NULL, NULL);
    }
}


/*!
 * @brief Start of an inbound publish (radio-owner task).
 *
 * @param[in] p_arg         Unused argument.
 * @param[in] p_topic       Topic name.
 * @param[in] total_length  Payload length.
 */
static void
on_publish (void * p_arg, char const * p_topic, uint32_t total_length)
{
    (void) p_arg;  // Unused: one client only.

    g_rx_length  = 0u;
    gb_rx_wanted = (0 == strcmp(p_topic, g_command_topic)) &&
                   (total_length <= COMMAND_MAX_BYTES);
}


/*!
 * @brief Payload data of an inbound publish (radio-owner task).
 *
 * @param[in] p_arg   Unused argument.
 * @param[in] p_data  Payload fragment.
 * @param[in] length  Fragment length.
 * @param[in] flags   MQTT_DATA_FLAG_LAST on the final fragment.
 */
static void
on_data (void * p_arg, uint8_t const * p_data, uint16_t length,
         uint8_t flags)
{
    (void) p_arg;  // Unused: one client only.

    for (uint32_t idx = 0u; gb_rx_wanted && (idx < length); idx++)
    {
        if (g_rx_length < COMMAND_MAX_BYTES)
        {
            g_rx_buf[g_rx_length] = p_data[idx];
            g_rx_length++;
        }
    }

    if (gb_rx_wanted && (0u != (flags & MQTT_DATA_FLAG_LAST)))
    {
        uint8_t  ack[TLM_CODEC_MAX_FRAME];
        uint32_t ack_length = 0u;

        gb_rx_wanted = false;

        // (void): the outcome is reported to the sender in the ACK frame.
        (void) comms_on_command(g_rx_buf, g_rx_length, ack, &ack_length);

        if (ack_length > 0u)
        {
            (void) comms_queue(TLM_TOPIC_ACK, ack, ack_length);  // Counted.
        }
    }
}

#else   /* TM_WIFI_MQTT */

/*!
 * @brief Report that this build has no network.
 *
 * @return -1.
 */
int32_t
comms_net_start (void)
{
    return (-1);
}

#endif  /* TM_WIFI_MQTT */

/*** end of file ***/


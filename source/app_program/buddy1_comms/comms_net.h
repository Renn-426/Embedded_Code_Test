/** @file comms_net.h
 *
 * @brief Communication, network side (Buddy 1): WiFi recovery and the MQTT
 *        client, run inside the CYW43 radio-owner task.
 *
 * @par
 * Statechart (Week 6 Section 6.1): WIFI CONNECTING, MQTT CONNECTING,
 * ONLINE and LINK LOST with 1, 2 then 4 s back-off.  While offline only the
 * newest message of each topic is kept; older ones are dropped and counted.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef COMMS_NET_H
#define COMMS_NET_H

#include <stdint.h>

int32_t comms_net_start(void);

#endif /* COMMS_NET_H */

/*** end of file ***/


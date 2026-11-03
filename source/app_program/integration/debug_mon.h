/** @file debug_mon.h
 *
 * @brief USB console monitor for the full mission (no MQTT needed).
 *
 * @par
 * Built only with DEBUG_MON=1.  A lowest-priority task reads the packed
 * status words every subsystem already publishes and prints them on the USB
 * console: one status line per second, plus a line the moment the vehicle
 * state, the faults, the last barcode, the hump count or the obstacle state
 * changes.  It only reads; it never takes a message or changes behaviour.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef DEBUG_MON_H
#define DEBUG_MON_H

#include <stdint.h>

int32_t debug_mon_start(void);

#endif /* DEBUG_MON_H */

/*** end of file ***/


/** @file sys_status.h
 *
 * @brief Latest-value status board shared by every subsystem (Week 6
 *        "sys_status"), stored as minimum-bit packed 32-bit words.
 *
 * @par
 * Each word has exactly one writer task.  A writer builds the complete word
 * in a local variable (bitpack_word_set) and publishes it with
 * sys_status_write(), which is one aligned 32-bit store and therefore atomic
 * on the Cortex-M0+.  Readers (Telemetry, Vehicle Control) take a snapshot
 * with sys_status_read().  No read-modify-write ever touches a shared word,
 * which is the race-free access rule required by Change Requirement 1.
 *
 * @par
 * The field layout of each word is defined in its owner's header and in
 * docs/MQTT_PROTOCOL.md; the used width of each word is returned by
 * sys_status_width() so Telemetry sends only the meaningful bits.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef SYS_STATUS_H
#define SYS_STATUS_H

#include <stdint.h>

/*!
 * @brief Status words, in telemetry frame order.  Owner in brackets.
 */
typedef enum
{
    SYS_STATUS_VEHICLE = 0,     // [Vehicle Control] vehicle.h
    SYS_STATUS_MOTION,          // [Buddy 2] motion.h
    SYS_STATUS_ENC_COUNTS,      // [Buddy 2] encoder.h
    SYS_STATUS_ODOMETRY,        // [Buddy 2] encoder.h
    SYS_STATUS_LINE,            // [Buddy 3] line_follow.h
    SYS_STATUS_BARCODE,         // [Buddy 3] barcode.h
    SYS_STATUS_IMU,             // [Buddy 4] imu.h
    SYS_STATUS_HUMP,            // [Buddy 4] imu.h
    SYS_STATUS_OBSTACLE,        // [Buddy 5] obstacle.h
    SYS_STATUS_COMMS,           // [Buddy 1] comms.h
    SYS_STATUS_HEALTH,          // [Buddy 1] telemetry.h
    SYS_STATUS_WORD_COUNT
} sys_status_word_t;

void     sys_status_write(sys_status_word_t word, uint32_t value);
uint32_t sys_status_read(sys_status_word_t word);
uint32_t sys_status_width(sys_status_word_t word);
uint32_t sys_status_total_bits(void);

#endif /* SYS_STATUS_H */

/*** end of file ***/


/** @file ultrasonic.h
 *
 * @brief HC-SR04 ranging: TRIG pulse, ECHO timed by the GPIO interrupt and
 *        handed to the task through sem_echo (Buddy 5).
 *
 * @par
 * WARNING: powered from 5 V the HC-SR04 drives ECHO to 5 V.  The RP2040
 * pins are 3.3 V only, so ECHO must pass through a divider (for example
 * 1 kohm series, 2 kohm to ground) before GP4.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

/*!
 * @brief Outcome of one measurement (Week 6: valid, unknown and fault are
 *        distinct; a missing return is not proof of a clear path).
 */
typedef enum
{
    ULTRASONIC_OK = 0,      // Distance measured.
    ULTRASONIC_NO_ECHO,     // ECHO started but no return in range.
    ULTRASONIC_FAULT        // ECHO never started: sensor or wiring fault.
} ultrasonic_result_t;

int32_t             ultrasonic_init(void);
ultrasonic_result_t ultrasonic_measure(uint32_t * p_distance_mm);

#endif /* ULTRASONIC_H */

/*** end of file ***/


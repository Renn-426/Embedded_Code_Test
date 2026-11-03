/** @file servo_hw.h
 *
 * @brief Scanning servo on GP15 (RP2040 PWM slice 7, channel B).
 *
 * @par
 * Owner: Buddy 5.  Only the Obstacle Scanning task moves the servo.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef SERVO_HW_H
#define SERVO_HW_H

#include <stdint.h>

#define SERVO_HW_MAX_DEG            180u

void servo_hw_init(void);
void servo_hw_set_angle(uint32_t angle_deg);

#endif /* SERVO_HW_H */

/*** end of file ***/


/** @file motor_hw.h
 *
 * @brief Robo Pico dual DC-motor driver (GP8-GP11) on RP2040 PWM.
 *
 * @par
 * Owner: Buddy 2 (Motion Control).  Only the Motion Control task may call
 * these functions (Week 6: "only the Motion Control Task writes the motor
 * pins").
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include <stdint.h>

// Full-scale drive command.
#define MOTOR_HW_FULL_PERMILLE      1000

void motor_hw_init(void);
void motor_hw_set(int32_t left_permille, int32_t right_permille);
void motor_hw_brake(void);

#endif /* MOTOR_HW_H */

/*** end of file ***/


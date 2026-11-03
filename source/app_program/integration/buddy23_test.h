/** @file buddy23_test.h
 *
 * @brief Buddy 2 + Buddy 3 bench test: follow a line under button control.
 *
 * @par
 * Built only with BUDDY_TEST=23.  The motion, encoder, line and barcode tasks
 * run with this supervisor task instead of Vehicle Control: the START button
 * (GP20) starts and stops line following, and the robot stops by itself on
 * line loss, a junction or a motor stall.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BUDDY23_TEST_H
#define BUDDY23_TEST_H

#include <stdint.h>

int32_t buddy23_test_start(void);

#endif /* BUDDY23_TEST_H */

/*** end of file ***/


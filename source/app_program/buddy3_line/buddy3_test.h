/** @file buddy3_test.h
 *
 * @brief Buddy 3 stand-alone bench test: IR sensors, line state, barcodes.
 *
 * @par
 * Built only with BUDDY_TEST=3.  The robot then starts the Buddy 3 tasks and
 * this test task, nothing else: the motors stay off, and the IR readings,
 * line states and decoded barcodes are printed on the USB console.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BUDDY3_TEST_H
#define BUDDY3_TEST_H

#include <stdint.h>
#include <stdbool.h>

int32_t buddy3_test_start(void);
bool    buddy3_test_calibrate(void);
void    buddy3_test_report_events(void);

#endif /* BUDDY3_TEST_H */

/*** end of file ***/


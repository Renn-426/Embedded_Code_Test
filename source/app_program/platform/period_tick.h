/** @file period_tick.h
 *
 * @brief Drift-free release of the periodic tasks from one 10 ms cyclic
 *        handler, with missed-period counting (evidence for NFR17).
 *
 * @par
 * A periodic task registers itself with a divider of the 10 ms base period
 * and then loops on period_tick_wait().  The cyclic handler wakes it with
 * tk_wup_tsk.  If the task is still busy when its next period starts, the
 * kernel queues the wake-up; period_tick_wait() then cancels the queued
 * wake-ups and counts each one as a missed period.
 *
 * @par
 * Owner: platform / integration.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef PERIOD_TICK_H
#define PERIOD_TICK_H

#include <stdint.h>
#include <stdbool.h>

// Most periodic tasks the handler can release.
#define PERIOD_TICK_MAX_TASKS   8u

bool     period_tick_register(int32_t h_task, uint32_t divider);
int32_t  period_tick_start(void);
uint32_t period_tick_wait(void);
uint32_t period_tick_missed_total(void);

#endif /* PERIOD_TICK_H */

/*** end of file ***/


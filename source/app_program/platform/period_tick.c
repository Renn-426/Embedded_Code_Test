/** @file period_tick.c
 *
 * @brief Drift-free release of the periodic tasks from one cyclic handler.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "period_tick.h"

/*!
 * @brief One registered periodic task.
 */
typedef struct
{
    ID          h_task;
    uint32_t    divider;
    uint32_t    countdown;
} period_entry_t;

static period_entry_t    g_entries[PERIOD_TICK_MAX_TASKS];
static uint32_t          g_entry_count  = 0u;
static uint32_t volatile g_missed_total = 0u;

static void period_tick_isr(void * p_ext_info);


/*!
 * @brief Register a task to be woken every divider x 10 ms.
 *
 * @param[in] h_task   Task handle returned by robot_ipc_start_task().
 * @param[in] divider  Period in units of PERIOD_BASE_MS (at least 1).
 *
 * @return true if registered.
 *
 * @par
 * Call during initialisation, before period_tick_start().
 */
bool
period_tick_register (int32_t h_task, uint32_t divider)
{
    bool b_registered = false;

    if ((h_task > 0) && (divider > 0u) &&
        (g_entry_count < PERIOD_TICK_MAX_TASKS))
    {
        g_entries[g_entry_count].h_task    = h_task;
        g_entries[g_entry_count].divider   = divider;
        g_entries[g_entry_count].countdown = divider;
        g_entry_count++;
        b_registered = true;
    }

    return (b_registered);
}


/*!
 * @brief Create and start the 10 ms cyclic handler.
 *
 * @return The cyclic handler handle (> 0), or a negative kernel error code.
 */
int32_t
period_tick_start (void)
{
    T_CCYC const ccyc =
    {
        .exinf  = NULL,
        .cycatr = TA_HLNG | TA_STA | TA_PHS,
        .cychdr = period_tick_isr,
        .cyctim = PERIOD_BASE_MS,
        .cycphs = PERIOD_BASE_MS
    };

    return (tk_cre_cyc(&ccyc));
}


/*!
 * @brief Block the calling periodic task until its next period starts.
 *
 * @return Number of periods this task missed since the previous call
 *         (0 when it kept up).
 */
uint32_t
period_tick_wait (void)
{
    INT const queued = tk_can_wup(TSK_SELF);
    uint32_t  missed = 0u;

    if (queued > 0)
    {
        // A queued wake-up means a release arrived while the task was still
        // running: that period was missed.  queued is a small positive
        // count, so the conversion is exact.
        uint32_t intsts = 0u;

        missed = (uint32_t) queued;
        DI(intsts);
        g_missed_total += missed;
        EI(intsts);
    }

    // The next release is the start of the following period.
    (void) tk_slp_tsk(TMO_FEVR);  // Cannot time out; errors are impossible.

    return (missed);
}


/*!
 * @brief Missed periods across every periodic task since boot.
 *
 * @return Total count.
 */
uint32_t
period_tick_missed_total (void)
{
    return (g_missed_total);
}


/*!
 * @brief 10 ms cyclic handler: wake each task whose period has elapsed.
 *
 * @param[in] p_ext_info  Unused extended information.
 *
 * @par
 * Runs in the kernel's time-event context (task-independent part), so it
 * only decrements counters and issues tk_wup_tsk.
 */
static void
period_tick_isr (void * p_ext_info)
{
    (void) p_ext_info;  // Unused: one handler serves the whole table.

    for (uint32_t idx = 0u; idx < g_entry_count; idx++)
    {
        g_entries[idx].countdown--;

        if (0u == g_entries[idx].countdown)
        {
            g_entries[idx].countdown = g_entries[idx].divider;

            // A full wake-up queue already records the overrun.
            (void) tk_wup_tsk(g_entries[idx].h_task);  // E_QOVR is benign.
        }
    }
}

/*** end of file ***/


/** @file pid_ctrl.h
 *
 * @brief Integer PID controller with integral anti-windup.
 *
 * @par
 * Owner: Buddy 2 (Motion Control).  Gains are in thousandths so the whole
 * controller runs in integer arithmetic.  The sample period is fixed by the
 * caller's task period, so it is folded into the gains.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef PID_CTRL_H
#define PID_CTRL_H

#include <stdint.h>

/*!
 * @brief Controller state and tuning.
 */
typedef struct
{
    int32_t     kp_milli;
    int32_t     ki_milli;
    int32_t     kd_milli;
    int32_t     output_min;
    int32_t     output_max;
    int32_t     integral;
    int32_t     integral_limit;
    int32_t     prev_error;
} pid_ctrl_t;

void    pid_ctrl_init(pid_ctrl_t * p_pid, int32_t kp_milli, int32_t ki_milli,
                      int32_t kd_milli, int32_t output_limit);
void    pid_ctrl_reset(pid_ctrl_t * p_pid);
int32_t pid_ctrl_update(pid_ctrl_t * p_pid, int32_t setpoint,
                        int32_t measured);

#endif /* PID_CTRL_H */

/*** end of file ***/


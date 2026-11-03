/** @file pid_ctrl.c
 *
 * @brief Integer PID controller with integral anti-windup.
 *
 * @par
 * output = (Kp e + Ki sum(e) + Kd (e - e_prev)) / 1000, clamped.  The
 * integral is limited so that its contribution alone cannot exceed the
 * output range (anti-windup), and it stops growing while the output is
 * saturated in the same direction.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>

#include "fix_math.h"
#include "pid_ctrl.h"

#define GAIN_SCALE              1000


/*!
 * @brief Set the gains and output range, and clear the state.
 *
 * @param[out] p_pid         Controller.
 * @param[in]  kp_milli      Proportional gain x 1000.
 * @param[in]  ki_milli      Integral gain x 1000 (per sample).
 * @param[in]  kd_milli      Derivative gain x 1000 (per sample).
 * @param[in]  output_limit  Output is clamped to +/- this value.
 */
void
pid_ctrl_init (pid_ctrl_t * p_pid, int32_t kp_milli, int32_t ki_milli,
               int32_t kd_milli, int32_t output_limit)
{
    p_pid->kp_milli       = kp_milli;
    p_pid->ki_milli       = ki_milli;
    p_pid->kd_milli       = kd_milli;
    p_pid->output_min     = -output_limit;
    p_pid->output_max     = output_limit;
    p_pid->integral_limit = 0;

    if (0 != ki_milli)
    {
        p_pid->integral_limit = (output_limit * GAIN_SCALE) / ki_milli;
    }

    pid_ctrl_reset(p_pid);
}


/*!
 * @brief Clear the integral and derivative history.
 *
 * @param[in,out] p_pid  Controller.
 */
void
pid_ctrl_reset (pid_ctrl_t * p_pid)
{
    p_pid->integral   = 0;
    p_pid->prev_error = 0;
}


/*!
 * @brief Run one controller step.
 *
 * @param[in,out] p_pid     Controller.
 * @param[in]     setpoint  Desired value.
 * @param[in]     measured  Measured value (same unit).
 *
 * @return Controller output, clamped to the configured range.
 */
int32_t
pid_ctrl_update (pid_ctrl_t * p_pid, int32_t setpoint, int32_t measured)
{
    int32_t const error      = setpoint - measured;
    int32_t const derivative = error - p_pid->prev_error;
    int32_t const candidate  = fix_math_clamp(p_pid->integral + error,
                                              -p_pid->integral_limit,
                                              p_pid->integral_limit);
    int32_t const raw        = ((p_pid->kp_milli * error) +
                                (p_pid->ki_milli * candidate) +
                                (p_pid->kd_milli * derivative)) /
                               GAIN_SCALE;
    int32_t const output     = fix_math_clamp(raw, p_pid->output_min,
                                              p_pid->output_max);

    // Conditional integration: keep the new integral only if the output is
    // not saturated, or if the error is pulling it back out of saturation.
    if ((raw == output) || ((raw > output) && (error < 0)) ||
        ((raw < output) && (error > 0)))
    {
        p_pid->integral = candidate;
    }

    p_pid->prev_error = error;

    return (output);
}

/*** end of file ***/


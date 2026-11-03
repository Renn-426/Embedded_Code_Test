/** @file buddy2_test.c
 *
 * @brief Buddy 2 stand-alone bench test: drive straight, report the wheels.
 *
 * @par
 * Built only with BUDDY_TEST=2 (see app_main.c).  Each press of GP20 sends
 * the real MOVE command (50 cm at the slow speed) to the Motion Task, so the
 * motor driver, both encoders and both speed PIDs run exactly as on the
 * mission, with no line sensor involved.  While it moves, this task prints
 * both wheel speeds and edge counts every 0.2 s, then a summary that says
 * which wheel (if either) is the problem.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "robot_cfg.h"
#include "robot_ipc.h"
#include "board_hw.h"
#include "sys_status.h"
#include "bitpack.h"
#include "fix_math.h"
#include "encoder.h"
#include "motion.h"
#include "buddy2_test.h"

// Time to open the serial monitor after the USB port appears.
#define BOOT_DELAY_MS           4000u

// Test drive: distance and speed.
#define TEST_DISTANCE_MM        500u
#define TEST_SPEED_MM_S         SPEED_SLOW_MM_S

// Loop step, and how many steps between lines while moving (0.2 s).
#define LOOP_STEP_MS            50u
#define REPORT_STEPS            4u

// Wait this long after GO before treating IDLE as "finished".
#define START_GRACE_MS          200u

// A wheel whose edge count differs from the other by more than this many
// percent is reported as the slow or fast side.
#define MISMATCH_PERCENT        15

static uint64_t g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static char const * const gp_mode_names[] =
{
    "IDLE", "FOLLOWING", "MOVING", "TURNING", "BRAKING", "STALLED"
};

/*!
 * @brief One test drive in progress.
 */
typedef struct
{
    bool     b_running;
    uint32_t elapsed_ms;
    int32_t  start_left;
    int32_t  start_right;
} buddy2_drive_t;

static void          buddy2_test_task(INT start_code, void * p_ext_info);
static void          start_drive(buddy2_drive_t * p_drive);
static void          watch_drive(buddy2_drive_t * p_drive, uint32_t step);
static void          watch_idle(int32_t * p_last_left,
                                int32_t * p_last_right);
static void          print_summary(buddy2_drive_t const * p_drive,
                                   motion_mode_t mode);
static motion_mode_t read_mode(void);


/*!
 * @brief Create the bench-test task.
 *
 * @return The task handle (> 0), or a negative kernel error code.
 *
 * @par
 * Call after motion_start().
 */
int32_t
buddy2_test_start (void)
{
    return (robot_ipc_start_task(buddy2_test_task, PRIO_TELEMETRY, g_stack,
                                 sizeof(g_stack)));
}


/*!
 * @brief Bench-test task: one straight drive per GP20 press.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
buddy2_test_task (INT start_code, void * p_ext_info)
{
    buddy2_drive_t drive      = { false, 0u, 0, 0 };
    bool           b_was_down = false;
    uint32_t       step       = 0u;
    int32_t        last_left  = 0;
    int32_t        last_right = 0;

    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    robot_ipc_sleep_ms(BOOT_DELAY_MS);

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "\n=== AAI03 Buddy 2 straight-drive test ===\n"
                     "Each GP20 press drives %lu cm straight at %lu mm/s "
                     "(no line sensor).\nPress GP20 again to stop early."
                     "\nWhile stopped, turn a wheel by hand: its edge count "
                     "is printed.\n\n", TEST_DISTANCE_MM / 10u,
                     TEST_SPEED_MM_S);

    for (;;)
    {
        bool const b_down = board_hw_is_start_pressed();

        if (b_down && (!b_was_down))
        {
            if (drive.b_running)
            {
                (void) motion_stop();  // Retried by the next press.

                // (UB *): string literal to the monitor API's type.
                (void) tm_printf((UB *) "xxx stopped by button\n");
            }
            else
            {
                start_drive(&drive);
                step = 0u;
            }
        }

        b_was_down = b_down;

        if (drive.b_running)
        {
            step++;
            watch_drive(&drive, step);
        }
        else
        {
            watch_idle(&last_left, &last_right);
        }

        robot_ipc_sleep_ms(LOOP_STEP_MS);
    }
}


/*!
 * @brief Record the starting counts and send the MOVE command.
 *
 * @param[out] p_drive  Drive record.
 */
static void
start_drive (buddy2_drive_t * p_drive)
{
    encoder_state_t enc;

    encoder_get_state(&enc);
    p_drive->start_left  = enc.left_count;
    p_drive->start_right = enc.right_count;
    p_drive->elapsed_ms  = 0u;
    p_drive->b_running   = motion_move_forward(TEST_DISTANCE_MM,
                                               TEST_SPEED_MM_S);

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "%s\n", p_drive->b_running ?
                     ">>> GO straight" : "xxx motion queue full");
}


/*!
 * @brief Print progress, and the summary once the drive has ended.
 *
 * @param[in,out] p_drive  Drive record.
 * @param[in]     step     Loop steps since the drive started.
 */
static void
watch_drive (buddy2_drive_t * p_drive, uint32_t step)
{
    motion_mode_t const mode = read_mode();

    p_drive->elapsed_ms += LOOP_STEP_MS;

    if (0u == (step % REPORT_STEPS))
    {
        encoder_state_t enc;

        encoder_get_state(&enc);

        // (UB *): string literal to the monitor API's character type.  The
        // mode enumeration has six values, so indexing is safe.
        (void) tm_printf((UB *) "%-8s  left %4ld mm/s %5ld edges   right "
                         "%4ld mm/s %5ld edges\n", gp_mode_names[mode],
                         enc.left_speed_mm_s,
                         enc.left_count - p_drive->start_left,
                         enc.right_speed_mm_s,
                         enc.right_count - p_drive->start_right);
    }

    if ((p_drive->elapsed_ms > START_GRACE_MS) &&
        ((MOTION_MODE_IDLE == mode) || (MOTION_MODE_STALLED == mode)))
    {
        print_summary(p_drive, mode);
        (void) motion_stop();  // Clears STALLED for the next press.
        p_drive->b_running = false;
    }
}


/*!
 * @brief While stopped, print the edge counts whenever either one changes.
 *
 * @param[in,out] p_last_left   Left count last printed.
 * @param[in,out] p_last_right  Right count last printed.
 *
 * @par
 * Turning one wheel exactly one revolution by hand shows the encoder's real
 * edges per revolution (ENC_EDGES_PER_REV).  A count that changes while the
 * wheels are still means the signal is noisy.
 */
static void
watch_idle (int32_t * p_last_left, int32_t * p_last_right)
{
    encoder_state_t enc;

    encoder_get_state(&enc);

    if ((enc.left_count != *p_last_left) ||
        (enc.right_count != *p_last_right))
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "by hand: left %5ld edges   right %5ld "
                         "edges\n", enc.left_count, enc.right_count);
        *p_last_left  = enc.left_count;
        *p_last_right = enc.right_count;
    }
}


/*!
 * @brief Say which wheel, if either, misbehaved.
 *
 * @param[in] p_drive  Drive record.
 * @param[in] mode     Motion mode when the drive ended.
 */
static void
print_summary (buddy2_drive_t const * p_drive, motion_mode_t mode)
{
    encoder_state_t enc;
    int32_t         left  = 0;
    int32_t         right = 0;
    int32_t         diff  = 0;
    int32_t         most  = 0;

    encoder_get_state(&enc);
    left  = fix_math_abs(enc.left_count - p_drive->start_left);
    right = fix_math_abs(enc.right_count - p_drive->start_right);
    diff  = fix_math_abs(left - right);
    most  = (left > right) ? left : right;

    // (UB *): string literals to the monitor API's character type.
    (void) tm_printf((UB *) "--- DONE: left %ld edges, right %ld edges, "
                     "%s\n", left, right,
                     (MOTION_MODE_STALLED == mode) ? "STALLED" : "finished");

    if ((0 == left) || (0 == right))
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "RESULT: the %s encoder never counted.  "
                         "Check its wiring, or that wheel did not turn.\n",
                         (0 == left) ? "LEFT" : "RIGHT");
    }
    else if ((diff * 100) > (most * MISMATCH_PERCENT))
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "RESULT: wheels differ by more than %d "
                         "percent: the %s side turned less.  Check that "
                         "encoder disc / magnet and the motor.\n",
                         MISMATCH_PERCENT,
                         (left < right) ? "LEFT" : "RIGHT");
    }
    else
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "RESULT: both wheels matched.  If the "
                         "robot still curved, compare the wheels and "
                         "tyres.\n");
    }

    (void) tm_printf((UB *) "\n");  // (UB *): monitor API type.
}


/*!
 * @brief Current Motion Task mode.
 *
 * @return The mode from the MOTION status word.
 */
static motion_mode_t
read_mode (void)
{
    uint32_t const word = sys_status_read(SYS_STATUS_MOTION);
    uint32_t const mode = bitpack_word_get(word, MOTION_MODE_SHIFT,
                                           MOTION_MODE_WIDTH);

    // Cast: the field is checked against the enumeration's range first.
    return ((mode <= (uint32_t) MOTION_MODE_STALLED) ?
            (motion_mode_t) mode : MOTION_MODE_IDLE);
}

/*** end of file ***/


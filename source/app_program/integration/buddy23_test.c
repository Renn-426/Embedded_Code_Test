/** @file buddy23_test.c
 *
 * @brief Buddy 2 + Buddy 3 bench test: follow a line under button control.
 *
 * @par
 * Built only with BUDDY_TEST=23 (see app_main.c).  It is the first
 * integration step: Buddy 3's Line Following Task supplies the line offset
 * and Buddy 2's Motion Task steers with it, exactly as in the mission.  This
 * task stands in for Vehicle Control with a much simpler job:
 *
 *   - calibrate the centre sensor by hand (5 s sweep across the line);
 *   - START button (GP20): start following at a slow speed, press again to
 *     stop;
 *   - stop by itself when the line is lost, at a junction, or if a wheel
 *     stalls (for example an encoder that does not count);
 *   - print the line state and both wheel speeds every 0.5 s, and every
 *     barcode read on the way.
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
#include "line_follow.h"
#include "motion.h"
#include "buddy3_test.h"
#include "buddy23_test.h"

// Time to open the serial monitor after the USB port appears.
#define BOOT_DELAY_MS           4000u

// Loop step, and how many steps between status lines (0.5 s).
#define LOOP_STEP_MS            50u
#define REPORT_STEPS            10u

// Test speed: the mission's slow speed, so a mistake is easy to catch.
#define TEST_SPEED_MM_S         SPEED_SLOW_MM_S

static uint64_t g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static char const * const gp_line_names[] =
{
    "CALIBRATING", "TRACKING", "JUNCTION", "LOST", "SEARCHING"
};

static char const * const gp_mode_names[] =
{
    "IDLE", "FOLLOWING", "MOVING", "TURNING", "BRAKING", "STALLED"
};

static bool gb_running = false;

static void buddy23_test_task(INT start_code, void * p_ext_info);
static void print_banner(void);
static void check_button(bool * p_b_was_down);
static void check_stop_reasons(void);
static void stop_with_reason(char const * p_reason);
static void print_status(void);


/*!
 * @brief Create the bench-test task.
 *
 * @return The task handle (> 0), or a negative kernel error code.
 *
 * @par
 * Call after motion_start(), line_follow_start() and barcode_start().
 */
int32_t
buddy23_test_start (void)
{
    return (robot_ipc_start_task(buddy23_test_task, PRIO_TELEMETRY, g_stack,
                                 sizeof(g_stack)));
}


/*!
 * @brief Bench-test task: calibrate once, then serve the button forever.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
buddy23_test_task (INT start_code, void * p_ext_info)
{
    uint32_t step         = 0u;
    bool     b_was_down   = false;

    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    robot_ipc_sleep_ms(BOOT_DELAY_MS);
    print_banner();

    if (!buddy3_test_calibrate())
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "WARNING: calibration failed.  Press "
                         "RESET and sweep across the line again before "
                         "running.\n\n");
    }

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "READY: put the robot on the line and press "
                     "the GP20 button.\n");

    for (;;)
    {
        check_button(&b_was_down);
        check_stop_reasons();
        buddy3_test_report_events();

        step++;

        if (step >= REPORT_STEPS)
        {
            step = 0u;
            print_status();
        }

        robot_ipc_sleep_ms(LOOP_STEP_MS);
    }
}


/*!
 * @brief Print the test title and what the operator does.
 */
static void
print_banner (void)
{
    // Every (UB *) cast below only changes the signedness of a string
    // literal's characters, as the monitor API expects; (void) discards the
    // printed character count.
    (void) tm_printf((UB *) "\n=== AAI03 Buddy 2 + 3 line test ===\n");
    (void) tm_printf((UB *) "GP20 button: start / stop.  Speed %lu mm/s."
                     "\n", TEST_SPEED_MM_S);
    (void) tm_printf((UB *) "Stops by itself on line lost, junction or "
                     "motor stall.\n\n");
}


/*!
 * @brief Start or stop line following on each press of the START button.
 *
 * @param[in,out] p_b_was_down  Button level at the previous call.
 */
static void
check_button (bool * p_b_was_down)
{
    bool const b_down = board_hw_is_start_pressed();

    // Act once per press: on the step where the button goes down.
    if (b_down && (!(*p_b_was_down)))
    {
        if (gb_running)
        {
            stop_with_reason("button");
        }
        else if (motion_follow_line(TEST_SPEED_MM_S))
        {
            gb_running = true;

            // (UB *): string literal to the monitor API's character type.
            (void) tm_printf((UB *) ">>> GO: following the line\n");
        }
        else
        {
            // (UB *): string literal to the monitor API's character type.
            (void) tm_printf((UB *) "xxx motion command queue full\n");
        }
    }

    *p_b_was_down = b_down;
}


/*!
 * @brief While running, stop on line loss, a junction or a stall.
 */
static void
check_stop_reasons (void)
{
    uint32_t const            line   = line_follow_read();
    line_follow_state_t const state  = line_follow_state(line);
    uint32_t const            motion = sys_status_read(SYS_STATUS_MOTION);

    // The mode field is 3 bits wide, so the cast is exact.
    motion_mode_t const       mode   = (motion_mode_t)
                                       bitpack_word_get(motion,
                                                        MOTION_MODE_SHIFT,
                                                        MOTION_MODE_WIDTH);

    if (!gb_running)
    {
        // Nothing to supervise.
    }
    else if (MOTION_MODE_STALLED == mode)
    {
        stop_with_reason("motor stall (wheel blocked, or encoder not "
                         "counting)");
    }
    else if (LINE_STATE_JUNCTION == state)
    {
        stop_with_reason("junction (left and centre sensors both black)");
        line_follow_clear_junction();
    }
    else if ((LINE_STATE_LOST == state) || (LINE_STATE_SEARCHING == state))
    {
        stop_with_reason("line lost");
    }
    else
    {
        // Still following.
    }
}


/*!
 * @brief Stop the motors and say why.
 *
 * @param[in] p_reason  Reason text.
 */
static void
stop_with_reason (char const * p_reason)
{
    (void) motion_stop();  // A full queue is retried on the next step.
    gb_running = false;

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "xxx STOPPED: %s\n", p_reason);
}


/*!
 * @brief Print one status line: line sensor, steering and wheel speeds.
 */
static void
print_status (void)
{
    uint32_t const            line   = line_follow_read();
    line_follow_state_t const state  = line_follow_state(line);
    uint32_t const            motion = sys_status_read(SYS_STATUS_MOTION);
    uint32_t const            mode   = bitpack_word_get(motion,
                                                        MOTION_MODE_SHIFT,
                                                        MOTION_MODE_WIDTH);
    int32_t const             left   = bitpack_to_signed(
                                           bitpack_word_get(motion,
                                               MOTION_LEFT_SHIFT,
                                               MOTION_SPEED_WIDTH),
                                           MOTION_SPEED_WIDTH);
    int32_t const             right  = bitpack_to_signed(
                                           bitpack_word_get(motion,
                                               MOTION_RIGHT_SHIFT,
                                               MOTION_SPEED_WIDTH),
                                           MOTION_SPEED_WIDTH);

    // (UB *): string literal to the monitor API's character type.  The
    // state has five values and the mode six, so indexing is safe.
    (void) tm_printf((UB *) "centre %4lu  offset %5ld  %-11s  motion %-9s"
                     "  left %4ld mm/s  right %4ld mm/s  faults 0x%02lx\n",
                     bitpack_word_get(line, LINE_CENTRE_SHIFT,
                                      LINE_CENTRE_WIDTH),
                     line_follow_offset(line), gp_line_names[state],
                     // Cast: compare the enumeration as unsigned.
                     (mode <= (uint32_t) MOTION_MODE_STALLED) ?
                     gp_mode_names[mode] : "?",
                     left, right, robot_ipc_faults());
}

/*** end of file ***/


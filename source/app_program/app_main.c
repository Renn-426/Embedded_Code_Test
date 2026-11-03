/** @file app_main.c
 *
 * @brief Application entry point: start-up order for the AAI03 robot car.
 *
 * @par
 * Order matters: the board and kernel objects come first, every subsystem
 * then creates its task and attaches its GPIO handlers, and only then is the
 * shared GPIO interrupt enabled and the 10 ms period tick started, so no
 * interrupt or period release can reach a task that does not exist yet.
 *
 * @par
 * Communication is started first even though it does not need to be: the
 * radio task already joins WiFi in the background, and registering the
 * network poll early lets the link come up while the rest boots.  A
 * communication failure never prevents the robot from running (NFR1).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "board_hw.h"
#include "robot_ipc.h"
#include "gpio_irq.h"
#include "period_tick.h"
#include "comms.h"
#include "telemetry.h"
#include "motion.h"
#include "line_follow.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "vehicle.h"
#include "motor_hw.h"
#include "buddy2_test.h"
#include "buddy3_test.h"
#include "buddy23_test.h"
#include "debug_mon.h"
#include "app_main.h"

// Bench test to build: 0 is the full mission, 2 is Buddy 2 alone, 3 is
// Buddy 3 alone, 23 is Buddy 2 + Buddy 3 line following.
// Set from the command line, for example make ... BUDDY_TEST=3.
#ifndef BUDDY_TEST
#define BUDDY_TEST              0
#endif

// 1 adds the USB console monitor to the full mission (make ... DEBUG_MON=1).
#ifndef DEBUG_MON
#define DEBUG_MON               0
#endif

static uint32_t g_boot_failures = 0u;

static void note(int32_t result, uint32_t failure_bit);
#if (2 == BUDDY_TEST)
static void start_buddy2_test(void);
#elif (3 == BUDDY_TEST)
static void start_buddy3_test(void);
#elif (23 == BUDDY_TEST)
static void start_buddy23_test(void);
#else
static void start_mission(void);
#endif


/*!
 * @brief micro T-Kernel user entry point.
 *
 * @return Never returns while the robot runs (the kernel would shut down).
 */
INT
usermain (void)
{
    board_hw_init();
    note(robot_ipc_create(), APP_MAIN_FAIL_IPC);
#if (2 == BUDDY_TEST)
    start_buddy2_test();
#elif (3 == BUDDY_TEST)
    start_buddy3_test();
#elif (23 == BUDDY_TEST)
    start_buddy23_test();
#else
    start_mission();
#endif
    note(gpio_irq_start(), APP_MAIN_FAIL_GPIO_IRQ);
    note(period_tick_start(), APP_MAIN_FAIL_TICK);

    // One line on the USB console.  The (UB *) cast only changes the
    // signedness of the string literal's characters, as the monitor API
    // expects; the (void) discards the printed character count.
    (void) tm_printf((UB *) "AAI03 robot: boot failures 0x%04x\n",
                     g_boot_failures);

    // The initial task has nothing more to do and must not return.
    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);  // Nobody wakes this task.
    }
}


/*!
 * @brief Start-up steps that failed.
 *
 * @return APP_MAIN_FAIL_* bits.
 */
uint32_t
app_main_boot_failures (void)
{
    return (g_boot_failures);
}


#if (2 == BUDDY_TEST)
/*!
 * @brief Start only Buddy 2 (motors and encoders) and its bench test.
 */
static void
start_buddy2_test (void)
{
    note(motion_start(), APP_MAIN_FAIL_MOTION);  // Also starts the encoders.
    note(buddy2_test_start(), APP_MAIN_FAIL_TEST);
}
#elif (3 == BUDDY_TEST)
/*!
 * @brief Start only Buddy 3 and its bench test; the motors are held off.
 */
static void
start_buddy3_test (void)
{
    motor_hw_init();  // Drives the motor pins to coast, so nothing moves.
    note(line_follow_start(), APP_MAIN_FAIL_LINE);
    note(barcode_start(), APP_MAIN_FAIL_BARCODE);
    note(buddy3_test_start(), APP_MAIN_FAIL_TEST);
}
#elif (23 == BUDDY_TEST)
/*!
 * @brief Start Buddy 2 and Buddy 3 with the line-following bench test.
 */
static void
start_buddy23_test (void)
{
    note(motion_start(), APP_MAIN_FAIL_MOTION);  // Also starts the encoders.
    note(line_follow_start(), APP_MAIN_FAIL_LINE);
    note(barcode_start(), APP_MAIN_FAIL_BARCODE);
    note(buddy23_test_start(), APP_MAIN_FAIL_TEST);
}
#else
/*!
 * @brief Start every subsystem for the full mission.
 */
static void
start_mission (void)
{
    note(comms_start(), APP_MAIN_FAIL_COMMS);
    note(telemetry_start(), APP_MAIN_FAIL_TELEMETRY);
    note(motion_start(), APP_MAIN_FAIL_MOTION);
    note(line_follow_start(), APP_MAIN_FAIL_LINE);
    note(barcode_start(), APP_MAIN_FAIL_BARCODE);
    note(imu_start(), APP_MAIN_FAIL_IMU);
    note(obstacle_start(), APP_MAIN_FAIL_OBSTACLE);
    note(vehicle_start(), APP_MAIN_FAIL_VEHICLE);
#if (1 == DEBUG_MON)
    note(debug_mon_start(), APP_MAIN_FAIL_TEST);
#endif
}
#endif


/*!
 * @brief Record the outcome of one start-up step.
 *
 * @param[in] result       0 or a positive handle on success, negative on
 *                         failure.
 * @param[in] failure_bit  APP_MAIN_FAIL_* bit for this step.
 */
static void
note (int32_t result, uint32_t failure_bit)
{
    if (result < 0)
    {
        g_boot_failures |= failure_bit;
    }
}

/*** end of file ***/


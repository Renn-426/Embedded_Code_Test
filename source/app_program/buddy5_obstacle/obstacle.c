/** @file obstacle.c
 *
 * @brief Obstacle Scanning Task: monitoring, staged scanning, profiling.
 *
 * @par
 * Scanning is done with the robot stopped (Week 6 position: stopped
 * profiling before bypass), so servo settling and the ultrasonic flight
 * time do not need motion compensation.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "robot_ipc.h"
#include "sys_status.h"
#include "bitpack.h"
#include "telemetry.h"
#include "motion.h"
#include "servo_hw.h"
#include "ultrasonic.h"
#include "obs_profile.h"
#include "obstacle.h"

#define COARSE_ANGLES           5u
#define DETECT_CONFIRMATIONS    2u
#define REPEAT_EVERY_PINGS      8u
#define FAULT_LIMIT             3u
#define FINE_MARGIN_DEG         30u
#define SCAN_MIN_DEG            20u
#define SCAN_MAX_DEG            160u
#define MM_PER_CM               10u
#define NOTHING_CM              255u
#define PROFILE_EVT_CM_WIDTH    8u
#define PROFILE_EVT_PLAN_WIDTH  3u

/*!
 * @brief Obstacle Scanning working state.
 */
typedef struct
{
    obstacle_state_t   state;
    uint32_t           near_count;
    uint32_t           fault_run;
    uint32_t           fault_total;
    uint32_t           profiles;
    uint32_t           front_cm;
    obs_profile_plan_t last_plan;
    bool               b_detected;
    uint32_t           point_count;
} obstacle_ctx_t;

static uint32_t const g_coarse_deg[COARSE_ANGLES] =
{
    30u, 60u, 90u, 120u, 150u
};

static obstacle_ctx_t      g_ctx;
static obs_profile_point_t g_points[OBS_PROFILE_MAX_POINTS];
static uint64_t            g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void obstacle_task(INT start_code, void * p_ext_info);
static void monitor_once(void);
static bool is_turning_or_braking(void);
static void run_scan(void);
static void measure_at(uint32_t angle_deg, uint32_t settle_ms);
static void note_result(ultrasonic_result_t result);
static void send_profile(obs_profile_t const * p_profile);
static void publish(void);


/*!
 * @brief Initialise the servo and sensor and start the task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
obstacle_start (void)
{
    int32_t result = 0;

    g_ctx.state       = OBSTACLE_STATE_MONITOR;
    g_ctx.near_count  = 0u;
    g_ctx.fault_run   = 0u;
    g_ctx.fault_total = 0u;
    g_ctx.profiles    = 0u;
    g_ctx.front_cm    = NOTHING_CM;
    g_ctx.last_plan   = OBS_PLAN_CONTINUE;
    g_ctx.b_detected  = false;
    g_ctx.point_count = 0u;

    servo_hw_init();
    result = ultrasonic_init();

    if (0 == result)
    {
        int32_t const h_task = robot_ipc_start_task(obstacle_task,
                                                    PRIO_OBSTACLE, g_stack,
                                                    sizeof(g_stack));

        result = (h_task > 0) ? 0 : -1;
    }

    return (result);
}


/*!
 * @brief Ask the task to profile the obstacle (SCAN_REQ).
 *
 * @par
 * Called by Vehicle Control once the robot has stopped.  The result
 * arrives as ROBOT_EVT_OBS_PROFILE; read it with obstacle_get_profile().
 */
void
obstacle_request_scan (void)
{
    robot_ipc_flag_set(ROBOT_IPC_FLAG_OBSTACLE, ROBOT_OBS_SCAN_REQ);
}


/*!
 * @brief Take the newest profile from mbf_obs_profile without waiting.
 *
 * @param[out] p_profile  Profile.
 *
 * @return true if a profile was available.
 */
bool
obstacle_get_profile (obs_profile_t * p_profile)
{
    uint8_t  msg[ROBOT_IPC_OBS_MSG_BYTES];
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_OBS_PROFILE, msg,
                                        sizeof(msg), ROBOT_IPC_POLL);
    bool     b_got  = false;

    while (OBS_PROFILE_MSG_BYTES == length)
    {
        obs_profile_unpack(msg, p_profile);
        b_got  = true;
        length = robot_ipc_receive(ROBOT_IPC_MBF_OBS_PROFILE, msg,
                                   sizeof(msg), ROBOT_IPC_POLL);
    }

    return (b_got);
}


/*!
 * @brief Obstacle Scanning Task: monitor, or scan on request.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
obstacle_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        // The flag wait doubles as the monitoring period.
        uint32_t const request = robot_ipc_flag_wait(ROBOT_IPC_FLAG_OBSTACLE,
                                                     ROBOT_OBS_SCAN_REQ,
                                                     MONITOR_PERIOD_MS);

        if (0u != request)
        {
            run_scan();
        }
        else
        {
            monitor_once();
        }

        publish();
    }
}


/*!
 * @brief MONITOR / OBSTACLE DETECTED: one forward ping.
 */
static void
monitor_once (void)
{
    uint32_t                  distance_mm = 0u;
    ultrasonic_result_t const result      = ultrasonic_measure(&distance_mm);

    note_result(result);

    if (is_turning_or_braking())
    {
        // The sensor sweeps past the floor and nearby objects while turning.
        // Such readings are not obstacles on the path, so none may count.
        // An obstacle already reported stays reported: clearing it here
        // would make the same obstacle raise a second event after the stop.
        g_ctx.near_count = 0u;
    }
    else if ((ULTRASONIC_OK == result) && (distance_mm < OBSTACLE_DETECT_MM))
    {
        g_ctx.front_cm = distance_mm / MM_PER_CM;
        g_ctx.near_count++;

        if ((g_ctx.near_count >= DETECT_CONFIRMATIONS) && (!g_ctx.b_detected))
        {
            g_ctx.b_detected = true;
            g_ctx.state      = OBSTACLE_STATE_DETECTED;
            robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_OBSTACLE);
            telemetry_post_event(TELEMETRY_EVT_OBSTACLE, g_ctx.front_cm);
        }
        else if (0u == (g_ctx.near_count % REPEAT_EVERY_PINGS))
        {
            // Still blocked: repeat the event (about every 0.5 s) in case
            // Vehicle Control was busy with a manoeuvre the first time.
            robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_OBSTACLE);
        }
        else
        {
            // Waiting for confirmation, or recently reported.
        }
    }
    else if ((ULTRASONIC_NO_ECHO == result) ||
             ((ULTRASONIC_OK == result) &&
              (distance_mm >= OBSTACLE_RELEASE_MM)))
    {
        // Path clear again (with hysteresis between detect and release).
        g_ctx.front_cm   = (ULTRASONIC_OK == result) ?
                           bitpack_saturate(distance_mm / MM_PER_CM,
                                            OBSTACLE_DIST_WIDTH) : NOTHING_CM;
        g_ctx.near_count = 0u;
        g_ctx.b_detected = false;
        g_ctx.state      = (OBSTACLE_STATE_ECHO_FAULT == g_ctx.state) ?
                           OBSTACLE_STATE_ECHO_FAULT : OBSTACLE_STATE_MONITOR;
    }
    else
    {
        // Between the thresholds, or a sensor fault: keep the state.
    }
}


/*!
 * @brief Report whether the robot is turning or stopping after a turn.
 *
 * @return true in MOTION_MODE_TURNING or MOTION_MODE_BRAKING.
 */
static bool
is_turning_or_braking (void)
{
    uint32_t const mode = bitpack_word_get(sys_status_read(SYS_STATUS_MOTION),
                                           MOTION_MODE_SHIFT,
                                           MOTION_MODE_WIDTH);

    return ((MOTION_MODE_TURNING == mode) || (MOTION_MODE_BRAKING == mode));
}


/*!
 * @brief SCAN_REQ: coarse scan, fine scan, profile and plan.
 */
static void
run_scan (void)
{
    uint32_t      low_deg  = SERVO_CENTRE_DEG;
    uint32_t      high_deg = SERVO_CENTRE_DEG;
    obs_profile_t profile;

    // Stage 1: coarse scan.
    g_ctx.state       = OBSTACLE_STATE_COARSE_SCAN;
    g_ctx.point_count = 0u;

    for (uint32_t idx = 0u; idx < COARSE_ANGLES; idx++)
    {
        measure_at(g_coarse_deg[idx], SERVO_SETTLE_MS);
    }

    // Stage 2: fine scan across the angles that saw the obstacle.
    for (uint32_t idx = 0u; idx < g_ctx.point_count; idx++)
    {
        if ((g_points[idx].b_echo) &&
            (g_points[idx].distance_mm < OBSTACLE_PROFILE_MM))
        {
            low_deg  = (g_points[idx].angle_deg < low_deg) ?
                       g_points[idx].angle_deg : low_deg;
            high_deg = (g_points[idx].angle_deg > high_deg) ?
                       g_points[idx].angle_deg : high_deg;
        }
    }

    low_deg     = (low_deg > (SCAN_MIN_DEG + FINE_MARGIN_DEG)) ?
                  (low_deg - FINE_MARGIN_DEG) : SCAN_MIN_DEG;
    high_deg    = ((high_deg + FINE_MARGIN_DEG) < SCAN_MAX_DEG) ?
                  (high_deg + FINE_MARGIN_DEG) : SCAN_MAX_DEG;
    g_ctx.state = OBSTACLE_STATE_FINE_SCAN;

    for (uint32_t angle = low_deg; angle <= high_deg;
         angle += SCAN_FINE_STEP_DEG)
    {
        measure_at(angle, SCAN_FINE_SETTLE_MS);
    }

    servo_hw_set_angle(SERVO_CENTRE_DEG);

    // Stage 3 and 4: profile and plan.
    g_ctx.state = OBSTACLE_STATE_PROFILE;
    obs_profile_compute(g_points, g_ctx.point_count, &profile);
    send_profile(&profile);
    robot_ipc_sleep_ms(SERVO_SETTLE_MS);

    g_ctx.near_count = 0u;
    g_ctx.b_detected = false;
    g_ctx.state      = OBSTACLE_STATE_MONITOR;
}


/*!
 * @brief Move the servo, wait for it to settle, and record one sample.
 *
 * @param[in] angle_deg  Servo angle.
 * @param[in] settle_ms  Settling time before measuring.
 *
 * @par
 * Sensor faults are left out of the scan so they never look like a clear
 * path (obs_profile.h).
 */
static void
measure_at (uint32_t angle_deg, uint32_t settle_ms)
{
    uint32_t            distance_mm = 0u;
    ultrasonic_result_t result      = ULTRASONIC_FAULT;

    servo_hw_set_angle(angle_deg);
    robot_ipc_sleep_ms(settle_ms);
    result = ultrasonic_measure(&distance_mm);
    note_result(result);

    if ((ULTRASONIC_FAULT != result) &&
        (g_ctx.point_count < OBS_PROFILE_MAX_POINTS))
    {
        g_points[g_ctx.point_count].angle_deg   = angle_deg;
        g_points[g_ctx.point_count].distance_mm = distance_mm;
        g_points[g_ctx.point_count].b_echo      = (ULTRASONIC_OK == result);
        g_ctx.point_count++;
    }
}


/*!
 * @brief Count consecutive sensor faults and enter or leave ECHO FAULT.
 *
 * @param[in] result  Outcome of the latest measurement.
 */
static void
note_result (ultrasonic_result_t result)
{
    if (ULTRASONIC_FAULT == result)
    {
        g_ctx.fault_run++;

        if ((FAULT_LIMIT == g_ctx.fault_run) &&
            (OBSTACLE_STATE_ECHO_FAULT != g_ctx.state))
        {
            g_ctx.state = OBSTACLE_STATE_ECHO_FAULT;
            g_ctx.fault_total++;
            robot_ipc_raise_fault(ROBOT_FAULT_NO_ECHO);
            robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_ECHO_FAULT);
        }
    }
    else
    {
        g_ctx.fault_run = 0u;

        if (OBSTACLE_STATE_ECHO_FAULT == g_ctx.state)
        {
            g_ctx.state = OBSTACLE_STATE_MONITOR;
        }
    }
}


/*!
 * @brief Send the profile to Vehicle Control and Telemetry.
 *
 * @param[in] p_profile  Finished profile.
 */
static void
send_profile (obs_profile_t const * p_profile)
{
    uint8_t  msg[OBS_PROFILE_MSG_BYTES];
    uint32_t data = 0u;

    obs_profile_pack(p_profile, msg);
    // A full buffer means Vehicle Control still holds an unread profile,
    // so the (void) discards a result that needs no action.
    (void) robot_ipc_send(ROBOT_IPC_MBF_OBS_PROFILE, msg, sizeof(msg),
                          ROBOT_IPC_POLL);
    robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_OBS_PROFILE);

    // The plan enumeration is below 8, so the cast is exact.
    data = bitpack_word_set(data, 0u, PROFILE_EVT_PLAN_WIDTH,
                            (uint32_t) p_profile->plan);
    data = bitpack_word_set(data, PROFILE_EVT_PLAN_WIDTH,
                            PROFILE_EVT_CM_WIDTH,
                            bitpack_saturate(p_profile->width_mm / MM_PER_CM,
                                             PROFILE_EVT_CM_WIDTH));
    data = bitpack_word_set(data,
                            PROFILE_EVT_PLAN_WIDTH + PROFILE_EVT_CM_WIDTH,
                            PROFILE_EVT_CM_WIDTH,
                            bitpack_saturate(p_profile->distance_mm /
                                             MM_PER_CM,
                                             PROFILE_EVT_CM_WIDTH));
    telemetry_post_event(TELEMETRY_EVT_PROFILE, data);

    g_ctx.profiles++;
    g_ctx.last_plan = p_profile->plan;
}


/*!
 * @brief Publish the SYS_STATUS_OBSTACLE word.
 */
static void
publish (void)
{
    uint32_t word = 0u;

    // The enumeration values are below 8, so the casts are exact.
    word = bitpack_word_set(word, OBSTACLE_DIST_SHIFT, OBSTACLE_DIST_WIDTH,
                            bitpack_saturate(g_ctx.front_cm,
                                             OBSTACLE_DIST_WIDTH));
    word = bitpack_word_set(word, OBSTACLE_STATE_SHIFT, OBSTACLE_STATE_WIDTH,
                            (uint32_t) g_ctx.state);  // Enum < 8.
    word = bitpack_word_set(word, OBSTACLE_PLAN_SHIFT, OBSTACLE_PLAN_WIDTH,
                            (uint32_t) g_ctx.last_plan);  // Enum < 8.
    word = bitpack_word_set(word, OBSTACLE_PROFILES_SHIFT,
                            OBSTACLE_PROFILES_WIDTH,
                            bitpack_saturate(g_ctx.profiles,
                                             OBSTACLE_PROFILES_WIDTH));
    word = bitpack_word_set(word, OBSTACLE_FAULTS_SHIFT,
                            OBSTACLE_FAULTS_WIDTH,
                            bitpack_saturate(g_ctx.fault_total,
                                             OBSTACLE_FAULTS_WIDTH));
    sys_status_write(SYS_STATUS_OBSTACLE, word);
}

/*** end of file ***/


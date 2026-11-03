/** @file vehicle.c
 *
 * @brief Vehicle Control Task: system statechart and mission sequencing.
 *
 * @par
 * Multi-step manoeuvres (calibration sweep, junction action, bypass, line
 * search, reverse) are short lists of Motion Control steps.  Each step is
 * issued, and the next one starts when Motion Control reports MOVE_DONE.
 * MOVE_DONE is cleared before each step is issued; because this task has a
 * higher priority than Motion Control, the new command is always queued
 * before Motion Control can report completion of the previous one.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "board_hw.h"
#include "robot_ipc.h"
#include "sys_status.h"
#include "bitpack.h"
#include "fix_math.h"
#include "telemetry.h"
#include "tlm_codec.h"
#include "motion.h"
#include "line_follow.h"
#include "barcode.h"
#include "obstacle.h"
#include "nav_map.h"
#include "vehicle.h"
#include "tune.h"

#define WAIT_MS                 50
#define SEEK_WAIT_MS            10      // Line task period: edge seen in time.
#define US_PER_MS               1000u
#define FOLLOW_SLOW_MS          600u    // Slow start after every turn.
#define MAX_STEPS               8u
// Calibration sweep: out CALIB_SWEEP_DEG to one side, 2 x that to the
// other, then back towards the line until the centre sensor reaches the
// edge it follows (at most CALIB_RETURN_MAX_DEG).  Was +25/-50/+25 by
// encoder angle alone: each turn coasts past its goal, so the robot ended
// a few degrees left, and from a dead-centre start the sensor was then on
// the wrong edge of the line and the robot steered away from it.
#define CALIB_SWEEP_DEG         20
#define CALIB_RETURN_MAX_DEG    45
#define SWEEP_UNIT_DEG          30
#define BITS_PER_BYTE           8u
#define TURN_90_DEG             90
#define TURN_180_DEG            180
#define CROSS_JUNCTION_MM       100
#define APPROACH_STEP           4u
#define MILLI                   1000
#define APPROACH_FACTOR_MILLI   1500
#define MIN_PASS_MM             100
#define STATE_EVT_WIDTH         4u
#define MAP_ACK_LETTER_WIDTH    5u
#define MAP_ACK_ACTION_SHIFT    5u
#define MAP_ACK_VERSION_SHIFT   7u
#define MAP_ACK_OK_SHIFT        11u
// Signed copies of the unsigned geometry constants.  Each is below 2^16,
// so the casts are exact.
#define HALF_VEHICLE_MM         ((int32_t) (VEHICLE_WIDTH_MM / 2u))
#define LATERAL_MIN_MM          ((int32_t) BYPASS_LATERAL_MIN_MM)
#define LATERAL_MAX_MM          ((int32_t) BYPASS_LATERAL_MAX_MM)
#define PASS_MARGIN_MM          ((int32_t) BYPASS_MARGIN_MM)
#define MDEG_PER_DEG            1000
#define DEPTH_EST_MM            ((int32_t) OBSTACLE_DEPTH_EST_MM)
#define LENGTH_MM               ((int32_t) VEHICLE_LENGTH_MM)
#define JUNCTION_ADVANCE        ((int32_t) JUNCTION_ADVANCE_MM)
#define FATAL_FAULTS            (ROBOT_FAULT_MOTOR_STALL |                 \
                                 ROBOT_FAULT_ENCODER |                     \
                                 ROBOT_FAULT_NO_ECHO |                     \
                                 ROBOT_FAULT_IR_CALIB)
#define SAFETY_EVENTS           (ROBOT_EVT_IMPACT | ROBOT_EVT_STALL | \
                                 ROBOT_EVT_ECHO_FAULT)

// Bench build only (make ... BENCH_NO_IMU=1): start without a working IMU so
// the other subsystems can be tested.  The IMU fault stays raised and is
// still reported over USB and MQTT; only the start-up check ignores it.
// Hump height and impact detection do not work in this build.  Never use it
// for the mission.
#ifndef BENCH_NO_IMU
#define BENCH_NO_IMU            0
#endif

#if BENCH_NO_IMU
#define START_FAULTS            0u
#else
#define START_FAULTS            ROBOT_FAULT_IMU
#endif

/*!
 * @brief Kinds of manoeuvre step.
 */
typedef enum
{
    STEP_FORWARD = 0,
    STEP_BACKWARD,
    STEP_TURN
} step_kind_t;

/*!
 * @brief One manoeuvre step: a distance in mm, or an angle in degrees
 *        (positive = left).
 */
typedef struct
{
    step_kind_t kind;
    int32_t     amount;
} step_t;

/*!
 * @brief Vehicle Control working state.
 */
typedef struct
{
    vehicle_state_t  state;
    uint32_t         entered_us;
    step_t           steps[MAX_STEPS];
    uint32_t         step_count;
    uint32_t         step_index;
    bool             b_steps_running;
    bool             b_scan_requested;
    bool             b_follow_slow;     // Still in the slow start of following.
    bool             b_seeking_edge;    // Calibration: last swing to the edge.
    bool             b_edge_stopped;    // The edge was found; STOP issued.
    int32_t          calib_side;        // +1: sweep left first; -1: right.
    bool             b_realigning;
    int32_t          bypass_deg;        // Detour angle of the current bypass.
    bool             b_button_down;
    bool             b_latched;
    nav_map_action_t latched;
    nav_map_t        map;
    obs_profile_t    profile;
    uint32_t         junctions;
    uint32_t         bypasses;
    uint32_t         retries;
} vehicle_ctx_t;

static vehicle_ctx_t g_ctx;
static uint64_t      g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void vehicle_task(INT start_code, void * p_ext_info);
static void enter(vehicle_state_t state);
static bool is_running(void);
static void check_safety(uint32_t events);
static void take_commands(void);
static void apply_command(uint32_t word);
static void take_barcodes(void);
static void dispatch(uint32_t events);
static void on_calibrating(bool b_finished);
static bool is_on_edge(void);
static void on_line_following(uint32_t events);
static void on_obstacle_scan(uint32_t events);
static void on_bypass(uint32_t events, bool b_finished);
static void on_reacquire(uint32_t events, bool b_finished);
static void poll_button(void);
static void check_timeouts(void);
static void start_run(void);
static void follow(void);
static void begin_junction(void);
static void begin_reacquire(void);
static void begin_bypass(void);
static void begin_plan(void);
static void run_steps(step_t const * p_steps, uint32_t count);
static void issue_step(void);
static bool advance_steps(uint32_t events);
static void publish_status(void);


/*!
 * @brief Start the Vehicle Control Task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
vehicle_start (void)
{
    g_ctx.state           = VEHICLE_INITIALISING;
    g_ctx.entered_us      = board_hw_time_us();
    g_ctx.step_count      = 0u;
    g_ctx.step_index      = 0u;
    g_ctx.b_steps_running = false;
    g_ctx.b_latched       = false;
    g_ctx.latched         = NAV_ACTION_STRAIGHT;
    g_ctx.b_button_down   = false;
    g_ctx.junctions       = 0u;
    g_ctx.bypasses        = 0u;
    g_ctx.retries         = 0u;
    nav_map_init(&g_ctx.map);

    return ((robot_ipc_start_task(vehicle_task, PRIO_VEHICLE, g_stack,
                                  sizeof(g_stack)) > 0) ? 0 : -1);
}


/*!
 * @brief Vehicle Control Task: wait for events and run the statechart.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
vehicle_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        // While the calibration swing looks for the line edge, poll at the
        // line task's rate, so the robot stops within a degree of the edge.
        int32_t const  wait_ms = g_ctx.b_seeking_edge ? SEEK_WAIT_MS :
                                                        WAIT_MS;
        uint32_t const events  = robot_ipc_flag_wait(ROBOT_IPC_FLAG_VEHICLE,
                                                     ROBOT_EVT_ALL, wait_ms);

        check_safety(events);

        if (0u != (events & ROBOT_EVT_CTRL_CMD))
        {
            take_commands();
        }

        if (0u != (events & ROBOT_EVT_NAV_CMD))
        {
            take_barcodes();
        }

        dispatch(events);
        poll_button();
        check_timeouts();
        publish_status();
    }
}


/*!
 * @brief Change state, report it and restart the state timer.
 *
 * @param[in] state  New state.
 */
static void
enter (vehicle_state_t state)
{
    uint32_t data = 0u;

    // The enumeration values are below 16, so the casts are exact.
    data = bitpack_word_set(data, 0u, STATE_EVT_WIDTH, (uint32_t) state);
    data = bitpack_word_set(data, STATE_EVT_WIDTH, STATE_EVT_WIDTH,
                            (uint32_t) g_ctx.state);
    telemetry_post_event(TELEMETRY_EVT_STATE, data);  // Counted.

    g_ctx.state           = state;
    g_ctx.entered_us      = board_hw_time_us();
    g_ctx.b_steps_running = false;

    if ((VEHICLE_SAFE_STOP == state) || (VEHICLE_MISSION_COMPLETE == state))
    {
        (void) motion_stop();  // A full queue still holds an earlier STOP.
    }

    if (VEHICLE_SAFE_STOP == state)
    {
        telemetry_post_event(TELEMETRY_EVT_FAULT, robot_ipc_faults());
    }

    if (VEHICLE_MISSION_COMPLETE == state)
    {
        // Publish the run summary (FR5): Telemetry answers this request.
        robot_ipc_flag_set(ROBOT_IPC_FLAG_COMM, ROBOT_COMM_SUMMARY_REQ);
    }
}


/*!
 * @brief Report whether the robot is on a mission (motors may move).
 *
 * @return true from CALIBRATING to REVERSE_RETRY.
 */
static bool
is_running (void)
{
    return ((g_ctx.state >= VEHICLE_CALIBRATING) &&
            (g_ctx.state <= VEHICLE_REVERSE_RETRY));
}


/*!
 * @brief SAFE STOP on impact, stall, echo fault or a fatal fault.
 *
 * @param[in] events  Events received this cycle.
 */
static void
check_safety (uint32_t events)
{
    bool const b_fault = (0u != (events & ROBOT_EVT_FAULT)) &&
                         (0u != (robot_ipc_faults() & FATAL_FAULTS));

    if ((is_running()) && ((0u != (events & SAFETY_EVENTS)) || b_fault))
    {
        enter(VEHICLE_SAFE_STOP);
    }
}


/*!
 * @brief Apply every queued MQTT command.
 */
static void
take_commands (void)
{
    uint8_t  msg[ROBOT_IPC_CTRL_MSG_BYTES];
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_CTRL_CMD, msg,
                                        sizeof(msg), ROBOT_IPC_POLL);

    while (ROBOT_IPC_CTRL_MSG_BYTES == length)
    {
        uint32_t word = 0u;

        for (uint32_t idx = 0u; idx < ROBOT_IPC_CTRL_MSG_BYTES; idx++)
        {
            word |= ((uint32_t) msg[idx]) << (idx * BITS_PER_BYTE);  // Wide.
        }

        apply_command(word);
        length = robot_ipc_receive(ROBOT_IPC_MBF_CTRL_CMD, msg, sizeof(msg),
                                   ROBOT_IPC_POLL);
    }
}


/*!
 * @brief Act on one command word (tlm_codec.h layout).
 *
 * @param[in] word  Command word.
 */
static void
apply_command (uint32_t word)
{
    uint32_t const command = bitpack_word_get(word, TLM_CMD_TYPE_SHIFT,
                                              TLM_CMD_TYPE_WIDTH);
    uint32_t const letter  = bitpack_word_get(word, TLM_CMD_LETTER_SHIFT,
                                              TLM_CMD_LETTER_WIDTH);
    uint32_t const action  = bitpack_word_get(word, TLM_CMD_ACTION_SHIFT,
                                              TLM_CMD_ACTION_WIDTH);
    uint32_t const version = bitpack_word_get(word, TLM_CMD_VERSION_SHIFT,
                                              TLM_CMD_VERSION_WIDTH);
    uint32_t       ack     = 0u;

    switch (command)
    {
        case TLM_CMD_START:
            if (VEHICLE_READY == g_ctx.state)
            {
                start_run();
            }
        break;

        case TLM_CMD_STOP:
            if (is_running())
            {
                enter(VEHICLE_MISSION_COMPLETE);
            }
        break;

        case TLM_CMD_RESET:
            if (!is_running())
            {
                robot_ipc_flag_clear(ROBOT_IPC_FLAG_FAULT, FATAL_FAULTS);
                (void) motion_stop();  // Leaves STALLED (Week 6 6.2).
                enter(VEHICLE_READY);
            }
        break;

        case TLM_CMD_MAP_SET:
            // Applied immediately; affects the next barcode only.
            ack = bitpack_word_set(ack, 0u, MAP_ACK_LETTER_WIDTH, letter);
            ack = bitpack_word_set(ack, MAP_ACK_ACTION_SHIFT,
                                   NAV_MAP_ACTION_WIDTH, action);
            ack = bitpack_word_set(ack, MAP_ACK_VERSION_SHIFT,
                                   NAV_MAP_VERSION_WIDTH, version);
            ack = bitpack_word_set(ack, MAP_ACK_OK_SHIFT, 1u,
                                   nav_map_set(&g_ctx.map, letter, action,
                                               version) ? 1u : 0u);
            telemetry_post_event(TELEMETRY_EVT_MAP_ACK, ack);
        break;

        default:
            // Validated by Communication; nothing else can arrive.
        break;
    }
}


/*!
 * @brief Latch the action of the newest decoded barcode (FR3).
 *
 * @par
 * The letter is resolved through the map now, at decode time, so a later
 * map update cannot change the action already latched for the coming
 * junction.  A newer barcode replaces an older latched action.
 */
static void
take_barcodes (void)
{
    uint8_t  msg = 0u;
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_NAV_CMD, &msg,
                                        sizeof(msg), ROBOT_IPC_POLL);

    while (ROBOT_IPC_NAV_MSG_BYTES == length)
    {
        g_ctx.latched   = nav_map_get(&g_ctx.map, barcode_msg_letter(msg));
        g_ctx.b_latched = true;
        length          = robot_ipc_receive(ROBOT_IPC_MBF_NAV_CMD, &msg,
                                            sizeof(msg), ROBOT_IPC_POLL);
    }
}


/*!
 * @brief Run the handler of the current state.
 *
 * @param[in] events  Events received this cycle.
 */
static void
dispatch (uint32_t events)
{
    bool const b_finished = advance_steps(events);

    switch (g_ctx.state)
    {
        case VEHICLE_INITIALISING:
            // Wait for the IMU to calibrate while the robot is still.
            if ((board_hw_time_us() - g_ctx.entered_us) >=
                (INIT_SETTLE_MS * US_PER_MS))
            {
                enter((0u != (robot_ipc_faults() & START_FAULTS)) ?
                      VEHICLE_SAFE_STOP : VEHICLE_READY);
            }
        break;

        case VEHICLE_CALIBRATING:
            on_calibrating(b_finished);
        break;

        case VEHICLE_LINE_FOLLOWING:
            if ((g_ctx.b_follow_slow) &&
                ((board_hw_time_us() - g_ctx.entered_us) >=
                 (FOLLOW_SLOW_MS * US_PER_MS)))
            {
                // Settled on the edge: up to the full following speed.
                // The tuned speed is 30..600, so the cast is exact.
                g_ctx.b_follow_slow = false;
                (void) motion_follow_line(
                           (uint32_t) tune_get(TUNE_SPEED_FOLLOW));
            }

            on_line_following(events);
        break;

        case VEHICLE_JUNCTION_ACTION:
            if (b_finished)
            {
                g_ctx.junctions++;
                follow();
            }
        break;

        case VEHICLE_OBSTACLE_SCAN:
            on_obstacle_scan(events);
        break;

        case VEHICLE_BYPASS:
            on_bypass(events, b_finished);
        break;

        case VEHICLE_LINE_REACQUIRE:
            on_reacquire(events, b_finished);
        break;

        case VEHICLE_REVERSE_RETRY:
            if (b_finished)
            {
                // Already stopped: request the new profile straight away.
                enter(VEHICLE_OBSTACLE_SCAN);
                g_ctx.b_scan_requested = true;
                obstacle_request_scan();
            }
        break;

        default:
            // READY, SAFE STOP and MISSION COMPLETE wait for a command.
        break;
    }
}


/*!
 * @brief CALIBRATING: finish the sweep, then swing back onto the line edge.
 *
 * @param[in] b_finished  true if the step list has just finished.
 *
 * @par
 * After the two sweep turns the range is known, so the line task can tell
 * where the edge is.  The return swing is then stopped as soon as the
 * centre sensor reads half black (offset 0 or more), which leaves it on
 * the edge it follows whatever the turns overshot.  If the edge is not
 * seen within CALIB_RETURN_MAX_DEG, following starts anyway and the
 * normal line-loss search takes over.
 */
static void
on_calibrating (bool b_finished)
{
    step_t const back[] =
    {
        { STEP_TURN, g_ctx.calib_side * CALIB_RETURN_MAX_DEG }
    };

    if ((!g_ctx.b_seeking_edge) && b_finished)
    {
        // The two sweep turns are done: the range is known.
        if (line_follow_end_calibration())
        {
            g_ctx.b_seeking_edge = true;
            g_ctx.b_edge_stopped = false;
            run_steps(back, 1u);
        }
        else
        {
            enter(VEHICLE_SAFE_STOP);
        }
    }
    else if (!g_ctx.b_seeking_edge)
    {
        // Still sweeping.
    }
    else if (b_finished)
    {
        // Stopped on the edge (or the swing ran out): start following.
        g_ctx.b_seeking_edge = false;
        follow();
    }
    else if ((!g_ctx.b_edge_stopped) && (is_on_edge()))
    {
        // Its MOVE_DONE ends the swing (b_finished above).
        g_ctx.b_edge_stopped = true;
        (void) motion_stop();  // A full queue still holds an earlier STOP.
    }
    else
    {
        // Still swinging towards the edge.
    }
}


/*!
 * @brief Report whether the centre sensor is on the line edge.
 *
 * @return true once the new calibration is in use and the reading is
 *         fresh and at least half black.
 */
static bool
is_on_edge (void)
{
    uint32_t const word = line_follow_read();

    return ((LINE_STATE_CALIBRATING != line_follow_state(word)) &&
            (line_follow_age_ms(word) <= LINE_STALE_MS) &&
            (line_follow_offset(word) >= 0));
}


/*!
 * @brief LINE FOLLOWING: obstacle first, then junction, then line loss.
 *
 * @param[in] events  Events received this cycle.
 */
static void
on_line_following (uint32_t events)
{
    if (0u != (events & ROBOT_EVT_OBSTACLE))
    {
        // The latched barcode action is kept for the next junction.
        g_ctx.b_scan_requested = false;
        g_ctx.retries          = 0u;
        enter(VEHICLE_OBSTACLE_SCAN);
        (void) motion_stop();  // Its MOVE_DONE starts the scan.
    }
    else if (0u != (events & ROBOT_EVT_JUNCTION))
    {
        begin_junction();
    }
    else if (0u != (events & ROBOT_EVT_LINE_LOST))
    {
        if ((g_ctx.b_latched) && (NAV_ACTION_UTURN == g_ctx.latched))
        {
            // The line ended with a U-turn saved: run it here.
            begin_junction();
        }
        else
        {
            begin_reacquire();
        }
    }
    else
    {
        // Keep following.
    }
}


/*!
 * @brief OBSTACLE SCAN: wait until stopped, request a profile, then plan.
 *
 * @param[in] events  Events received this cycle.
 */
static void
on_obstacle_scan (uint32_t events)
{
    if ((!g_ctx.b_scan_requested) && (0u != (events & ROBOT_EVT_MOVE_DONE)))
    {
        g_ctx.b_scan_requested = true;
        obstacle_request_scan();
    }
    else if ((g_ctx.b_scan_requested) &&
             (0u != (events & ROBOT_EVT_OBS_PROFILE)) &&
             (obstacle_get_profile(&g_ctx.profile)))
    {
        begin_plan();
    }
    else
    {
        // Still stopping or scanning.
    }
}


/*!
 * @brief BYPASS: run the detour; on the approach leg, LINE_FOUND ends it.
 *
 * @param[in] events      Events received this cycle.
 * @param[in] b_finished  true if the step list has just finished.
 *
 * @par
 * LINE_LOST and JUNCTION are ignored here: leaving the line is intended,
 * and rejoining it at an angle can look like a junction (Week 6 row 6).
 */
static void
on_bypass (uint32_t events, bool b_finished)
{
    bool const b_found = (0u != (events & ROBOT_EVT_LINE_FOUND));

    if ((g_ctx.b_realigning) && b_finished)
    {
        g_ctx.bypasses++;
        follow();
    }
    else if ((!g_ctx.b_realigning) && b_found &&
             (g_ctx.step_index >= APPROACH_STEP))
    {
        // Back on the line at the detour angle: turn to the original heading.
        int32_t const angle     = (OBS_PLAN_PASS_LEFT == g_ctx.profile.plan) ?
                                  g_ctx.bypass_deg : -g_ctx.bypass_deg;
        step_t const  realign[] = { { STEP_TURN, angle } };

        g_ctx.b_realigning = true;
        run_steps(realign, 1u);
    }
    else if (b_finished)
    {
        // The approach ended without finding the line: search for it.
        begin_reacquire();
    }
    else
    {
        // Detour in progress.
    }
}


/*!
 * @brief LINE REACQUIRE: sweep until LINE_FOUND.
 *
 * @param[in] events      Events received this cycle.
 * @param[in] b_finished  true if the sweep has finished.
 */
static void
on_reacquire (uint32_t events, bool b_finished)
{
    if (0u != (events & ROBOT_EVT_LINE_FOUND))
    {
        follow();
    }
    else if (b_finished)
    {
        enter(VEHICLE_SAFE_STOP);
    }
    else
    {
        // Still sweeping.
    }
}


/*!
 * @brief Robo Pico START button: start, stop or reset on each press.
 */
static void
poll_button (void)
{
    bool const b_down = board_hw_is_start_pressed();

    if (b_down && (!g_ctx.b_button_down))
    {
        if (VEHICLE_READY == g_ctx.state)
        {
            start_run();
        }
        else if (is_running())
        {
            // Manual end of course (the brief leaves the end trigger TBC).
            enter(VEHICLE_MISSION_COMPLETE);
        }
        else if (VEHICLE_INITIALISING != g_ctx.state)
        {
            robot_ipc_flag_clear(ROBOT_IPC_FLAG_FAULT, FATAL_FAULTS);
            (void) motion_stop();  // Leaves STALLED.
            enter(VEHICLE_READY);
        }
        else
        {
            // Ignore presses while initialising.
        }
    }

    g_ctx.b_button_down = b_down;
}


/*!
 * @brief Abandon states that have run too long.
 */
static void
check_timeouts (void)
{
    uint32_t const age_ms = (board_hw_time_us() - g_ctx.entered_us) /
                            US_PER_MS;
    bool           b_late = false;

    switch (g_ctx.state)
    {
        case VEHICLE_LINE_REACQUIRE:
            b_late = (age_ms > REACQUIRE_TIMEOUT_MS);
        break;

        case VEHICLE_OBSTACLE_SCAN:
            b_late = (age_ms > PROFILE_TIMEOUT_MS);
        break;

        // The four step-list states share one limit (labels fall through).
        case VEHICLE_CALIBRATING:
        case VEHICLE_JUNCTION_ACTION:
        case VEHICLE_BYPASS:
        case VEHICLE_REVERSE_RETRY:
            b_late = (age_ms > SEQUENCE_TIMEOUT_MS);
        break;

        default:
            b_late = false;
        break;
    }

    if (b_late)
    {
        enter(VEHICLE_SAFE_STOP);
    }
}


/*!
 * @brief START: calibrate the IR sensors with a sweep across the line.
 */
static void
start_run (void)
{
    // The centre sensor follows the right-hand edge of the line when
    // steer_dir is +1 (line_follow.h), so the sweep goes left, then right
    // past the line onto white, and on_calibrating() swings back left onto
    // that edge.  With steer_dir -1 (left-hand edge) it is mirrored.
    int32_t const side    = (tune_get(TUNE_STEER_DIR) >= 0) ? 1 : -1;
    step_t const  sweep[] =
    {
        { STEP_TURN, side * CALIB_SWEEP_DEG },
        { STEP_TURN, -side * 2 * CALIB_SWEEP_DEG }
    };

    g_ctx.calib_side     = side;
    g_ctx.b_seeking_edge = false;
    g_ctx.b_edge_stopped = false;
    g_ctx.b_latched      = false;
    enter(VEHICLE_CALIBRATING);
    line_follow_begin_calibration();
    run_steps(sweep, sizeof(sweep) / sizeof(sweep[0]));
}


/*!
 * @brief Return to LINE FOLLOWING.
 *
 * @par
 * The junction hold is released here as well, because rejoining the line
 * at an angle can leave the line task holding a false junction.
 */
static void
follow (void)
{
    line_follow_clear_junction();
    enter(VEHICLE_LINE_FOLLOWING);
    // Following begins at the slow speed, because the robot has just come
    // out of a turn and may still be carrying rotation and a heading error;
    // dispatch() raises it to the full speed after FOLLOW_SLOW_MS.
    // The 4-slot command queue cannot be full: this task issues at most two
    // commands per cycle and Motion Control drains the queue every 10 ms.
    // The tuned speed is 30..600, so the cast is exact.
    g_ctx.b_follow_slow = true;
    (void) motion_follow_line((uint32_t) tune_get(TUNE_SPEED_SLOW));
}


/*!
 * @brief JUNCTION ACTION: execute the latched action (straight if none).
 */
static void
begin_junction (void)
{
    nav_map_action_t const action = g_ctx.b_latched ? g_ctx.latched :
                                                      NAV_ACTION_STRAIGHT;
    step_t                 steps[2];
    uint32_t               count  = 2u;

    // The axle must reach the junction centre before turning.
    steps[0].kind   = STEP_FORWARD;
    steps[0].amount = JUNCTION_ADVANCE;
    steps[1].kind   = STEP_TURN;

    switch (action)
    {
        case NAV_ACTION_LEFT:
            steps[1].amount = TURN_90_DEG;
        break;

        case NAV_ACTION_RIGHT:
            steps[1].amount = -TURN_90_DEG;
        break;

        case NAV_ACTION_UTURN:
            steps[0].kind   = STEP_TURN;
            steps[0].amount = TURN_180_DEG;
            count           = 1u;
        break;

        default:
            steps[0].amount = CROSS_JUNCTION_MM;
            count           = 1u;
        break;
    }

    // The action enumeration is below 4, so the cast is exact.
    telemetry_post_event(TELEMETRY_EVT_JUNCTION, (uint32_t) action);
    g_ctx.b_latched = false;
    enter(VEHICLE_JUNCTION_ACTION);
    run_steps(steps, count);
}


/*!
 * @brief LINE REACQUIRE: sweep left and right around the current heading.
 */
static void
begin_reacquire (void)
{
    // Left 1, right 2, left 3, right 4, left 2 sweep units: the sweep ends
    // on the heading it started from.
    step_t const sweep[] =
    {
        { STEP_TURN, SWEEP_UNIT_DEG },
        { STEP_TURN, -2 * SWEEP_UNIT_DEG },
        { STEP_TURN, 3 * SWEEP_UNIT_DEG },
        { STEP_TURN, -4 * SWEEP_UNIT_DEG },
        { STEP_TURN, 2 * SWEEP_UNIT_DEG }
    };

    enter(VEHICLE_LINE_REACQUIRE);
    run_steps(sweep, sizeof(sweep) / sizeof(sweep[0]));
}


/*!
 * @brief Act on the obstacle profile's plan (brief stage 4).
 */
static void
begin_plan (void)
{
    int32_t const reverse_mm = tune_get(TUNE_REVERSE_MM);  // Live setting.
    step_t const  back[]     = { { STEP_BACKWARD, reverse_mm } };

    switch (g_ctx.profile.plan)
    {
        case OBS_PLAN_CONTINUE:
            follow();
        break;

        case OBS_PLAN_PASS_LEFT:
        case OBS_PLAN_PASS_RIGHT:
            begin_bypass();
        break;

        case OBS_PLAN_REVERSE:
            g_ctx.retries++;

            if (g_ctx.retries > BYPASS_RETRIES_MAX)
            {
                enter(VEHICLE_SAFE_STOP);
            }
            else
            {
                enter(VEHICLE_REVERSE_RETRY);
                run_steps(back, 1u);
            }
        break;

        default:
            enter(VEHICLE_SAFE_STOP);
        break;
    }
}


/*!
 * @brief BYPASS: leave the line, pass the obstacle, and come back at the
 *        same angle until LINE_FOUND.
 *
 * @par
 * The sideways move comes from the profile: the obstacle edge on the
 * passing side plus half the vehicle plus BYPASS_MARGIN_MM, with the sonar
 * beam spread already taken off (obs_profile.c).  It used to be half the
 * raw width plus half the vehicle plus 60 mm at a fixed 45 degrees, and the
 * raw width of a small box is inflated by the beam, so the robot swung much
 * further out than needed.  The turn angle is then atan(sideways / ahead):
 * a small obstacle far ahead gets a shallow turn, a big one close by a
 * steep one, within BYPASS_ANGLE_MIN_DEG..BYPASS_ANGLE_MAX_DEG.
 */
static void
begin_bypass (void)
{
    int32_t const side     = (OBS_PLAN_PASS_LEFT == g_ctx.profile.plan) ?
                             1 : -1;

    // Profile distances are at most 2550 mm (8-bit cm fields) and the
    // sideways move at most 630 mm, so the conversions are exact.
    int32_t const width    = (int32_t) g_ctx.profile.width_mm;
    int32_t const ahead    = (int32_t) g_ctx.profile.distance_mm;
    int32_t const measured = (0u != g_ctx.profile.lateral_mm) ?
                             (int32_t) g_ctx.profile.lateral_mm :
                             ((width / 2) + HALF_VEHICLE_MM + PASS_MARGIN_MM);
    int32_t const lateral  = fix_math_clamp(measured, LATERAL_MIN_MM,
                                            LATERAL_MAX_MM);

    // Keep the margin between the robot's nose and the obstacle face.
    int32_t const room     = fix_math_clamp(ahead - PASS_MARGIN_MM, 1,
                                            INT32_MAX);
    int32_t const angle    = fix_math_clamp(
                                 fix_math_atan2_mdeg(lateral, room) /
                                 MDEG_PER_DEG,
                                 BYPASS_ANGLE_MIN_DEG, BYPASS_ANGLE_MAX_DEG);
    int32_t const sin_q15  = fix_math_sin_q15(angle * MDEG_PER_DEG);
    int32_t const cos_q15  = fix_math_cos_q15(angle * MDEG_PER_DEG);

    // sin is at least sin(25 deg) > 0.4, so neither division can fail.
    int32_t const leg      = (lateral * FIX_MATH_Q15_ONE) / sin_q15;
    int32_t const forward  = (lateral * cos_q15) / sin_q15;
    int32_t const pass     = fix_math_clamp(ahead + DEPTH_EST_MM +
                                            LENGTH_MM - forward,
                                            MIN_PASS_MM, INT32_MAX);
    step_t const  detour[] =
    {
        { STEP_TURN, side * angle },
        { STEP_FORWARD, leg },
        { STEP_TURN, -side * angle },
        { STEP_FORWARD, pass },
        { STEP_TURN, -side * angle },
        { STEP_FORWARD, (leg * APPROACH_FACTOR_MILLI) / MILLI }
    };

    g_ctx.bypass_deg   = angle;
    g_ctx.b_realigning = false;
    enter(VEHICLE_BYPASS);
    run_steps(detour, sizeof(detour) / sizeof(detour[0]));
}


/*!
 * @brief Load a step list and issue its first step.
 *
 * @param[in] p_steps  Steps (copied).
 * @param[in] count    Number of steps, at most MAX_STEPS.
 */
static void
run_steps (step_t const * p_steps, uint32_t count)
{
    g_ctx.step_count = (count < MAX_STEPS) ? count : MAX_STEPS;

    for (uint32_t idx = 0u; idx < g_ctx.step_count; idx++)
    {
        g_ctx.steps[idx] = p_steps[idx];
    }

    g_ctx.step_index      = 0u;
    g_ctx.b_steps_running = (g_ctx.step_count > 0u);

    if (g_ctx.b_steps_running)
    {
        issue_step();
    }
}


/*!
 * @brief Send the current step to Motion Control.
 */
static void
issue_step (void)
{
    step_t const   step   = g_ctx.steps[g_ctx.step_index];

    // |amount| is below 2^15 for every step, so the conversion is exact.
    uint32_t const amount = (uint32_t) fix_math_abs(step.amount);

    robot_ipc_flag_clear(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_MOVE_DONE);

    switch (step.kind)
    {
        case STEP_FORWARD:
            // Queued.  The tuned speed is 30..600: the cast is exact.
            (void) motion_move_forward(amount,
                                       (uint32_t) tune_get(TUNE_SPEED_SLOW));
        break;

        case STEP_BACKWARD:
            // Queued.  The tuned speed is 30..600: the cast is exact.
            (void) motion_move_backward(amount,
                                        (uint32_t) tune_get(TUNE_SPEED_SLOW));
        break;

        default:
            if (step.amount >= 0)
            {
                (void) motion_turn_left(amount);  // Queued.
            }
            else
            {
                (void) motion_turn_right(amount);  // Queued.
            }
        break;
    }
}


/*!
 * @brief Advance the step list on MOVE_DONE.
 *
 * @param[in] events  Events received this cycle.
 *
 * @return true when the last step has just completed.
 */
static bool
advance_steps (uint32_t events)
{
    bool b_finished = false;

    if ((g_ctx.b_steps_running) && (0u != (events & ROBOT_EVT_MOVE_DONE)))
    {
        g_ctx.step_index++;

        if (g_ctx.step_index < g_ctx.step_count)
        {
            issue_step();
        }
        else
        {
            g_ctx.b_steps_running = false;
            b_finished            = true;
        }
    }

    return (b_finished);
}


/*!
 * @brief Publish the SYS_STATUS_VEHICLE word.
 */
static void
publish_status (void)
{
    uint32_t word = 0u;

    // The enumeration values are below 16, so the casts are exact.
    word = bitpack_word_set(word, VEHICLE_STATE_SHIFT, VEHICLE_STATE_WIDTH,
                            (uint32_t) g_ctx.state);  // Enum < 16.
    word = bitpack_word_set(word, VEHICLE_ACTION_SHIFT, VEHICLE_ACTION_WIDTH,
                            (uint32_t) g_ctx.latched);  // Enum < 4.
    word = bitpack_word_set(word, VEHICLE_LATCHED_SHIFT, 1u,
                            g_ctx.b_latched ? 1u : 0u);
    word = bitpack_word_set(word, VEHICLE_MAP_SHIFT, VEHICLE_MAP_WIDTH,
                            nav_map_version(&g_ctx.map));
    word = bitpack_word_set(word, VEHICLE_JUNCTIONS_SHIFT,
                            VEHICLE_JUNCTIONS_WIDTH,
                            bitpack_saturate(g_ctx.junctions,
                                             VEHICLE_JUNCTIONS_WIDTH));
    word = bitpack_word_set(word, VEHICLE_BYPASSES_SHIFT,
                            VEHICLE_BYPASSES_WIDTH,
                            bitpack_saturate(g_ctx.bypasses,
                                             VEHICLE_BYPASSES_WIDTH));
    word = bitpack_word_set(word, VEHICLE_RETRIES_SHIFT,
                            VEHICLE_RETRIES_WIDTH,
                            bitpack_saturate(g_ctx.retries,
                                             VEHICLE_RETRIES_WIDTH));
    sys_status_write(SYS_STATUS_VEHICLE, word);
}

/*** end of file ***/


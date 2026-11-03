/** @file motion.c
 *
 * @brief Motion Control Task: closed-loop wheel speed, line steering,
 *        encoder-based moves and turns, braking and stall detection.
 *
 * @par
 * Statechart: Week 6 Section 6.2 (IDLE, FOLLOWING, MOVING DISTANCE, TURNING,
 * BRAKING, STALLED).  Each wheel runs its own speed PID with a feed-forward
 * term that covers the motor dead band; FOLLOW adds a PD steering
 * correction from the line offset.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "robot_ipc.h"
#include "period_tick.h"
#include "sys_status.h"
#include "bitpack.h"
#include "fix_math.h"
#include "motor_hw.h"
#include "encoder.h"
#include "pid_ctrl.h"
#include "line_follow.h"
#include "motion.h"
#include "tune.h"

#define WHEEL_LEFT              0u
#define WHEEL_RIGHT             1u
#define WHEEL_COUNT             2u
#define GAIN_SCALE              1000
#define PERMILLE_SCALE          1000
#define UM_PER_MM               1000u
#define SPEED_FIELD_WIDTH       10u
#define PID_TRIM_PERMILLE       500
#define BRAKE_DECAY_DIVISOR     2
#define BRAKE_TIMEOUT_MS        500u
#define STALL_GAP_FACTOR        3u
#define DECEL_GAIN              4u
#define DECEL_FLOOR_MM_S        80u
#define MS_PER_PERIOD           (PERIOD_BASE_MS * PERIOD_MOTION_TICKS)
#define ARG_MAGNITUDE_BITS      15u

// Signed copies of the unsigned tuning constants.  Every value is below
// 2^16, so each cast is exact.
#define MIN_DRIVE               ((int32_t) MOTOR_MIN_PERMILLE)
#define FEED_PER_M_S            ((int32_t) MOTOR_FF_PERMILLE_PER_M_S)
#define STOPPED_SPEED           ((int32_t) STOPPED_SPEED_MM_S)

// Wheel travel per degree of an on-the-spot turn is pi * track / 360, in
// um, using pi = 355/113 (120 mm track: 1047 um per degree).  The track is
// tunable live, so this is computed in turn_um_per_deg().
#define PI_NUM                  355u
#define PI_DEN                  113u
#define DEG_PER_REV             360u

/*!
 * @brief Motion Control working state (owned by the task).
 */
typedef struct
{
    motion_mode_t   mode;
    uint32_t        speed_mm_s;
    int32_t         direction;
    uint32_t        goal_um;
    int32_t         start_count[WHEEL_COUNT];
    uint32_t        mode_age_ms;
    int32_t         target[WHEEL_COUNT];
    int32_t         output[WHEEL_COUNT];
    int32_t         prev_offset;
    pid_ctrl_t      pid[WHEEL_COUNT];
} motion_ctx_t;

static motion_ctx_t g_ctx;
static uint64_t     g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void     motion_task(INT start_code, void * p_ext_info);
static bool     send_command(motion_cmd_type_t type, uint32_t speed_mm_s,
                             int32_t argument);
static void     take_commands(encoder_state_t const * p_enc);
static void     begin_command(uint32_t word, encoder_state_t const * p_enc);
static void     start_mode(uint32_t type, int32_t arg);
static void     set_targets(encoder_state_t const * p_enc);
static void     follow_targets(void);
static uint32_t travelled_um(encoder_state_t const * p_enc);
static uint32_t turn_um_per_deg(void);
static void     drive_wheels(encoder_state_t const * p_enc);
static void     check_stall(encoder_state_t const * p_enc);
static bool     is_wheel_stalled(uint32_t wheel, uint32_t edge_age_ms);
static void     enter_mode(motion_mode_t mode);
static void     publish_status(encoder_state_t const * p_enc);
static int32_t  sign_of(int32_t value);


/*!
 * @brief Initialise the motor driver and encoders and start the task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
motion_start (void)
{
    int32_t result = 0;

    motor_hw_init();

    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        pid_ctrl_init(&g_ctx.pid[wheel], PID_KP_MILLI, PID_KI_MILLI,
                      PID_KD_MILLI, PID_TRIM_PERMILLE);
        g_ctx.target[wheel]      = 0;
        g_ctx.output[wheel]      = 0;
        g_ctx.start_count[wheel] = 0;
    }

    g_ctx.mode        = MOTION_MODE_IDLE;
    g_ctx.prev_offset = 0;

    result = encoder_start();

    if (0 == result)
    {
        int32_t const h_task = robot_ipc_start_task(motion_task, PRIO_MOTION,
                                                    g_stack,
                                                    sizeof(g_stack));

        if (!period_tick_register(h_task, PERIOD_MOTION_TICKS))
        {
            result = -1;
        }
    }

    return (result);
}


/*!
 * @brief Drive forward a set distance (brief API moveForward).
 *
 * @param[in] distance_mm  Distance to travel.
 * @param[in] speed_mm_s   Cruise speed.
 *
 * @return true if the command was queued.
 */
bool
motion_move_forward (uint32_t distance_mm, uint32_t speed_mm_s)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const distance = (int32_t) bitpack_saturate(distance_mm,
                                                      ARG_MAGNITUDE_BITS);

    return (send_command(MOTION_CMD_MOVE, speed_mm_s, distance));
}


/*!
 * @brief Drive backward a set distance (brief API moveBackward).
 *
 * @param[in] distance_mm  Distance to travel.
 * @param[in] speed_mm_s   Cruise speed.
 *
 * @return true if the command was queued.
 */
bool
motion_move_backward (uint32_t distance_mm, uint32_t speed_mm_s)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const distance = (int32_t) bitpack_saturate(distance_mm,
                                                      ARG_MAGNITUDE_BITS);

    return (send_command(MOTION_CMD_MOVE, speed_mm_s, -distance));
}


/*!
 * @brief Turn on the spot to the left (brief API turnLeft).
 *
 * @param[in] angle_deg  Angle in degrees.
 *
 * @return true if the command was queued.
 */
bool
motion_turn_left (uint32_t angle_deg)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const angle = (int32_t) bitpack_saturate(angle_deg,
                                                   ARG_MAGNITUDE_BITS);

    // The tuned turn speed is 30..600, so the cast is exact.
    return (send_command(MOTION_CMD_TURN,
                         (uint32_t) tune_get(TUNE_SPEED_TURN), angle));
}


/*!
 * @brief Turn on the spot to the right (brief API turnRight).
 *
 * @param[in] angle_deg  Angle in degrees.
 *
 * @return true if the command was queued.
 */
bool
motion_turn_right (uint32_t angle_deg)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const angle = (int32_t) bitpack_saturate(angle_deg,
                                                   ARG_MAGNITUDE_BITS);

    // The tuned turn speed is 30..600, so the cast is exact.
    return (send_command(MOTION_CMD_TURN,
                         (uint32_t) tune_get(TUNE_SPEED_TURN), -angle));
}


/*!
 * @brief Follow the line at a cruise speed until another command arrives.
 *
 * @param[in] speed_mm_s  Base speed.
 *
 * @return true if the command was queued.
 */
bool
motion_follow_line (uint32_t speed_mm_s)
{
    return (send_command(MOTION_CMD_FOLLOW, speed_mm_s, 0));
}


/*!
 * @brief Ramp down and stop (brief API stop).
 *
 * @return true if the command was queued.
 */
bool
motion_stop (void)
{
    return (send_command(MOTION_CMD_STOP, 0u, 0));
}


/*!
 * @brief Pack a motion command into its 29-bit word.
 *
 * @param[in] type        Command type.
 * @param[in] speed_mm_s  Speed, saturated to 10 bits.
 * @param[in] argument    Signed distance (mm) or angle (degrees).
 *
 * @return The command word.
 */
uint32_t
motion_pack_command (motion_cmd_type_t type, uint32_t speed_mm_s,
                     int32_t argument)
{
    uint32_t word = 0u;

    // The enumeration value is below 8, so the cast is exact.
    word = bitpack_word_set(word, MOTION_CMD_TYPE_SHIFT,
                            MOTION_CMD_TYPE_WIDTH, (uint32_t) type);
    word = bitpack_word_set(word, MOTION_CMD_SPEED_SHIFT,
                            MOTION_CMD_SPEED_WIDTH,
                            bitpack_saturate(speed_mm_s, SPEED_FIELD_WIDTH));
    word = bitpack_word_set(word, MOTION_CMD_ARG_SHIFT, MOTION_CMD_ARG_WIDTH,
                            bitpack_from_signed(argument,
                                                MOTION_CMD_ARG_WIDTH));

    return (word);
}


/*!
 * @brief Motion Control Task, released every 10 ms.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
motion_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        encoder_state_t enc;

        (void) period_tick_wait();  // Missed periods are counted centrally.

        encoder_get_state(&enc);
        take_commands(&enc);
        set_targets(&enc);
        drive_wheels(&enc);
        check_stall(&enc);
        publish_status(&enc);
        g_ctx.mode_age_ms += MS_PER_PERIOD;
    }
}


/*!
 * @brief Pack a command and queue it without blocking the caller.
 *
 * @param[in] type        Command type.
 * @param[in] speed_mm_s  Speed.
 * @param[in] argument    Signed argument.
 *
 * @return true if queued.
 */
static bool
send_command (motion_cmd_type_t type, uint32_t speed_mm_s, int32_t argument)
{
    uint32_t const word = motion_pack_command(type, speed_mm_s, argument);
    uint8_t        msg[ROBOT_IPC_MOTION_MSG_BYTES];

    for (uint32_t idx = 0u; idx < ROBOT_IPC_MOTION_MSG_BYTES; idx++)
    {
        // Little-endian byte order; each byte is masked to 8 bits.
        msg[idx] = (uint8_t) ((word >> (idx * 8u)) & 0xFFu);
    }

    return (robot_ipc_send(ROBOT_IPC_MBF_MOTION_CMD, msg, sizeof(msg),
                           ROBOT_IPC_POLL));
}


/*!
 * @brief Apply every queued command; the newest one wins.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
take_commands (encoder_state_t const * p_enc)
{
    uint8_t  msg[ROBOT_IPC_MOTION_MSG_BYTES];
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_MOTION_CMD, msg,
                                        sizeof(msg), ROBOT_IPC_POLL);

    while (ROBOT_IPC_MOTION_MSG_BYTES == length)
    {
        uint32_t word = 0u;

        for (uint32_t idx = 0u; idx < ROBOT_IPC_MOTION_MSG_BYTES; idx++)
        {
            word |= ((uint32_t) msg[idx]) << (idx * 8u);  // Widening cast.
        }

        begin_command(word, p_enc);
        length = robot_ipc_receive(ROBOT_IPC_MBF_MOTION_CMD, msg,
                                   sizeof(msg), ROBOT_IPC_POLL);
    }
}


/*!
 * @brief Start executing one command word.
 *
 * @param[in] word   Packed command.
 * @param[in] p_enc  Current encoder state (start of travel).
 */
static void
begin_command (uint32_t word, encoder_state_t const * p_enc)
{
    uint32_t const type  = bitpack_word_get(word, MOTION_CMD_TYPE_SHIFT,
                                            MOTION_CMD_TYPE_WIDTH);
    int32_t const  arg   = bitpack_to_signed(
                               bitpack_word_get(word, MOTION_CMD_ARG_SHIFT,
                                                MOTION_CMD_ARG_WIDTH),
                               MOTION_CMD_ARG_WIDTH);
    uint32_t const speed = bitpack_word_get(word, MOTION_CMD_SPEED_SHIFT,
                                            MOTION_CMD_SPEED_WIDTH);

    // Only STOP may leave STALLED (Week 6 Section 6.2).
    if ((MOTION_MODE_STALLED != g_ctx.mode) || (MOTION_CMD_STOP == type))
    {
        g_ctx.speed_mm_s               = speed;
        g_ctx.direction                = (arg < 0) ? -1 : 1;
        g_ctx.start_count[WHEEL_LEFT]  = p_enc->left_count;
        g_ctx.start_count[WHEEL_RIGHT] = p_enc->right_count;
        start_mode(type, arg);
    }
}


/*!
 * @brief Enter the mode that executes a command.
 *
 * @param[in] type  Command type field.
 * @param[in] arg   Signed argument (distance in mm or angle in degrees).
 */
static void
start_mode (uint32_t type, int32_t arg)
{
    // |arg| is below 2^15, so the conversion to unsigned is exact.
    switch (type)
    {
        case MOTION_CMD_FOLLOW:
            g_ctx.prev_offset = 0;
            enter_mode(MOTION_MODE_FOLLOWING);
        break;

        case MOTION_CMD_MOVE:
            g_ctx.goal_um = (uint32_t) fix_math_abs(arg) * UM_PER_MM;  // >=0.
            enter_mode(MOTION_MODE_MOVING);
        break;

        case MOTION_CMD_TURN:
            g_ctx.goal_um = (uint32_t) fix_math_abs(arg) *  // Non-negative.
                            turn_um_per_deg();
            enter_mode(MOTION_MODE_TURNING);
        break;

        default:
            enter_mode(MOTION_MODE_BRAKING);
        break;
    }
}


/*!
 * @brief Work out each wheel's target speed for the current mode.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
set_targets (encoder_state_t const * p_enc)
{
    // The speed field is at most 1023, so it converts to int32_t exactly.
    int32_t const speed = (int32_t) g_ctx.speed_mm_s;

    switch (g_ctx.mode)
    {
        case MOTION_MODE_FOLLOWING:
            follow_targets();
        break;

        case MOTION_MODE_MOVING:
        {
            uint32_t const done_um = travelled_um(p_enc);
            uint32_t const left_mm = (g_ctx.goal_um > done_um) ?
                                     ((g_ctx.goal_um - done_um) / UM_PER_MM) :
                                     0u;
            uint32_t const ramp    = DECEL_FLOOR_MM_S + (left_mm * DECEL_GAIN);

            // ramp and speed are below 2^16: the conversion is exact.
            int32_t const  cruise  = (ramp < g_ctx.speed_mm_s) ?
                                     (int32_t) ramp : speed;

            g_ctx.target[WHEEL_LEFT]  = cruise * g_ctx.direction;
            g_ctx.target[WHEEL_RIGHT] = cruise * g_ctx.direction;

            if (0u == left_mm)
            {
                enter_mode(MOTION_MODE_BRAKING);
            }
        }
        break;

        case MOTION_MODE_TURNING:
            g_ctx.target[WHEEL_LEFT]  = -speed * g_ctx.direction;
            g_ctx.target[WHEEL_RIGHT] = speed * g_ctx.direction;

            if (travelled_um(p_enc) >= g_ctx.goal_um)
            {
                enter_mode(MOTION_MODE_BRAKING);
            }
        break;

        default:
            g_ctx.target[WHEEL_LEFT]  = 0;
            g_ctx.target[WHEEL_RIGHT] = 0;
        break;
    }
}


/*!
 * @brief Line-following targets: base speed plus PD steering.
 */
static void
follow_targets (void)
{
    uint32_t const word = line_follow_read();
    int32_t const  base = (int32_t) g_ctx.speed_mm_s;  // At most 1023.

    if (line_follow_age_ms(word) > LINE_STALE_MS)
    {
        // Stale steering data: go straight and slowly (Week 6 negotiation
        // point with Buddy 3 on steering-data age).
        g_ctx.target[WHEEL_LEFT]  = tune_get(TUNE_SPEED_SLOW);
        g_ctx.target[WHEEL_RIGHT] = tune_get(TUNE_SPEED_SLOW);
    }
    else
    {
        int32_t const offset = tune_get(TUNE_STEER_DIR) *
                               line_follow_offset(word);
        int32_t const raw    = ((tune_get(TUNE_STEER_KP) * offset) +
                                (tune_get(TUNE_STEER_KD) *
                                 (offset - g_ctx.prev_offset))) /
                               GAIN_SCALE;
        int32_t const steer  = fix_math_clamp(raw, -base, base);

        g_ctx.target[WHEEL_LEFT]  = base + steer;
        g_ctx.target[WHEEL_RIGHT] = base - steer;
        g_ctx.prev_offset         = offset;
    }
}


/*!
 * @brief Wheel travel per degree of an on-the-spot turn, in um.
 *
 * @return pi * track / 360 for the current (tunable) track width.
 */
static uint32_t
turn_um_per_deg (void)
{
    // The tuned track is 50..300 mm, so the cast is exact.
    uint32_t const track_mm = (uint32_t) tune_get(TUNE_TRACK_MM);

    return ((track_mm * UM_PER_MM * PI_NUM) / (DEG_PER_REV * PI_DEN));
}


/*!
 * @brief Mean absolute wheel travel since the current command started.
 *
 * @param[in] p_enc  Current encoder state.
 *
 * @return Travel in micrometres.
 */
static uint32_t
travelled_um (encoder_state_t const * p_enc)
{
    int32_t const left  = fix_math_abs(p_enc->left_count -
                                       g_ctx.start_count[WHEEL_LEFT]);
    int32_t const right = fix_math_abs(p_enc->right_count -
                                       g_ctx.start_count[WHEEL_RIGHT]);

    // Both counts are non-negative, so the conversion is exact.
    // The tuned edge length is 500..30000, so its cast is exact too.
    return (((uint32_t) (left + right) *
             (uint32_t) tune_get(TUNE_ENC_UM_PER_EDGE)) / 2u);
}


/*!
 * @brief Run the wheel speed controllers and drive the motors.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
drive_wheels (encoder_state_t const * p_enc)
{
    int32_t const measured[WHEEL_COUNT] =
    {
        p_enc->left_speed_mm_s, p_enc->right_speed_mm_s
    };

    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        int32_t const target = g_ctx.target[wheel];
        int32_t       drive  = 0;

        if (MOTION_MODE_BRAKING == g_ctx.mode)
        {
            // Ramp the last output down rather than reversing the motor.
            drive = g_ctx.output[wheel] / BRAKE_DECAY_DIVISOR;
        }
        else if ((0 != target) && (MOTION_MODE_STALLED != g_ctx.mode))
        {
            int32_t const feed = sign_of(target) *
                                 (MIN_DRIVE +
                                  ((fix_math_abs(target) * FEED_PER_M_S) /
                                   PERMILLE_SCALE));

            drive = feed + pid_ctrl_update(&g_ctx.pid[wheel], target,
                                           measured[wheel]);
        }
        else
        {
            pid_ctrl_reset(&g_ctx.pid[wheel]);
        }

        g_ctx.output[wheel] = fix_math_clamp(drive, -MOTOR_HW_FULL_PERMILLE,
                                             MOTOR_HW_FULL_PERMILLE);
    }

    encoder_set_direction(sign_of(g_ctx.output[WHEEL_LEFT]),
                          sign_of(g_ctx.output[WHEEL_RIGHT]));
    motor_hw_set(g_ctx.output[WHEEL_LEFT], g_ctx.output[WHEEL_RIGHT]);

    // Braking ends once both wheels have (almost) stopped, or after a time
    // limit, so encoder noise on a stopped wheel cannot hold up the next step.
    if ((MOTION_MODE_BRAKING == g_ctx.mode) &&
        (((fix_math_abs(p_enc->left_speed_mm_s) < STOPPED_SPEED) &&
          (fix_math_abs(p_enc->right_speed_mm_s) < STOPPED_SPEED)) ||
         (g_ctx.mode_age_ms >= BRAKE_TIMEOUT_MS)))
    {
        motor_hw_set(0, 0);
        enter_mode(MOTION_MODE_IDLE);
        robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_MOVE_DONE);
    }
}


/*!
 * @brief Detect a stalled wheel: drive applied but no encoder edges.
 *
 * @param[in] p_enc  Current encoder state.
 *
 * @par
 * Week 6: if PWM is above its minimum but no encoder pulse arrives for
 * STALL_TIMEOUT_MS, stop the motors and raise STALL.
 */
static void
check_stall (encoder_state_t const * p_enc)
{
    bool const b_driving = (MOTION_MODE_FOLLOWING == g_ctx.mode) ||
                           (MOTION_MODE_MOVING == g_ctx.mode) ||
                           (MOTION_MODE_TURNING == g_ctx.mode);
    bool const b_left    = is_wheel_stalled(WHEEL_LEFT,
                                            p_enc->left_edge_age_ms);
    bool const b_right   = is_wheel_stalled(WHEEL_RIGHT,
                                            p_enc->right_edge_age_ms);

    if (b_driving && (g_ctx.mode_age_ms > STALL_TIMEOUT_MS) &&
        (b_left || b_right))
    {
        motor_hw_set(0, 0);
        g_ctx.output[WHEEL_LEFT]  = 0;
        g_ctx.output[WHEEL_RIGHT] = 0;
        enter_mode(MOTION_MODE_STALLED);
        robot_ipc_raise_fault(ROBOT_FAULT_MOTOR_STALL);
        robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_STALL);
    }
}


/*!
 * @brief Change mode and restart the mode timer.
 *
 * @param[in] mode  New mode.
 */
static void
enter_mode (motion_mode_t mode)
{
    g_ctx.mode        = mode;
    g_ctx.mode_age_ms = 0u;

    // A new mode starts clean: trim wound up in a turn must not carry into
    // the next move, where it would keep the robot turning.
    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        pid_ctrl_reset(&g_ctx.pid[wheel]);
    }
}


/*!
 * @brief Decide whether one driven wheel has stopped producing edges.
 *
 * @param[in] wheel        WHEEL_LEFT or WHEEL_RIGHT.
 * @param[in] edge_age_ms  Time since the wheel's last edge.
 *
 * @return true if the wheel is driven and its edges are overdue.
 *
 * @par
 * A slowed inner wheel is still moving even when its edges are far apart,
 * so the limit grows with the edge interval expected at the target speed.
 */
static bool
is_wheel_stalled (uint32_t wheel, uint32_t edge_age_ms)
{
    int32_t const target    = fix_math_abs(g_ctx.target[wheel]);
    bool          b_stalled = false;

    if ((0 != target) && (fix_math_abs(g_ctx.output[wheel]) >= MIN_DRIVE))
    {
        // um per edge / mm per s = ms per edge.  Both are positive here, so
        // the casts are exact.
        uint32_t const gap_ms = (uint32_t) tune_get(TUNE_ENC_UM_PER_EDGE) /
                                (uint32_t) target;
        uint32_t const limit  = gap_ms * STALL_GAP_FACTOR;

        b_stalled = edge_age_ms > ((limit > STALL_TIMEOUT_MS) ?
                                   limit : STALL_TIMEOUT_MS);
    }

    return (b_stalled);
}


/*!
 * @brief Publish the SYS_STATUS_MOTION word.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
publish_status (encoder_state_t const * p_enc)
{
    uint32_t word = 0u;

    // The enumeration value is below 8, so the cast is exact.
    word = bitpack_word_set(word, MOTION_MODE_SHIFT, MOTION_MODE_WIDTH,
                            (uint32_t) g_ctx.mode);
    word = bitpack_word_set(word, MOTION_LEFT_SHIFT, MOTION_SPEED_WIDTH,
                            bitpack_from_signed(p_enc->left_speed_mm_s,
                                                MOTION_SPEED_WIDTH));
    word = bitpack_word_set(word, MOTION_RIGHT_SHIFT, MOTION_SPEED_WIDTH,
                            bitpack_from_signed(p_enc->right_speed_mm_s,
                                                MOTION_SPEED_WIDTH));
    word = bitpack_word_set(word, MOTION_STALLED_SHIFT, 1u,
                            (MOTION_MODE_STALLED == g_ctx.mode) ? 1u : 0u);
    sys_status_write(SYS_STATUS_MOTION, word);
}


/*!
 * @brief Sign of a value.
 *
 * @param[in] value  Any value.
 *
 * @return -1, 0 or +1.
 */
static int32_t
sign_of (int32_t value)
{
    int32_t sign = 0;

    if (value > 0)
    {
        sign = 1;
    }
    else if (value < 0)
    {
        sign = -1;
    }
    else
    {
        // Zero has no sign.
    }

    return (sign);
}

/*** end of file ***/


/** @file encoder.c
 *
 * @brief Wheel encoders: interrupt edge counting and the Encoder Task.
 *
 * @par
 * The interrupt handler only counts edges and timestamps them.  The Encoder
 * Task runs every 10 ms, copies the interrupt data inside a short critical
 * section, converts it to speed and distance, and publishes the result under
 * the enc_state mutex.  Speed comes from the interval between the last two
 * edges rather than from edges per period, because at 200 mm/s fewer than
 * one edge arrives in 10 ms.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "board_hw.h"
#include "gpio_irq.h"
#include "robot_ipc.h"
#include "period_tick.h"
#include "sys_status.h"
#include "bitpack.h"
#include "encoder.h"
#include "tune.h"

#define WHEEL_LEFT              0u
#define WHEEL_RIGHT             1u
#define WHEEL_COUNT             2u
#define US_PER_MS               1000u
#define UM_PER_MM               1000
#define MM_PER_CM               10
#define EDGE_TIMEOUT_US         (ENC_EDGE_TIMEOUT_MS * US_PER_MS)
#define SPEED_WINDOW            5u      // Periods the speed is averaged over.

// Edge length, tunable live (TUNE_ENC_UM_PER_EDGE, 50..30000 um).
// NOTE: the signed distance product overflows after about 200 m of travel
// per wheel, far beyond one 5-minute run.
#define UM_PER_EDGE             (tune_get(TUNE_ENC_UM_PER_EDGE))

/*!
 * @brief Per-wheel data written by the interrupt handler.
 */
typedef struct
{
    int32_t     count;
    uint32_t    last_edge_us;
    uint32_t    period_us;
    int32_t     direction;
} wheel_isr_t;

// Interrupt data; volatile because the GPIO handler writes it.
static wheel_isr_t volatile g_wheels[WHEEL_COUNT];

// Published snapshot (enc_state), guarded by ROBOT_IPC_LOCK_ENC_STATE.
static encoder_state_t      g_state;

// Recent counts and their times, for the speed (encoder task only).
static int32_t              g_hist_count[WHEEL_COUNT][SPEED_WINDOW];
static uint32_t             g_hist_us[SPEED_WINDOW];
static uint32_t             g_hist_next = 0u;

static uint64_t             g_stack[STACK_SMALL / sizeof(uint64_t)];

static void    encoder_task(INT start_code, void * p_ext_info);
static int32_t wheel_speed(uint32_t wheel, wheel_isr_t const * p_wheel,
                           uint32_t now_us);
static void    publish(encoder_state_t const * p_state);
static void    encoder_edge_isr(uint32_t pin, uint32_t edges,
                                uint32_t time_us, uint32_t levels);


/*!
 * @brief Configure the encoder inputs and start the Encoder Task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
encoder_start (void)
{
    int32_t result = 0;

    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        g_wheels[wheel].count        = 0;
        g_wheels[wheel].last_edge_us = 0u;
        g_wheels[wheel].period_us    = 0u;
        g_wheels[wheel].direction    = 1;
    }

    board_hw_gpio_input(PIN_ENC_LEFT_A, BOARD_HW_PULL_UP);
    board_hw_gpio_input(PIN_ENC_LEFT_B, BOARD_HW_PULL_UP);
    board_hw_gpio_input(PIN_ENC_RIGHT_A, BOARD_HW_PULL_UP);
    board_hw_gpio_input(PIN_ENC_RIGHT_B, BOARD_HW_PULL_UP);

    if ((!gpio_irq_attach(PIN_ENC_LEFT_A, encoder_edge_isr, true, true)) ||
        (!gpio_irq_attach(PIN_ENC_RIGHT_A, encoder_edge_isr, true, true)))
    {
        result = -1;
    }
    else
    {
        int32_t const h_task = robot_ipc_start_task(encoder_task,
                                                    PRIO_ENCODER, g_stack,
                                                    sizeof(g_stack));

        if (!period_tick_register(h_task, PERIOD_ENCODER_TICKS))
        {
            result = -1;
        }
    }

    return (result);
}


/*!
 * @brief Tell a single-channel encoder which way each wheel is driven.
 *
 * @param[in] left_sign   +1 forward, -1 reverse (0 keeps the last value).
 * @param[in] right_sign  +1 forward, -1 reverse (0 keeps the last value).
 *
 * @par
 * Called by the Motion Control task.  Ignored for quadrature encoders,
 * which measure direction themselves.  Each store is a single word write.
 */
void
encoder_set_direction (int32_t left_sign, int32_t right_sign)
{
    if (0 != left_sign)
    {
        g_wheels[WHEEL_LEFT].direction = (left_sign > 0) ? 1 : -1;
    }

    if (0 != right_sign)
    {
        g_wheels[WHEEL_RIGHT].direction = (right_sign > 0) ? 1 : -1;
    }
}


/*!
 * @brief Copy the latest published encoder state.
 *
 * @param[out] p_state  Destination.
 */
void
encoder_get_state (encoder_state_t * p_state)
{
    robot_ipc_lock(ROBOT_IPC_LOCK_ENC_STATE);
    *p_state = g_state;
    robot_ipc_unlock(ROBOT_IPC_LOCK_ENC_STATE);
}


/*!
 * @brief Encoder Task: convert edge data to speed and distance every 10 ms.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
encoder_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        wheel_isr_t     copy[WHEEL_COUNT];
        encoder_state_t state;
        uint32_t        intsts = 0u;
        uint32_t        now_us = 0u;

        (void) period_tick_wait();  // Missed periods are counted centrally.

        // Copy the interrupt data atomically: it spans several words.
        DI(intsts);
        now_us = board_hw_time_us();

        for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
        {
            copy[wheel].count        = g_wheels[wheel].count;
            copy[wheel].last_edge_us = g_wheels[wheel].last_edge_us;
            copy[wheel].period_us    = g_wheels[wheel].period_us;
            copy[wheel].direction    = g_wheels[wheel].direction;
        }

        EI(intsts);

        state.left_count        = copy[WHEEL_LEFT].count;
        state.right_count       = copy[WHEEL_RIGHT].count;
        state.left_speed_mm_s   = wheel_speed(WHEEL_LEFT, &copy[WHEEL_LEFT],
                                              now_us);
        state.right_speed_mm_s  = wheel_speed(WHEEL_RIGHT,
                                              &copy[WHEEL_RIGHT], now_us);

        // Remember this sample for the speed window.
        g_hist_count[WHEEL_LEFT][g_hist_next]  = copy[WHEEL_LEFT].count;
        g_hist_count[WHEEL_RIGHT][g_hist_next] = copy[WHEEL_RIGHT].count;
        g_hist_us[g_hist_next]                 = now_us;
        g_hist_next                            = (g_hist_next + 1u) %
                                                 SPEED_WINDOW;
        state.distance_mm       = ((state.left_count + state.right_count) *
                                   UM_PER_EDGE) / (2 * UM_PER_MM);
        state.left_edge_age_ms  = (now_us - copy[WHEEL_LEFT].last_edge_us) /
                                  US_PER_MS;
        state.right_edge_age_ms = (now_us -
                                   copy[WHEEL_RIGHT].last_edge_us) /
                                  US_PER_MS;
        state.time_us           = now_us;

        publish(&state);
    }
}


/*!
 * @brief Speed of one wheel from its count over the last SPEED_WINDOW
 *        periods.
 *
 * @param[in] wheel    WHEEL_LEFT or WHEEL_RIGHT.
 * @param[in] p_wheel  Copied interrupt data of the wheel.
 * @param[in] now_us   Current time.
 *
 * @return Signed speed in mm/s; 0 if no edge arrived recently.
 *
 * @par
 * The count rate is used rather than the last edge interval: with motor
 * shaft encoders edges arrive every millisecond, two often share one
 * interrupt, and a single interval then reads half the true speed.  The
 * sign comes from the counted direction.
 */
static int32_t
wheel_speed (uint32_t wheel, wheel_isr_t const * p_wheel, uint32_t now_us)
{
    int32_t speed = 0;

    if ((now_us - p_wheel->last_edge_us) < EDGE_TIMEOUT_US)
    {
        // The oldest sample in the ring is the one about to be replaced.
        uint32_t const oldest   = g_hist_next;
        uint32_t const span_ms  = (now_us - g_hist_us[oldest]) / US_PER_MS;
        int32_t const  edges    = p_wheel->count - g_hist_count[wheel][oldest];

        if (span_ms > 0u)
        {
            // edges * um/edge / ms = um/ms = mm/s.  |edges| stays below
            // 1000 in 50 ms and the edge length below 30000, so the
            // product fits; span_ms is at most a few hundred, exact as
            // int32_t.
            speed = (edges * UM_PER_EDGE) / (int32_t) span_ms;
        }
    }

    return (speed);
}


/*!
 * @brief Publish a snapshot to enc_state and the status board.
 *
 * @param[in] p_state  New snapshot.
 */
static void
publish (encoder_state_t const * p_state)
{
    uint32_t counts = 0u;
    uint32_t odometry = 0u;

    robot_ipc_lock(ROBOT_IPC_LOCK_ENC_STATE);
    g_state = *p_state;
    robot_ipc_unlock(ROBOT_IPC_LOCK_ENC_STATE);

    // Counts and distance are reported modulo 2^16 (the decoder unwraps
    // them); the casts keep the low bits of the two's complement value.
    counts   = bitpack_word_set(counts, ENCODER_LEFT_SHIFT,
                                ENCODER_COUNT_WIDTH,
                                (uint32_t) p_state->left_count);  // Low bits.
    counts   = bitpack_word_set(counts, ENCODER_RIGHT_SHIFT,
                                ENCODER_COUNT_WIDTH,
                                (uint32_t) p_state->right_count);  // Low.
    odometry = bitpack_word_set(odometry, 0u, ENCODER_DISTANCE_WIDTH,
                                (uint32_t) (p_state->distance_mm /
                                            MM_PER_CM));
    sys_status_write(SYS_STATUS_ENC_COUNTS, counts);
    sys_status_write(SYS_STATUS_ODOMETRY, odometry);
}


/*!
 * @brief GPIO handler for both encoder channel-A pins (interrupt context).
 *
 * @param[in] pin      PIN_ENC_LEFT_A or PIN_ENC_RIGHT_A.
 * @param[in] edges    GPIO_IRQ_EDGE_* bits (two bits = two edges).
 * @param[in] time_us  Interrupt entry time.
 * @param[in] levels   Pin levels at interrupt entry (for channel B).
 */
static void
encoder_edge_isr (uint32_t pin, uint32_t edges, uint32_t time_us,
                  uint32_t levels)
{
    uint32_t const wheel   = (PIN_ENC_LEFT_A == pin) ? WHEEL_LEFT :
                                                       WHEEL_RIGHT;
    uint32_t const pin_b   = (WHEEL_LEFT == wheel) ? PIN_ENC_LEFT_B :
                                                     PIN_ENC_RIGHT_B;
    int32_t        steps   = 1;
    int32_t        heading = g_wheels[wheel].direction;

    if ((time_us - g_wheels[wheel].last_edge_us) < ENC_MIN_EDGE_US)
    {
        // Too soon after the last edge to be a real slot: noise.  Ignore it
        // so it neither counts nor shortens the measured period.
        steps = 0;
    }
    else if ((0u != (edges & GPIO_IRQ_EDGE_RISING)) &&
             (0u != (edges & GPIO_IRQ_EDGE_FALLING)))
    {
        steps = 2;
    }

    if (0u != ENC_QUADRATURE)
    {
        // Channel B equal to channel A after the edge means forward.
        bool const b_level_a = (0u != ((levels >> pin) & 1u));
        bool const b_level_b = (0u != ((levels >> pin_b) & 1u));

        heading = (b_level_a == b_level_b) ? 1 : -1;
    }

    if (steps > 0)
    {
        g_wheels[wheel].count       += steps * heading;
        g_wheels[wheel].period_us    = time_us -
                                       g_wheels[wheel].last_edge_us;
        g_wheels[wheel].last_edge_us = time_us;
    }
}

/*** end of file ***/


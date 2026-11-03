/** @file barcode.c
 *
 * @brief Barcode Task: interrupt edge capture and Code 39 decoding.
 *
 * @par
 * FIFO entries are 16 bits (Change Requirement 1): bit 15 is the sensor
 * level after the edge (1 = black) and bits 14..0 the duration of the
 * element that just ended, in 16 us ticks (saturating at 524 ms).  The ISR
 * writes the head index and the task writes the tail index; the two are
 * separate words, never packed together, so neither side performs a
 * read-modify-write on data the other side writes.
 *
 * @par
 * The quiet zone that ends a capture is adaptive: at least BARCODE_QUIET_MS
 * and at least four times the widest element seen so far.  A Code 39 quiet
 * zone is at least ten narrow elements wide while a wide element is at most
 * three, so this does not split a barcode above about 45 mm/s.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#ifndef DEBUG_MON
#define DEBUG_MON               0
#endif
#if (1 == DEBUG_MON)
#include <tm/tmonitor.h>
#endif

#include "robot_cfg.h"
#include "board_hw.h"
#include "gpio_irq.h"
#include "ir_hw.h"
#include "robot_ipc.h"
#include "sys_status.h"
#include "bitpack.h"
#include "telemetry.h"
#include "code39.h"
#include "barcode.h"

#define TICK_US                 16u
#define ENTRY_BLACK_BIT         0x8000u
#define ENTRY_WIDTH_MASK        0x7FFFu
#define MAX_ELEMENTS            36u
#define QUIET_FACTOR            4u
#define US_PER_MS               1000u
#define LETTER_COUNT            26u
#define FIRST_LETTER            0x41u     // ASCII code of the letter A.
#define ADC_IRQ_LEVEL           2         // Below the GPIO edge interrupt.

/*!
 * @brief One finished capture.
 */
typedef struct
{
    uint32_t    widths[MAX_ELEMENTS];
    uint32_t    count;
    bool        b_overflow;
    bool        b_early;        // Ended on a decode, not on a quiet zone.
} capture_t;

// Interrupt-to-task FIFO.  Head and overflow count: written by the ISR only.
// Tail: written by the task only.
static uint16_t volatile g_fifo[BARCODE_FIFO_SLOTS];
static uint32_t volatile g_fifo_head      = 0u;
static uint32_t volatile g_fifo_tail      = 0u;
static uint32_t volatile g_overflow_count = 0u;
static uint32_t          g_last_edge_us   = 0u;     // ISR only.

#if BARCODE_SENSOR_ANALOGUE
#define BOX_LEN                 BARCODE_AVERAGE   // Running average length.

/*!
 * @brief Edge finder state (ADC interrupt only).
 */
typedef struct
{
    uint32_t    valley;        // Lowest darkness of the current white element.
    uint32_t    peak_env;      // Highest darkness seen recently (decays).
    uint32_t    bar_peak;      // Highest darkness of the current bar.
    uint32_t    sample_idx;    // Samples seen so far (wraps; differences used).
    uint32_t    bar_peak_idx;  // Sample index of the current bar's peak.
    uint32_t    last_fall_idx; // Sample index of the last bar's end.
    uint32_t    prev1;         // Previous raw darkness (median input).
    uint32_t    prev2;         // The one before that.
    uint32_t    box[BOX_LEN];  // Last median outputs (average input).
    uint32_t    box_sum;
    uint32_t    box_pos;
    bool        b_black;
    bool        b_started;
} sampler_t;

#define US_PER_SAMPLE           (US_PER_MS * 1000u / IR_SAMPLE_HZ)
#define MIN_BAR_SAMPLES         ((BARCODE_MIN_BAR_MS * IR_SAMPLE_HZ) / 1000u)

static sampler_t         g_sampler;
static uint16_t          g_history[BARCODE_HISTORY];   // ISR only.

// For the debug monitor: single aligned stores from the interrupt.
static uint32_t volatile g_dbg_dark   = 0u;
static uint32_t volatile g_dbg_white  = 0u;
static uint32_t volatile g_dbg_black  = 0u;

#if (1 == DEBUG_MON)
// Bench aid: the last eight seconds of darkness readings, printed with each
// capture so the waveform can be replayed on a PC.
#define TRACE_SAMPLES           16384u
#define TRACE_DECIMATE          8u
#define TRACE_PER_LINE          32u
#define TRACE_LINE_PAUSE_MS     25          // Lets the USB console drain.
static uint16_t volatile g_trace[TRACE_SAMPLES];
static uint32_t volatile g_trace_head   = 0u;   // ISR only.
static bool volatile     gb_trace_frozen = false;  // Task: hold while printing.
#endif
#endif

static ID                gh_task          = 0;
static uint32_t          g_status         = 0u;     // Task only.
static uint32_t          g_sequence       = 0u;     // Task only.
static uint32_t          g_reads          = 0u;     // Task only.
static uint32_t          g_rejects        = 0u;     // Task only.
static uint64_t          g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void     barcode_task(INT start_code, void * p_ext_info);
static void     capture(capture_t * p_capture);
static void     settle(void);
static bool     drain(capture_t * p_capture, bool b_first);
static void     report(uint8_t symbol, bool b_reversed);
static void     reject(barcode_reject_t reason, uint32_t count);
static void     set_state(barcode_state_t state);
static barcode_reject_t to_reject(code39_result_t result);
static void     push_edge(uint32_t time_us, bool b_black);
#if (1 == DEBUG_MON)
static void     dump_capture(capture_t const * p_capture);
#if BARCODE_SENSOR_ANALOGUE
static void     dump_trace(void);
#endif
#endif
#if BARCODE_SENSOR_ANALOGUE
static void     barcode_adc_isr(UINT int_number);
static void     on_sample(uint32_t raw, uint32_t time_us);
static void     end_bar(sampler_t * p_s, uint32_t mid, uint32_t time_us);
static uint32_t median3(uint32_t a_val, uint32_t b_val, uint32_t c_val);
static uint32_t filter_sample(sampler_t * p_s, uint32_t now);
#else
static void     barcode_edge_isr(uint32_t pin, uint32_t edges,
                                 uint32_t time_us, uint32_t levels);
#endif


/*!
 * @brief Configure the barcode input and start the Barcode Task.
 *
 * @return 0 on success, or a negative error code.
 *
 * @par
 * The IR pins themselves are configured by line_follow_start() (ir_hw).
 */
int32_t
barcode_start (void)
{
    int32_t result = 0;

    gh_task = robot_ipc_start_task(barcode_task, PRIO_BARCODE, g_stack,
                                   sizeof(g_stack));

#if BARCODE_SENSOR_ANALOGUE
    {
        // The ADC itself was started by ir_hw_init() (Line Following).
        T_DINT const dint =
        {
            .intatr = TA_HLNG,
            .inthdr = barcode_adc_isr
        };

        if ((gh_task <= 0) || (E_OK != tk_def_int(IR_HW_ADC_IRQ_NUM, &dint)))
        {
            result = -1;
        }
        else
        {
            EnableInt(IR_HW_ADC_IRQ_NUM, ADC_IRQ_LEVEL);
        }
    }
#else
    if ((gh_task <= 0) ||
        (!gpio_irq_attach(PIN_IR_BARCODE_DO, barcode_edge_isr, true, true)))
    {
        result = -1;
    }
#endif

    return (result);
}


/*!
 * @brief Current analogue levels for the debug monitor.
 *
 * @param[out] p_dark   Latest reading as darkness (0 = brightest).
 * @param[out] p_white  White level the edge finder is working from.
 * @param[out] p_black  Black level the edge finder is working from.
 *
 * @par
 * All zero with the digital sensor.
 */
void
barcode_debug_levels (uint32_t * p_dark, uint32_t * p_white,
                      uint32_t * p_black)
{
#if BARCODE_SENSOR_ANALOGUE
    *p_dark  = g_dbg_dark;
    *p_white = g_dbg_white;
    *p_black = g_dbg_black;
#else
    *p_dark  = 0u;
    *p_white = 0u;
    *p_black = 0u;
#endif
}


/*!
 * @brief Extract the letter from a navigation message.
 *
 * @param[in] msg  Navigation message byte.
 *
 * @return Letter index, 0 = A .. 25 = Z.
 */
uint32_t
barcode_msg_letter (uint8_t msg)
{
    return (bitpack_word_get(msg, BARCODE_LETTER_SHIFT,
                             BARCODE_LETTER_WIDTH));
}


/*!
 * @brief Barcode Task: IDLE, CAPTURING, DECODING, REPORT or REJECT.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
barcode_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        capture_t cap;

        // IDLE: wait for the first edge.
        set_state(BARCODE_STATE_IDLE);
        (void) tk_slp_tsk(TMO_FEVR);  // No timeout: cannot fail.

        set_state(BARCODE_STATE_CAPTURING);
        capture(&cap);

#if (1 == DEBUG_MON)
        dump_capture(&cap);
#endif
        set_state(BARCODE_STATE_DECODING);

        if (cap.b_overflow)
        {
            reject(BARCODE_REJECT_OVERFLOW, cap.count);
        }
        else
        {
            uint8_t               symbol     = 0u;
            bool                  b_reversed = false;
            code39_result_t const result     = code39_decode(cap.widths,
                                                             cap.count,
                                                             &symbol,
                                                             &b_reversed);

            if (CODE39_OK == result)
            {
                report(symbol, b_reversed);
            }
            else
            {
                reject(to_reject(result), cap.count);
            }
        }

        if (cap.b_early)
        {
            // The command is out; now let the rest of the label pass.
            settle();
        }

        // Discard wake-ups queued by edges that belonged to this capture.
        (void) tk_can_wup(TSK_SELF);  // The count itself is not needed.
    }
}


/*!
 * @brief Collect elements until a quiet zone passes with no edge.
 *
 * @param[out] p_capture  The finished capture.
 */
static void
capture (capture_t * p_capture)
{
    uint32_t const overflow_before = g_overflow_count;
    bool           b_first         = true;
    bool           b_quiet         = false;

    p_capture->count      = 0u;
    p_capture->b_overflow = false;
    p_capture->b_early    = false;

    while (!b_quiet)
    {
        uint32_t widest     = 0u;
        uint32_t quiet_ms   = BARCODE_QUIET_MS;
        uint8_t  symbol     = 0u;
        bool     b_reversed = false;

        b_first = drain(p_capture, b_first);

        // A complete barcode ends the capture at once, so the command is
        // latched well before the junction whatever follows the last bar.
        if (CODE39_OK == code39_decode(p_capture->widths, p_capture->count,
                                       &symbol, &b_reversed))
        {
            p_capture->b_early = true;
            break;
        }

        for (uint32_t idx = 0u; idx < p_capture->count; idx++)
        {
            widest = (p_capture->widths[idx] > widest) ?
                     p_capture->widths[idx] : widest;
        }

        if (((widest * TICK_US * QUIET_FACTOR) / US_PER_MS) > quiet_ms)
        {
            quiet_ms = (widest * TICK_US * QUIET_FACTOR) / US_PER_MS;
        }

        // quiet_ms is at most about 2.1 s, so the conversion is exact.
        b_quiet = (E_TMOUT == tk_slp_tsk((TMO) quiet_ms));
    }

    (void) drain(p_capture, b_first);  // Final drain; result not needed.
    p_capture->b_overflow = (overflow_before != g_overflow_count);
}


/*!
 * @brief Wait out the quiet zone after an early decode, discarding edges.
 *
 * @par
 * A capture that ends on a decode leaves the trailing edges of the label
 * (its last quiet zone and the edge of the paper) still to come.  Left
 * alone they would start a new capture and be rejected as too short.  So
 * the task stays here until BARCODE_QUIET_MS passes with no edge, dropping
 * whatever arrives, before it returns to IDLE.
 */
static void
settle (void)
{
    capture_t scratch;
    bool      b_quiet = false;

    while (!b_quiet)
    {
        // BARCODE_QUIET_MS is a small positive constant: the cast is exact.
        b_quiet = (E_TMOUT == tk_slp_tsk((TMO) BARCODE_QUIET_MS));
    }

    scratch.count = 0u;
    (void) drain(&scratch, true);  // Empties the FIFO; contents not needed.
}


/*!
 * @brief Move FIFO entries into the capture.
 *
 * @param[in,out] p_capture  Capture being built.
 * @param[in]     b_first    true if no entry has been taken yet.
 *
 * @return The updated "no entry taken yet" flag.
 *
 * @par
 * The first entry of a capture is the surface before the first edge, so it
 * is not stored.  code39_decode() finds the frame among the rest.
 */
static bool
drain (capture_t * p_capture, bool b_first)
{
    bool     b_still_first = b_first;
    uint32_t tail          = g_fifo_tail;

    while (tail != g_fifo_head)
    {
        uint32_t const entry = g_fifo[tail];

        if (b_still_first)
        {
            b_still_first = false;
        }
        else if (p_capture->count < MAX_ELEMENTS)
        {
            p_capture->widths[p_capture->count] = entry & ENTRY_WIDTH_MASK;
            p_capture->count++;
        }
        else
        {
            // Too many elements for a single-letter frame.
            p_capture->b_overflow = true;
        }

        tail = (tail + 1u) % BARCODE_FIFO_SLOTS;
    }

    g_fifo_tail = tail;

    return (b_still_first);
}


/*!
 * @brief Send a decoded letter to Vehicle Control and Telemetry.
 *
 * @param[in] symbol      ASCII letter 'A'..'Z'.
 * @param[in] b_reversed  true if read backwards.
 */
static void
report (uint8_t symbol, bool b_reversed)
{
    uint32_t const letter = (uint32_t) symbol - FIRST_LETTER;  // Widening.
    uint32_t       msg    = 0u;
    uint8_t        byte   = 0u;

    set_state(BARCODE_STATE_REPORT);
    g_sequence = (g_sequence + 1u) % (1u << BARCODE_SEQ_WIDTH);
    g_reads++;

    msg  = bitpack_word_set(msg, BARCODE_LETTER_SHIFT, BARCODE_LETTER_WIDTH,
                            letter % LETTER_COUNT);
    msg  = bitpack_word_set(msg, BARCODE_REVERSED_SHIFT, 1u,
                            b_reversed ? 1u : 0u);
    msg  = bitpack_word_set(msg, BARCODE_SEQ_SHIFT, BARCODE_SEQ_WIDTH,
                            g_sequence);
    byte = (uint8_t) msg;  // msg uses only bits 7..0: exact.

    (void) robot_ipc_send(ROBOT_IPC_MBF_NAV_CMD, &byte, sizeof(byte),
                          ROBOT_IPC_POLL);  // Full buffer: newest dropped.
    robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_NAV_CMD);
    telemetry_post_event(TELEMETRY_EVT_BARCODE, msg);

    // The low 8 bits of the status word repeat the message.
    g_status = bitpack_word_set(g_status, 0u, BARCODE_MSG_WIDTH, msg);
    g_status = bitpack_word_set(g_status, BARCODE_READS_SHIFT,
                                BARCODE_COUNT_WIDTH,
                                bitpack_saturate(g_reads,
                                                 BARCODE_COUNT_WIDTH));
    sys_status_write(SYS_STATUS_BARCODE, g_status);
}


/*!
 * @brief Record a rejected capture.  Nothing is sent to Vehicle Control.
 *
 * @param[in] reason  Why the capture was rejected.
 * @param[in] count   Number of elements captured.
 */
static void
reject (barcode_reject_t reason, uint32_t count)
{
    uint32_t data = 0u;

    set_state(BARCODE_STATE_REJECT);
    g_rejects++;

    // The enumeration value is below 8, so the cast is exact.
    data = bitpack_word_set(data, 0u, BARCODE_REJECT_REASON_WIDTH,
                            (uint32_t) reason);  // Enum < 8.
    data = bitpack_word_set(data, BARCODE_REJECT_COUNT_SHIFT,
                            BARCODE_REJECT_COUNT_WIDTH,
                            bitpack_saturate(count,
                                             BARCODE_REJECT_COUNT_WIDTH));
    telemetry_post_event(TELEMETRY_EVT_DECODE_FAIL, data);

    if (BARCODE_REJECT_OVERFLOW == reason)
    {
        robot_ipc_raise_fault(ROBOT_FAULT_BARCODE_OVF);
    }

    g_status = bitpack_word_set(g_status, BARCODE_REJECTS_SHIFT,
                                BARCODE_COUNT_WIDTH,
                                bitpack_saturate(g_rejects,
                                                 BARCODE_COUNT_WIDTH));
    sys_status_write(SYS_STATUS_BARCODE, g_status);
}


/*!
 * @brief Record the task state in the status word.
 *
 * @param[in] state  New state.
 */
static void
set_state (barcode_state_t state)
{
    // The enumeration value is below 8, so the cast is exact.
    g_status = bitpack_word_set(g_status, BARCODE_STATE_SHIFT,
                                BARCODE_STATE_WIDTH, (uint32_t) state);
    sys_status_write(SYS_STATUS_BARCODE, g_status);
}


/*!
 * @brief Map a decoder result to a reject reason.
 *
 * @param[in] result  Failed decoder result.
 *
 * @return Reject reason.
 */
static barcode_reject_t
to_reject (code39_result_t result)
{
    barcode_reject_t reason = BARCODE_REJECT_COUNT;

    switch (result)
    {
        case CODE39_BAD_RATIO:
            reason = BARCODE_REJECT_RATIO;
        break;

        case CODE39_NO_MATCH:
            reason = BARCODE_REJECT_NO_MATCH;
        break;

        case CODE39_BAD_FRAME:
            reason = BARCODE_REJECT_FRAME;
        break;

        default:
            reason = BARCODE_REJECT_COUNT;
        break;
    }

    return (reason);
}


#if (1 == DEBUG_MON)
/*!
 * @brief Print the element widths of a capture on the USB console.
 *
 * @param[in] p_capture  The finished capture.
 *
 * @par
 * Bench aid (DEBUG_MON=1 builds only).  Widths are in milliseconds and
 * alternate bar, space, bar... when the capture began on a bar.
 */
static void
dump_capture (capture_t const * p_capture)
{
    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "cap %lu elements%s:", p_capture->count,
                     p_capture->b_early ? " (decoded)" : "");

    for (uint32_t idx = 0u; idx < p_capture->count; idx++)
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) " %lu",
                         (p_capture->widths[idx] * TICK_US) / US_PER_MS);
    }

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "\n");

#if BARCODE_SENSOR_ANALOGUE
    if (p_capture->count >= 10u)
    {
        dump_trace();
    }
#endif
}


#if BARCODE_SENSOR_ANALOGUE
/*!
 * @brief Print the darkness trace, oldest first, one sample in four.
 */
static void
dump_trace (void)
{
    uint32_t idx = 0u;

    gb_trace_frozen = true;       // The ISR stops writing; order is stable.
    idx             = g_trace_head;  // Oldest sample.

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "trace %lu samples at %lu Hz:\n",
                     TRACE_SAMPLES / TRACE_DECIMATE,
                     IR_SAMPLE_HZ / TRACE_DECIMATE);

    for (uint32_t num = 0u; num < (TRACE_SAMPLES / TRACE_DECIMATE); num++)
    {
        bool const b_line_end = (0u == ((num + 1u) % TRACE_PER_LINE));

        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "%lu%s", (uint32_t) g_trace[idx],
                         b_line_end ? "\n" : " ");
        idx = (idx + TRACE_DECIMATE) % TRACE_SAMPLES;

        if (b_line_end)
        {
            (void) tk_dly_tsk(TRACE_LINE_PAUSE_MS);  // Cannot fail here.
        }
    }

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "end trace\n");
    gb_trace_frozen = false;
}
#endif
#endif


/*!
 * @brief Record one black/white edge for the Barcode task.
 *
 * @param[in] time_us  Time of the edge.
 * @param[in] b_black  true if the surface is black from this edge on.
 *
 * @par
 * Interrupt context.  The entry holds the width of the element that has
 * just ended, in TICK_US units, and the colour that starts now.
 */
static void
push_edge (uint32_t time_us, bool b_black)
{
    uint32_t const ticks = (time_us - g_last_edge_us) / TICK_US;
    uint32_t const next  = (g_fifo_head + 1u) % BARCODE_FIFO_SLOTS;
    uint32_t       entry = (ticks > ENTRY_WIDTH_MASK) ? ENTRY_WIDTH_MASK :
                                                        ticks;

    g_last_edge_us = time_us;

    if (b_black)
    {
        entry |= ENTRY_BLACK_BIT;
    }

    if (next == g_fifo_tail)
    {
        g_overflow_count++;
    }
    else
    {
        g_fifo[g_fifo_head] = (uint16_t) entry;  // entry fits in 16 bits.
        g_fifo_head         = next;
    }

    (void) tk_wup_tsk(gh_task);  // E_QOVR is benign: the task drains all.
}


#if BARCODE_SENSOR_ANALOGUE
/*!
 * @brief ADC interrupt: a pair of conversions is ready.
 *
 * @param[in] int_number  Unused: this handler serves one interrupt only.
 */
static void
barcode_adc_isr (UINT int_number)
{
    uint32_t raw = 0u;

    (void) int_number;  // Unused: this handler serves one interrupt only.

    if (ir_hw_sampler_take(&raw))
    {
        on_sample(raw, board_hw_time_us());
    }
}


/*!
 * @brief Turn one barcode conversion into black/white edges.
 *
 * @param[in] raw      ADC reading.
 * @param[in] time_us  Time of the reading.
 *
 * @par
 * The reading is first turned into darkness, so black is always the high
 * side.  The white level follows the lowest darkness of the current white
 * element and the black level the highest darkness seen recently.  A bar is
 * noticed when the darkness climbs an eighth of that contrast above white
 * and ends when it drops below half of the bar's own height.  Both edges
 * are then recorded at that half-height level, the start by looking back
 * through the recent samples, so a bar narrower than the sensor's spot,
 * which never reaches full black, is still measured at its true width and
 * the spaces beside it are not stretched.  With no contrast (plain surface)
 * no edges are produced at all.
 */
static void
on_sample (uint32_t raw, uint32_t time_us)
{
    uint32_t const    now  = (0u != IR_AO_BLACK_IS_HIGH) ?
                             raw : (IR_HW_ADC_MAX - raw);
    sampler_t * const p_s  = &g_sampler;
    uint32_t          dark = 0u;

    if (!p_s->b_started)
    {
        p_s->valley        = now;
        p_s->peak_env      = now;
        p_s->bar_peak      = now;
        p_s->sample_idx    = 0u;
        p_s->bar_peak_idx  = 0u;
        p_s->last_fall_idx = 0u;
        p_s->prev1         = now;
        p_s->prev2         = now;
        p_s->box_sum       = BOX_LEN * now;
        p_s->box_pos       = 0u;
        p_s->b_black       = false;
        p_s->b_started     = true;

        for (uint32_t pos = 0u; pos < BOX_LEN; pos++)
        {
            p_s->box[pos] = now;
        }
    }

    dark = filter_sample(p_s, now);

    g_history[p_s->sample_idx % BARCODE_HISTORY] = (uint16_t) dark;  // 12 bits.

    if (dark > p_s->peak_env)
    {
        p_s->peak_env = dark;
    }
    else
    {
        p_s->peak_env -= (p_s->peak_env - dark) >> BARCODE_DECAY_SHIFT;
    }

    if (p_s->b_black)
    {
        uint32_t const mid = p_s->valley + ((p_s->bar_peak - p_s->valley) / 2u);

        if (dark > p_s->bar_peak)
        {
            p_s->bar_peak     = dark;
            p_s->bar_peak_idx = p_s->sample_idx;
        }

        if (dark < mid)
        {
            end_bar(p_s, mid, time_us);
            p_s->valley = dark;
        }
    }
    else
    {
        uint32_t span = 0u;

        if (dark < p_s->valley)
        {
            p_s->valley = dark;
        }
        else
        {
            p_s->valley += (dark - p_s->valley) >> BARCODE_DECAY_SHIFT;
        }

        span = p_s->peak_env - p_s->valley;

        if ((span >= BARCODE_MIN_SPAN) &&
            (dark > (p_s->valley + (span >> BARCODE_ENTER_SHIFT))))
        {
            p_s->b_black      = true;
            p_s->bar_peak     = dark;
            p_s->bar_peak_idx = p_s->sample_idx;
        }
    }

    p_s->sample_idx++;

    g_dbg_dark  = dark;
    g_dbg_white = p_s->valley;
    g_dbg_black = p_s->peak_env;

#if (1 == DEBUG_MON)
    if (!gb_trace_frozen)
    {
        g_trace[g_trace_head] = (uint16_t) dark;  // 12-bit value: exact.
        g_trace_head          = (g_trace_head + 1u) % TRACE_SAMPLES;
    }
#endif
}


/*!
 * @brief Record both edges of a bar that has just ended.
 *
 * @param[in,out] p_s      Edge finder state.
 * @param[in]     mid      Half-height level of the bar (darkness).
 * @param[in]     time_us  Time of the sample that fell below it.
 *
 * @par
 * The bar's start is where the darkness first climbed past the same level:
 * walk back from the peak through the history, no further than the previous
 * bar's end.
 */
static void
end_bar (sampler_t * p_s, uint32_t mid, uint32_t time_us)
{
    uint32_t       back  = p_s->sample_idx - p_s->bar_peak_idx;
    uint32_t const since = p_s->sample_idx - p_s->last_fall_idx;
    uint32_t const limit = (since < (BARCODE_HISTORY - 1u)) ?
                           since : (BARCODE_HISTORY - 1u);

    while ((back < limit) &&
           (g_history[(p_s->sample_idx - back - 1u) % BARCODE_HISTORY] >= mid))
    {
        back++;
    }

    p_s->b_black = false;

    // Too short, or too faint against the contrast seen so far, is noise.
    if ((back >= MIN_BAR_SAMPLES) &&
        ((p_s->bar_peak - p_s->valley) >=
         ((p_s->peak_env - p_s->valley) >> BARCODE_ACCEPT_SHIFT)))
    {
        push_edge(time_us - (back * US_PER_SAMPLE), true);
        push_edge(time_us, false);
        p_s->last_fall_idx = p_s->sample_idx;
    }
}


/*!
 * @brief Smooth one darkness reading.
 *
 * @param[in,out] p_s  Edge finder state (filter memory).
 * @param[in]     now  Raw darkness.
 *
 * @return Filtered darkness.
 *
 * @par
 * A single-sample spike cannot pass the 3-sample median; the running
 * average of BARCODE_AVERAGE samples then cuts the remaining noise, at a
 * cost of half that many samples of delay.
 */
static uint32_t
filter_sample (sampler_t * p_s, uint32_t now)
{
    uint32_t const med = median3(p_s->prev2, p_s->prev1, now);

    p_s->prev2 = p_s->prev1;
    p_s->prev1 = now;

    p_s->box_sum          -= p_s->box[p_s->box_pos];
    p_s->box[p_s->box_pos] = med;
    p_s->box_sum          += med;
    p_s->box_pos           = (p_s->box_pos + 1u) % BOX_LEN;

    return (p_s->box_sum / BOX_LEN);
}


/*!
 * @brief Median of three values.
 *
 * @param[in] a_val  First value.
 * @param[in] b_val  Second value.
 * @param[in] c_val  Third value.
 *
 * @return The middle value.
 */
static uint32_t
median3 (uint32_t a_val, uint32_t b_val, uint32_t c_val)
{
    uint32_t const hi_ab = (a_val > b_val) ? a_val : b_val;
    uint32_t const lo_ab = (a_val > b_val) ? b_val : a_val;
    uint32_t const mid   = (c_val > hi_ab) ? hi_ab :
                           ((c_val < lo_ab) ? lo_ab : c_val);

    return (mid);
}
#else
/*!
 * @brief GPIO interrupt: one edge of the digital barcode output.
 *
 * @param[in] pin      PIN_IR_BARCODE_DO.
 * @param[in] edges    GPIO_IRQ_EDGE_* bits.
 * @param[in] time_us  Time of the edge.
 * @param[in] levels   Pin levels at that time.
 */
static void
barcode_edge_isr (uint32_t pin, uint32_t edges, uint32_t time_us,
                  uint32_t levels)
{
    uint32_t const level = (levels >> pin) & 1u;

    if ((GPIO_IRQ_EDGE_RISING | GPIO_IRQ_EDGE_FALLING) == edges)
    {
        g_overflow_count++;  // Both edges in one interrupt: a pulse lost.
    }

    push_edge(time_us, (IR_DO_BLACK_IS_HIGH == level));
}
#endif

/*** end of file ***/


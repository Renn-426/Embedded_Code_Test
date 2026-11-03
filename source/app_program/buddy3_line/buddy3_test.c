/** @file buddy3_test.c
 *
 * @brief Buddy 3 stand-alone bench test: IR sensors, line state, barcodes.
 *
 * @par
 * Built only with BUDDY_TEST=3 (see app_main.c).  It exercises the real
 * Buddy 3 code, not a copy: the Line Following and Barcode tasks run exactly
 * as on the mission, and this low-priority task only watches them.
 *
 * @par
 * Sequence: wait for the serial monitor, print the pin map, run a 5 s
 * calibration sweep, then every 0.5 s print the IR readings and line state,
 * and print every barcode letter and decode failure as soon as it happens.
 * Vehicle Control is not running, so this task also releases a JUNCTION hold
 * that Vehicle Control would normally release.
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
#include "sys_status.h"
#include "bitpack.h"
#include "ir_hw.h"
#include "line_follow.h"
#include "barcode.h"
#include "telemetry.h"
#include "buddy3_test.h"

// Time to open the serial monitor after the USB port appears.
#define BOOT_DELAY_MS           4000u

// Calibration sweep: length, and how often the centre reading is sampled.
#define CALIB_MS                5000u
#define CALIB_STEP_MS           20u
#define CALIB_DOT_STEPS         25u

// Main loop: events are checked every step, the status line printed every
// REPORT_STEPS steps (0.5 s).
#define LOOP_STEP_MS            50u
#define REPORT_STEPS            10u

// The barcode FIFO and the event buffer hold far fewer than this.
#define DRAIN_LIMIT             16u

#define BITS_PER_BYTE           8u
#define LETTER_A                65u     // ASCII 'A'.
#define LETTER_COUNT            26u

static uint64_t g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static char const * const gp_line_names[] =
{
    "CALIBRATING", "TRACKING", "JUNCTION", "LOST", "SEARCHING"
};

static char const * const gp_reject_names[] =
{
    "FIFO overflow", "wrong bar count", "bad wide/narrow ratio",
    "no Code 39 match", "bad frame", "no start character"
};

static void buddy3_test_task(INT start_code, void * p_ext_info);
static void print_banner(void);
static void drain_barcodes(void);
static void drain_events(void);
static void print_event(uint32_t type, uint32_t data);
static void print_status(void);
static char const * black_or_white(bool b_black);


/*!
 * @brief Create the bench-test task.
 *
 * @return The task handle (> 0), or a negative kernel error code.
 *
 * @par
 * Call after line_follow_start() and barcode_start().
 */
int32_t
buddy3_test_start (void)
{
    return (robot_ipc_start_task(buddy3_test_task, PRIO_TELEMETRY, g_stack,
                                 sizeof(g_stack)));
}


/*!
 * @brief Bench-test task: calibrate once, then report forever.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
buddy3_test_task (INT start_code, void * p_ext_info)
{
    uint32_t step = 0u;

    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    robot_ipc_sleep_ms(BOOT_DELAY_MS);
    print_banner();
    (void) buddy3_test_calibrate();  // Result is printed.

    for (;;)
    {
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
 * @brief Print the test title and the Buddy 3 pin map.
 */
static void
print_banner (void)
{
    // Every (UB *) cast below only changes the signedness of a string
    // literal's characters, as the monitor API expects; (void) discards the
    // printed character count.
    (void) tm_printf((UB *) "\n=== AAI03 Buddy 3 bench test ===\n");
    (void) tm_printf((UB *) "Motors are OFF.  Only Buddy 3 runs.\n");
    (void) tm_printf((UB *) "Centre IR AO  GP%lu (ADC%lu)\n",
                     PIN_IR_CENTRE_AO, ADC_INPUT_IR_CENTRE);
    (void) tm_printf((UB *) "Left IR DO    GP%lu\n",  // UB: API type.
                     PIN_IR_LEFT_DO);
    (void) tm_printf((UB *) "Barcode IR DO GP%lu\n\n",  // UB: API type.
                     PIN_IR_BARCODE_DO);
}


/*!
 * @brief Run the real calibration and report the range it saw.
 *
 * @par
 * The user slides the centre sensor across white and black.  The range is
 * sampled here from the line_state word, which the line task refreshes
 * every 10 ms, so the ADC is never read by two tasks.
 *
 * @return true if the calibration passed.
 */
bool
buddy3_test_calibrate (void)
{
    uint32_t lowest  = IR_HW_ADC_MAX;
    uint32_t highest = 0u;
    bool     b_ok    = false;

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "CALIBRATION: for the next 5 s, slide the "
                     "centre sensor slowly across WHITE and the BLACK "
                     "line.\n");
    line_follow_begin_calibration();

    for (uint32_t elapsed = 0u; elapsed < CALIB_MS; elapsed += CALIB_STEP_MS)
    {
        uint32_t const raw = bitpack_word_get(line_follow_read(),
                                              LINE_CENTRE_SHIFT,
                                              LINE_CENTRE_WIDTH);

        lowest  = (raw < lowest) ? raw : lowest;
        highest = (raw > highest) ? raw : highest;

        if (0u == ((elapsed / CALIB_STEP_MS) % CALIB_DOT_STEPS))
        {
            (void) tm_printf((UB *) ".");  // (UB *): monitor API type.
        }

        robot_ipc_sleep_ms(CALIB_STEP_MS);
    }

    b_ok = line_follow_end_calibration();

    // (UB *): string literals to the monitor API's character type.
    (void) tm_printf((UB *) "\nCentre range %lu .. %lu, contrast %lu "
                     "(needs %lu): %s\n\n", lowest, highest,
                     highest - lowest, LINE_MIN_CONTRAST,
                     b_ok ? "PASS" : "FAIL, defaults kept");

    return (b_ok);
}


/*!
 * @brief Print every barcode, decode failure and line event since the last
 *        call.  Used by this test and by the Buddy 2 + 3 test.
 */
void
buddy3_test_report_events (void)
{
    drain_barcodes();
    drain_events();
}


/*!
 * @brief Print every barcode letter the Barcode Task has decoded.
 */
static void
drain_barcodes (void)
{
    uint8_t msg    = 0u;
    bool    b_more = true;

    for (uint32_t count = 0u; b_more && (count < DRAIN_LIMIT); count++)
    {
        uint32_t const length = robot_ipc_receive(ROBOT_IPC_MBF_NAV_CMD,
                                                  &msg, sizeof(msg),
                                                  ROBOT_IPC_POLL);

        if (length > 0u)
        {
            uint32_t const letter   = barcode_msg_letter(msg);
            uint32_t const reversed = bitpack_word_get(msg,
                                                       BARCODE_REVERSED_SHIFT,
                                                       1u);

            // The letter is below 26, so LETTER_A + letter is printable.
            // (UB *): string literals to the monitor API's character type.
            (void) tm_printf((UB *) ">>> BARCODE '%c'%s\n",
                             (letter < LETTER_COUNT) ?
                             (LETTER_A + letter) : 63u,  // 63 is '?'.
                             (0u != reversed) ? " (read backwards)" : "");
        }

        b_more = (length > 0u);
    }
}


/*!
 * @brief Print the events Buddy 3 posts for telemetry.
 *
 * @par
 * The Telemetry Task is not running, so the events would otherwise stay in
 * the buffer.  BARCODE events are skipped: drain_barcodes() prints them.
 */
static void
drain_events (void)
{
    uint8_t msg[ROBOT_IPC_EVENT_MSG_BYTES];
    bool    b_more = true;

    for (uint32_t count = 0u; b_more && (count < DRAIN_LIMIT); count++)
    {
        uint32_t const length = robot_ipc_receive(ROBOT_IPC_MBF_TLM_EVENT,
                                                  msg, sizeof(msg),
                                                  ROBOT_IPC_POLL);

        if (ROBOT_IPC_EVENT_MSG_BYTES == length)
        {
            uint32_t word = 0u;

            for (uint32_t idx = 0u; idx < ROBOT_IPC_EVENT_MSG_BYTES; idx++)
            {
                // Cast: widen each byte before shifting it into place.
                word |= ((uint32_t) msg[idx]) << (idx * BITS_PER_BYTE);
            }

            print_event(bitpack_word_get(word, 0u, TELEMETRY_EVT_TYPE_WIDTH),
                        bitpack_word_get(word, TELEMETRY_EVT_DATA_SHIFT,
                                         TELEMETRY_EVT_DATA_WIDTH));
        }

        b_more = (length > 0u);
    }
}


/*!
 * @brief Print one telemetry event in words.
 *
 * @param[in] type  TELEMETRY_EVT_* value.
 * @param[in] data  Event data.
 */
static void
print_event (uint32_t type, uint32_t data)
{
    // Every (UB *) cast below only changes the signedness of a string
    // literal's characters, as the monitor API expects.
    switch (type)
    {
        case TELEMETRY_EVT_BARCODE:
            // Already printed from the navigation message.
        break;

        case TELEMETRY_EVT_DECODE_FAIL:
        {
            uint32_t const reason = bitpack_word_get(data, 0u,
                                        BARCODE_REJECT_REASON_WIDTH);
            uint32_t const bars   = bitpack_word_get(data,
                                        BARCODE_REJECT_COUNT_SHIFT,
                                        BARCODE_REJECT_COUNT_WIDTH);

            // (UB *): string literal to the monitor API's character type.
            (void) tm_printf((UB *) "xxx BARCODE REJECTED: %s, %lu bars\n",
                             (reason <= BARCODE_REJECT_START) ?
                             gp_reject_names[reason] : "unknown", bars);
        }
        break;

        case TELEMETRY_EVT_LINE:
            // (UB *): string literal to the monitor API's character type.
            (void) tm_printf((UB *) "--- LINE %s\n",
                             (0u != data) ? "LOST" : "FOUND");
        break;

        default:
            // (UB *): string literal to the monitor API's character type.
            (void) tm_printf((UB *) "--- event %lu, data 0x%lx\n",
                             type, data);
        break;
    }
}


/*!
 * @brief Print one status line and release a JUNCTION hold.
 */
static void
print_status (void)
{
    uint32_t const            line    = line_follow_read();
    uint32_t const            barcode = sys_status_read(SYS_STATUS_BARCODE);
    line_follow_state_t const state   = line_follow_state(line);

    // (UB *): string literal to the monitor API's character type.  The
    // state enumeration has five values, so indexing the names is safe.
    (void) tm_printf((UB *) "centre %4lu  offset %5ld  left %s  barcode %s"
                     "  %-11s  reads %lu  rejects %lu  faults 0x%02lx\n",
                     bitpack_word_get(line, LINE_CENTRE_SHIFT,
                                      LINE_CENTRE_WIDTH),
                     line_follow_offset(line),
                     black_or_white(ir_hw_is_left_black()),
                     black_or_white(ir_hw_is_barcode_black()),
                     gp_line_names[state],
                     bitpack_word_get(barcode, BARCODE_READS_SHIFT,
                                      BARCODE_COUNT_WIDTH),
                     bitpack_word_get(barcode, BARCODE_REJECTS_SHIFT,
                                      BARCODE_COUNT_WIDTH),
                     robot_ipc_faults());

    if (LINE_STATE_JUNCTION == state)
    {
        // Vehicle Control would act on the junction, then release it.
        (void) tm_printf((UB *) "--- JUNCTION seen (both sensors black)\n");
        line_follow_clear_junction();
    }
}


/*!
 * @brief Word for a digital IR reading.
 *
 * @param[in] b_black  The sensor sees black.
 *
 * @return "BLACK" or "white".
 */
static char const *
black_or_white (bool b_black)
{
    return (b_black ? "BLACK" : "white");
}

/*** end of file ***/


/** @file telemetry.c
 *
 * @brief Telemetry Task: 5 Hz status frames, 1 Hz heartbeat, events and
 *        run summaries (Week 6 NFR15).
 *
 * @par
 * The lowest-priority application task.  It only formats messages and
 * queues them for the radio-owner task; a delayed frame shows up as a gap in
 * the sequence number, never as a control fault.
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
#include "tlm_codec.h"
#include "comms.h"
#include "encoder.h"
#include "barcode.h"
#include "imu.h"
#include "obstacle.h"
#include "vehicle.h"
#include "telemetry.h"

#define HEALTH_FIELD_WIDTH      8u
#define HEALTH_ERRORS_SHIFT     8u
#define HEALTH_FAULTS_SHIFT     16u
#define BITS_PER_BYTE           8u
#define BYTE_MASK               0xFFu
#define FRAMES_PER_SECOND       (1000u / (PERIOD_BASE_MS * \
                                          PERIOD_TELEMETRY_TICKS))
#define WORD_COUNT              ((uint32_t) SYS_STATUS_WORD_COUNT)  // Exact.

static uint32_t volatile g_dropped = 0u;    // Any producer task.
static uint32_t          g_seq     = 0u;    // Telemetry Task only.
static uint32_t          g_frames  = 0u;    // Telemetry Task only.
static uint64_t          g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void telemetry_task(INT start_code, void * p_ext_info);
static void send_status(void);
static void send_heartbeat(void);
static void send_events(void);
static void send_summary(void);
static void write_health(void);


/*!
 * @brief Start the Telemetry Task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
telemetry_start (void)
{
    int32_t       result = 0;
    int32_t const h_task = robot_ipc_start_task(telemetry_task,
                                                PRIO_TELEMETRY, g_stack,
                                                sizeof(g_stack));

    if (!period_tick_register(h_task, PERIOD_TELEMETRY_TICKS))
    {
        result = -1;
    }

    return (result);
}


/*!
 * @brief Report a one-off event without waiting (any task, not ISRs).
 *
 * @param[in] type  Event type.
 * @param[in] data  Event data (layout in telemetry.h), 28 bits used.
 *
 * @par
 * A full mbf_tlm_event drops the event and counts it.
 */
void
telemetry_post_event (telemetry_evt_t type, uint32_t data)
{
    uint32_t word   = 0u;
    uint8_t  msg[ROBOT_IPC_EVENT_MSG_BYTES];
    bool     b_sent = false;

    // The event enumeration is below 16, so the cast is exact.
    word = bitpack_word_set(word, 0u, TELEMETRY_EVT_TYPE_WIDTH,
                            (uint32_t) type);
    word = bitpack_word_set(word, TELEMETRY_EVT_DATA_SHIFT,
                            TELEMETRY_EVT_DATA_WIDTH, data);

    for (uint32_t idx = 0u; idx < ROBOT_IPC_EVENT_MSG_BYTES; idx++)
    {
        // Little-endian; each byte is masked to 8 bits.
        msg[idx] = (uint8_t) ((word >> (idx * BITS_PER_BYTE)) & BYTE_MASK);
    }

    b_sent = robot_ipc_send(ROBOT_IPC_MBF_TLM_EVENT, msg, sizeof(msg),
                            ROBOT_IPC_POLL);

    if (!b_sent)
    {
        uint32_t intsts = 0u;

        DI(intsts);
        g_dropped++;
        EI(intsts);
    }
}


/*!
 * @brief Events dropped because mbf_tlm_event was full.
 *
 * @return Drop count since boot.
 */
uint32_t
telemetry_dropped_events (void)
{
    return (g_dropped);
}


/*!
 * @brief Telemetry Task, released every 200 ms.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
telemetry_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        (void) period_tick_wait();  // Missed periods are counted centrally.

        g_frames++;
        write_health();
        send_status();

        if (0u == (g_frames % HEARTBEAT_EVERY_FRAMES))
        {
            send_heartbeat();
        }

        send_events();

        if (0u != robot_ipc_flag_wait(ROBOT_IPC_FLAG_COMM,
                                      ROBOT_COMM_SUMMARY_REQ, ROBOT_IPC_POLL))
        {
            send_summary();
        }
    }
}


/*!
 * @brief Queue one TELEMETRY frame of every status word.
 */
static void
send_status (void)
{
    uint32_t words[SYS_STATUS_WORD_COUNT];
    uint8_t  frame[TLM_CODEC_MAX_FRAME];
    uint32_t length = 0u;

    for (uint32_t idx = 0u; idx < WORD_COUNT; idx++)
    {
        // idx is below the enumeration count, so the cast is exact.
        words[idx] = sys_status_read((sys_status_word_t) idx);
    }

    length = tlm_codec_telemetry(g_seq, words, frame, sizeof(frame));
    g_seq++;
    (void) comms_queue(TLM_TOPIC_TELEMETRY, frame, length);  // Drops counted.
}


/*!
 * @brief Queue one HEARTBEAT frame.
 */
static void
send_heartbeat (void)
{
    uint32_t const vehicle = sys_status_read(SYS_STATUS_VEHICLE);
    uint8_t        frame[TLM_CODEC_MAX_FRAME];
    uint32_t const length  = tlm_codec_heartbeat(
                                 g_seq, g_frames / FRAMES_PER_SECOND,
                                 bitpack_word_get(vehicle,
                                                  VEHICLE_STATE_SHIFT,
                                                  VEHICLE_STATE_WIDTH),
                                 robot_ipc_faults(), frame, sizeof(frame));

    g_seq++;
    (void) comms_queue(TLM_TOPIC_HEARTBEAT, frame, length);  // Counted.
}


/*!
 * @brief Queue an EVENT frame for every pending event.
 */
static void
send_events (void)
{
    uint8_t  msg[ROBOT_IPC_EVENT_MSG_BYTES];
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_TLM_EVENT, msg,
                                        sizeof(msg), ROBOT_IPC_POLL);

    while (ROBOT_IPC_EVENT_MSG_BYTES == length)
    {
        uint8_t  frame[TLM_CODEC_MAX_FRAME];
        uint32_t word = 0u;

        for (uint32_t idx = 0u; idx < ROBOT_IPC_EVENT_MSG_BYTES; idx++)
        {
            word |= ((uint32_t) msg[idx]) << (idx * BITS_PER_BYTE);  // Wide.
        }

        length = tlm_codec_event(g_seq, word, frame, sizeof(frame));
        g_seq++;
        (void) comms_queue(TLM_TOPIC_EVENT, frame, length);  // Counted.
        length = robot_ipc_receive(ROBOT_IPC_MBF_TLM_EVENT, msg, sizeof(msg),
                                   ROBOT_IPC_POLL);
    }
}


/*!
 * @brief Answer a summary request (FR14) with the run summary (FR5).
 */
static void
send_summary (void)
{
    uint32_t const      hump     = sys_status_read(SYS_STATUS_HUMP);
    uint32_t const      barcode  = sys_status_read(SYS_STATUS_BARCODE);
    uint32_t const      obstacle = sys_status_read(SYS_STATUS_OBSTACLE);
    uint32_t const      odometry = sys_status_read(SYS_STATUS_ODOMETRY);
    uint8_t             frame[TLM_CODEC_MAX_FRAME];
    uint32_t            length   = 0u;
    tlm_codec_summary_t summary;

    summary.max_hump_mm = bitpack_word_get(hump, IMU_HUMP_MAX_SHIFT,
                                           IMU_HUMP_MM_WIDTH);
    summary.humps       = bitpack_word_get(hump, IMU_HUMP_COUNT_SHIFT,
                                           IMU_HUMP_COUNT_WIDTH);
    summary.barcodes    = bitpack_word_get(barcode, BARCODE_READS_SHIFT,
                                           BARCODE_COUNT_WIDTH);
    summary.rejects     = bitpack_word_get(barcode, BARCODE_REJECTS_SHIFT,
                                           BARCODE_COUNT_WIDTH);
    summary.obstacles   = bitpack_word_get(obstacle, OBSTACLE_PROFILES_SHIFT,
                                           OBSTACLE_PROFILES_WIDTH);
    summary.distance_cm = bitpack_word_get(odometry, 0u,
                                           ENCODER_DISTANCE_WIDTH);
    summary.run_time_s  = g_frames / FRAMES_PER_SECOND;
    summary.faults      = robot_ipc_faults();

    length = tlm_codec_summary(&summary, frame, sizeof(frame));
    (void) comms_queue(TLM_TOPIC_SUMMARY, frame, length);  // Counted.
}


/*!
 * @brief Publish the SYS_STATUS_HEALTH word (this task is its owner).
 */
static void
write_health (void)
{
    uint32_t word = 0u;

    word = bitpack_word_set(word, 0u, HEALTH_FIELD_WIDTH,
                            bitpack_saturate(period_tick_missed_total(),
                                             HEALTH_FIELD_WIDTH));
    word = bitpack_word_set(word, HEALTH_ERRORS_SHIFT, HEALTH_FIELD_WIDTH,
                            bitpack_saturate(robot_ipc_error_count(),
                                             HEALTH_FIELD_WIDTH));
    word = bitpack_word_set(word, HEALTH_FAULTS_SHIFT, HEALTH_FIELD_WIDTH,
                            robot_ipc_faults());
    sys_status_write(SYS_STATUS_HEALTH, word);
}

/*** end of file ***/


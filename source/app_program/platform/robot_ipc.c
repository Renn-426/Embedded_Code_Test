/** @file robot_ipc.c
 *
 * @brief micro T-Kernel objects shared between the subsystems.
 *
 * @par
 * Reference: micro T-Kernel 3.0 Specification (IEEE 2050-2018), chapters on
 * event flags, message buffers and mutexes.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_ipc.h"

// micro T-Kernel stores each message with a 4-byte header and rounds the
// body up to 4 bytes (kernel/tkernel/messagebuf.h).
#define MBF_HEADER_BYTES        4u
#define MBF_ROUND_BYTES         4u
#define MBF_SLOT_BYTES(SIZE)    ((((SIZE) + (MBF_ROUND_BYTES - 1u)) /  \
                                  MBF_ROUND_BYTES) * MBF_ROUND_BYTES + \
                                 MBF_HEADER_BYTES)

// Messages each buffer can hold before a sender sees it full.
#define NAV_MSG_SLOTS           4u
#define OBS_MSG_SLOTS           2u
#define CTRL_MSG_SLOTS          4u
#define MOTION_MSG_SLOTS        4u
#define EVENT_MSG_SLOTS         16u
#define MQTT_MSG_SLOTS          8u

#define NAV_BUF_BYTES     (NAV_MSG_SLOTS *                                    \
                           MBF_SLOT_BYTES(ROBOT_IPC_NAV_MSG_BYTES))
#define OBS_BUF_BYTES     (OBS_MSG_SLOTS *                                    \
                           MBF_SLOT_BYTES(ROBOT_IPC_OBS_MSG_BYTES))
#define CTRL_BUF_BYTES    (CTRL_MSG_SLOTS *                                   \
                           MBF_SLOT_BYTES(ROBOT_IPC_CTRL_MSG_BYTES))
#define MOTION_BUF_BYTES  (MOTION_MSG_SLOTS *                                 \
                           MBF_SLOT_BYTES(ROBOT_IPC_MOTION_MSG_BYTES))
#define EVENT_BUF_BYTES   (EVENT_MSG_SLOTS *                                  \
                           MBF_SLOT_BYTES(ROBOT_IPC_EVENT_MSG_BYTES))
#define MQTT_BUF_BYTES    (MQTT_MSG_SLOTS *                                   \
                           MBF_SLOT_BYTES(ROBOT_IPC_MQTT_MSG_BYTES))

#define WORD_BYTES              4u

// Object counts as unsigned loop bounds.  The casts are exact: each
// enumeration count is a small positive constant.
#define FLAG_COUNT              ((uint32_t) ROBOT_IPC_FLAG_COUNT)
#define MBF_COUNT               ((uint32_t) ROBOT_IPC_MBF_COUNT)
#define LOCK_COUNT              ((uint32_t) ROBOT_IPC_LOCK_COUNT)

// Kernel object handles, created once by robot_ipc_create().
static ID       gh_flags[ROBOT_IPC_FLAG_COUNT];
static ID       gh_mbfs[ROBOT_IPC_MBF_COUNT];
static ID       gh_locks[ROBOT_IPC_LOCK_COUNT];

// Failed kernel calls since boot (diagnostic, reported in telemetry).
static uint32_t volatile g_error_count = 0u;

// Static message buffer storage (TA_USERBUF), word aligned.
static uint32_t g_nav_buf[NAV_BUF_BYTES / WORD_BYTES];
static uint32_t g_obs_buf[OBS_BUF_BYTES / WORD_BYTES];
static uint32_t g_ctrl_buf[CTRL_BUF_BYTES / WORD_BYTES];
static uint32_t g_motion_buf[MOTION_BUF_BYTES / WORD_BYTES];
static uint32_t g_event_buf[EVENT_BUF_BYTES / WORD_BYTES];
static uint32_t g_mqtt_buf[MQTT_BUF_BYTES / WORD_BYTES];

// Storage and largest message of each buffer, in robot_ipc_mbf_t order.
static uint32_t * const gp_mbf_storage[ROBOT_IPC_MBF_COUNT] =
{
    g_nav_buf, g_obs_buf, g_ctrl_buf, g_motion_buf, g_event_buf, g_mqtt_buf
};
static uint32_t const g_mbf_bytes[ROBOT_IPC_MBF_COUNT] =
{
    NAV_BUF_BYTES, OBS_BUF_BYTES, CTRL_BUF_BYTES, MOTION_BUF_BYTES,
    EVENT_BUF_BYTES, MQTT_BUF_BYTES
};
static uint32_t const g_mbf_max_msg[ROBOT_IPC_MBF_COUNT] =
{
    ROBOT_IPC_NAV_MSG_BYTES, ROBOT_IPC_OBS_MSG_BYTES,
    ROBOT_IPC_CTRL_MSG_BYTES, ROBOT_IPC_MOTION_MSG_BYTES,
    ROBOT_IPC_EVENT_MSG_BYTES, ROBOT_IPC_MQTT_MSG_BYTES
};

static void   count_error(ER ercd);
static TMO    to_kernel_timeout(int32_t timeout_ms);


/*!
 * @brief Create every shared kernel object.
 *
 * @return 0 on success, or the first negative kernel error code.
 *
 * @par
 * Called once from usermain() before any subsystem task is started.
 */
int32_t
robot_ipc_create (void)
{
    int32_t result = 0;

    // Step 1: event flags.
    for (uint32_t idx = 0u; idx < FLAG_COUNT; idx++)
    {
        T_CFLG const cflg =
        {
            .exinf = NULL, .flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0u
        };

        gh_flags[idx] = tk_cre_flg(&cflg);

        if (gh_flags[idx] < E_OK)
        {
            result = gh_flags[idx];
        }
    }

    // Step 2: message buffers on static storage.
    for (uint32_t idx = 0u; idx < MBF_COUNT; idx++)
    {
        // Buffer and message sizes are compile-time constants below 2^15,
        // so the conversions to the kernel's signed size types are exact.
        T_CMBF const cmbf =
        {
            .exinf  = NULL,
            .mbfatr = TA_TFIFO | TA_USERBUF,
            .bufsz  = (SZ) g_mbf_bytes[idx],      // At most 352 bytes.
            .maxmsz = (INT) g_mbf_max_msg[idx],   // At most 40 bytes.
            .bufptr = gp_mbf_storage[idx]
        };

        gh_mbfs[idx] = tk_cre_mbf(&cmbf);

        if (gh_mbfs[idx] < E_OK)
        {
            result = gh_mbfs[idx];
        }
    }

    // Step 3: mutexes with priority inheritance.
    for (uint32_t idx = 0u; idx < LOCK_COUNT; idx++)
    {
        T_CMTX const cmtx =
        {
            .exinf = NULL, .mtxatr = TA_TFIFO | TA_INHERIT, .ceilpri = 1
        };

        gh_locks[idx] = tk_cre_mtx(&cmtx);

        if (gh_locks[idx] < E_OK)
        {
            result = gh_locks[idx];
        }
    }

    return (result);
}


/*!
 * @brief Create and start a task on a caller-owned static stack.
 *
 * @param[in] p_entry      Task function (name ends in _task).
 * @param[in] priority     Kernel priority, 1 is the highest.
 * @param[in] p_stack      Stack storage, 8-byte aligned.
 * @param[in] stack_bytes  Size of the stack storage in bytes.
 *
 * @return The task handle (> 0), or a negative kernel error code.
 */
int32_t
robot_ipc_start_task (robot_ipc_task_fn_t p_entry, int32_t priority,
                      uint64_t * p_stack, uint32_t stack_bytes)
{
    // Priorities are 1..32 and stacks at most a few KiB, so both
    // conversions to the kernel's signed types are exact.
    T_CTSK const ctsk =
    {
        .exinf   = NULL,
        .tskatr  = TA_HLNG | TA_RNG0 | TA_USERBUF,
        .task    = p_entry,
        .itskpri = (PRI) priority,       // 1..32.
        .stksz   = (SZ) stack_bytes,     // A few KiB.
        .bufptr  = p_stack
    };
    ID h_task = tk_cre_tsk(&ctsk);

    if (h_task > E_OK)
    {
        ER const ercd = tk_sta_tsk(h_task, 0);

        if (E_OK != ercd)
        {
            h_task = ercd;
        }
    }

    count_error(h_task);

    return (h_task);
}


/*!
 * @brief Set bits in an event flag (allowed from interrupt handlers).
 *
 * @param[in] flag  Which flag.
 * @param[in] bits  Bits to set.
 */
void
robot_ipc_flag_set (robot_ipc_flag_t flag, uint32_t bits)
{
    if (flag < ROBOT_IPC_FLAG_COUNT)
    {
        count_error(tk_set_flg(gh_flags[flag], bits));
    }
}


/*!
 * @brief Clear bits in an event flag.
 *
 * @param[in] flag  Which flag.
 * @param[in] bits  Bits to clear.
 */
void
robot_ipc_flag_clear (robot_ipc_flag_t flag, uint32_t bits)
{
    if (flag < ROBOT_IPC_FLAG_COUNT)
    {
        // tk_clr_flg keeps the bits that are 1 in its pattern argument.
        count_error(tk_clr_flg(gh_flags[flag], ~bits));
    }
}


/*!
 * @brief Wait until any of the given bits is set, then clear those bits.
 *
 * @param[in] flag        Which flag.
 * @param[in] bits        Bits of interest.
 * @param[in] timeout_ms  ROBOT_IPC_POLL, ROBOT_IPC_FOREVER or milliseconds.
 *
 * @return The bits of interest that were set (and are now cleared), or 0 on
 *         timeout.
 */
uint32_t
robot_ipc_flag_wait (robot_ipc_flag_t flag, uint32_t bits,
                     int32_t timeout_ms)
{
    uint32_t matched = 0u;

    if (flag < ROBOT_IPC_FLAG_COUNT)
    {
        UINT     pattern = 0u;
        ER const ercd    = tk_wai_flg(gh_flags[flag], bits,
                                      TWF_ORW | TWF_BITCLR, &pattern,
                                      to_kernel_timeout(timeout_ms));

        if (E_OK == ercd)
        {
            matched = pattern & bits;
        }
        else if (E_TMOUT != ercd)
        {
            count_error(ercd);
        }
        else
        {
            // Timeout: nothing happened, which is not an error.
        }
    }

    return (matched);
}


/*!
 * @brief Read an event flag without waiting or clearing.
 *
 * @param[in] flag  Which flag.
 *
 * @return The current bit pattern.
 */
uint32_t
robot_ipc_flag_peek (robot_ipc_flag_t flag)
{
    uint32_t pattern = 0u;

    if (flag < ROBOT_IPC_FLAG_COUNT)
    {
        T_RFLG   rflg = { .exinf = NULL, .wtsk = 0, .flgptn = 0u };
        ER const ercd = tk_ref_flg(gh_flags[flag], &rflg);

        if (E_OK == ercd)
        {
            pattern = rflg.flgptn;
        }

        count_error(ercd);
    }

    return (pattern);
}


/*!
 * @brief Record one or more fault conditions and notify Vehicle Control.
 *
 * @param[in] fault_bits  ROBOT_FAULT_* bits to set.
 */
void
robot_ipc_raise_fault (uint32_t fault_bits)
{
    robot_ipc_flag_set(ROBOT_IPC_FLAG_FAULT, fault_bits);
    robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_FAULT);
}


/*!
 * @brief Current fault bitmask.
 *
 * @return ROBOT_FAULT_* bits that are set.
 */
uint32_t
robot_ipc_faults (void)
{
    return (robot_ipc_flag_peek(ROBOT_IPC_FLAG_FAULT));
}


/*!
 * @brief Send a message to a message buffer.
 *
 * @param[in] mbf         Which buffer.
 * @param[in] p_msg       Message bytes.
 * @param[in] size_bytes  Message length; must not exceed the buffer maximum.
 * @param[in] timeout_ms  ROBOT_IPC_POLL (the telemetry rule: never block a
 *                        producer), ROBOT_IPC_FOREVER or milliseconds.
 *
 * @return true if the message was queued; false if the buffer was full or
 *         the message invalid (the message is then dropped).
 */
bool
robot_ipc_send (robot_ipc_mbf_t mbf, uint8_t const * p_msg,
                uint32_t size_bytes, int32_t timeout_ms)
{
    bool b_sent = false;

    if ((mbf < ROBOT_IPC_MBF_COUNT) && (NULL != p_msg) &&
        (size_bytes > 0u) && (size_bytes <= g_mbf_max_msg[mbf]))
    {
        // size_bytes is at most ROBOT_IPC_MQTT_MSG_BYTES: exact as INT.
        ER const ercd = tk_snd_mbf(gh_mbfs[mbf], p_msg, (INT) size_bytes,
                                   to_kernel_timeout(timeout_ms));

        b_sent = (E_OK == ercd);

        if (E_TMOUT != ercd)
        {
            count_error(ercd);
        }
    }

    return (b_sent);
}


/*!
 * @brief Receive a message from a message buffer.
 *
 * @param[in]  mbf         Which buffer.
 * @param[out] p_msg       Destination.
 * @param[in]  capacity    Destination size; must be at least the buffer's
 *                         largest message or nothing is received.
 * @param[in]  timeout_ms  ROBOT_IPC_POLL, ROBOT_IPC_FOREVER or milliseconds.
 *
 * @return The message length in bytes, or 0 if none arrived in time.
 */
uint32_t
robot_ipc_receive (robot_ipc_mbf_t mbf, uint8_t * p_msg, uint32_t capacity,
                   int32_t timeout_ms)
{
    uint32_t length = 0u;

    if ((mbf < ROBOT_IPC_MBF_COUNT) && (NULL != p_msg) &&
        (capacity >= g_mbf_max_msg[mbf]))
    {
        INT const size = tk_rcv_mbf(gh_mbfs[mbf], p_msg,
                                    to_kernel_timeout(timeout_ms));

        if (size > 0)
        {
            // A positive message size never exceeds 40: exact conversion.
            length = (uint32_t) size;
        }
        else if (E_TMOUT != size)
        {
            count_error(size);
        }
        else
        {
            // Timeout: no message, which is not an error.
        }
    }

    return (length);
}


/*!
 * @brief Lock a shared-state mutex (waits forever).
 *
 * @param[in] lock  Which mutex.
 */
void
robot_ipc_lock (robot_ipc_lock_t lock)
{
    if (lock < ROBOT_IPC_LOCK_COUNT)
    {
        count_error(tk_loc_mtx(gh_locks[lock], TMO_FEVR));
    }
}


/*!
 * @brief Unlock a shared-state mutex.
 *
 * @param[in] lock  Which mutex.
 */
void
robot_ipc_unlock (robot_ipc_lock_t lock)
{
    if (lock < ROBOT_IPC_LOCK_COUNT)
    {
        count_error(tk_unl_mtx(gh_locks[lock]));
    }
}


/*!
 * @brief Put the calling task to sleep.
 *
 * @param[in] delay_ms  Milliseconds (rounded up to the 10 ms kernel tick).
 */
void
robot_ipc_sleep_ms (uint32_t delay_ms)
{
    // tk_dly_tsk takes an unsigned RELTIM, so no conversion is needed.
    count_error(tk_dly_tsk(delay_ms));
}


/*!
 * @brief Number of failed kernel calls since boot.
 *
 * @return Error count.
 */
uint32_t
robot_ipc_error_count (void)
{
    return (g_error_count);
}


/*!
 * @brief Count a failed kernel call.
 *
 * @param[in] ercd  Kernel return code; values >= E_OK are successes.
 */
static void
count_error (ER ercd)
{
    if (ercd < E_OK)
    {
        uint32_t intsts = 0u;

        // Interrupts off around the increment: several tasks share it.
        DI(intsts);
        g_error_count++;
        EI(intsts);
    }
}


/*!
 * @brief Convert a robot timeout to the kernel's TMO type.
 *
 * @param[in] timeout_ms  ROBOT_IPC_POLL, ROBOT_IPC_FOREVER or milliseconds.
 *
 * @return TMO_POL, TMO_FEVR or the millisecond value.
 */
static TMO
to_kernel_timeout (int32_t timeout_ms)
{
    TMO tmout = timeout_ms;

    if (ROBOT_IPC_FOREVER == timeout_ms)
    {
        tmout = TMO_FEVR;
    }
    else if (timeout_ms < 0)
    {
        tmout = TMO_POL;
    }
    else
    {
        // Non-negative: used as given.
    }

    return (tmout);
}

/*** end of file ***/


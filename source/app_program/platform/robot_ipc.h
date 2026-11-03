/** @file robot_ipc.h
 *
 * @brief micro T-Kernel objects shared between the subsystems (Week 6
 *        Section 4 task interaction design).
 *
 * @par
 * Event flags:  flg_vehicle_evt (events into Vehicle Control), flg_fault
 * (the shared fault bitmask of Change Requirement 1), flg_comm (link and
 * summary requests into Telemetry) and flg_obstacle (SCAN_REQ).  Setting and
 * clearing bits goes through tk_set_flg/tk_clr_flg, which the kernel performs
 * atomically, so no task ever does a read-modify-write on a shared bitmask.
 *
 * @par
 * Message buffers:  mbf_nav_cmd, mbf_obs_profile, mbf_ctrl_cmd,
 * mbf_motion_cmd, mbf_tlm_event and mbf_mqtt_tx.  Every buffer and task stack
 * uses statically allocated storage (TA_USERBUF) so the RAM budget is visible
 * in the linker map.
 *
 * @par
 * Kernel object IDs never leave this module; subsystems use the functions
 * below.  A failed kernel call is counted (robot_ipc_error_count) rather than
 * ignored.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef ROBOT_IPC_H
#define ROBOT_IPC_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

// Timeouts accepted by the wait functions, in milliseconds.
#define ROBOT_IPC_POLL              0
#define ROBOT_IPC_FOREVER           (-1)

// flg_vehicle_evt bits: events that can change the mission state.
#define ROBOT_EVT_JUNCTION          0x0001u
#define ROBOT_EVT_LINE_LOST         0x0002u
#define ROBOT_EVT_LINE_FOUND        0x0004u
#define ROBOT_EVT_OBSTACLE          0x0008u
#define ROBOT_EVT_IMPACT            0x0010u
#define ROBOT_EVT_MOVE_DONE         0x0020u
#define ROBOT_EVT_STALL             0x0040u
#define ROBOT_EVT_NAV_CMD           0x0080u
#define ROBOT_EVT_OBS_PROFILE       0x0100u
#define ROBOT_EVT_CTRL_CMD          0x0200u
#define ROBOT_EVT_FAULT             0x0400u
#define ROBOT_EVT_ECHO_FAULT        0x0800u
#define ROBOT_EVT_ALL               0x0FFFu

// flg_fault bits: one bit per fault condition (Change Requirement 1).
#define ROBOT_FAULT_MOTOR_STALL     0x01u
#define ROBOT_FAULT_ENCODER         0x02u
#define ROBOT_FAULT_IMU             0x04u
#define ROBOT_FAULT_NO_ECHO         0x08u
#define ROBOT_FAULT_WIFI_LOSS       0x10u
#define ROBOT_FAULT_IR_CALIB        0x20u
#define ROBOT_FAULT_BARCODE_OVF     0x40u
#define ROBOT_FAULT_MQTT_LOSS       0x80u
#define ROBOT_FAULT_WIDTH           8u

// flg_comm bits.
#define ROBOT_COMM_SUMMARY_REQ      0x01u
#define ROBOT_COMM_LINK_UP          0x02u
#define ROBOT_COMM_LINK_DOWN        0x04u

// flg_obstacle bits.
#define ROBOT_OBS_SCAN_REQ          0x01u

// Largest message, in bytes, accepted by each message buffer.
#define ROBOT_IPC_NAV_MSG_BYTES     1u
#define ROBOT_IPC_OBS_MSG_BYTES     6u
#define ROBOT_IPC_CTRL_MSG_BYTES    4u
#define ROBOT_IPC_MOTION_MSG_BYTES  4u
#define ROBOT_IPC_EVENT_MSG_BYTES   4u
#define ROBOT_IPC_MQTT_MSG_BYTES    40u

/*!
 * @brief Event flags.
 */
typedef enum
{
    ROBOT_IPC_FLAG_VEHICLE = 0,
    ROBOT_IPC_FLAG_FAULT,
    ROBOT_IPC_FLAG_COMM,
    ROBOT_IPC_FLAG_OBSTACLE,
    ROBOT_IPC_FLAG_COUNT
} robot_ipc_flag_t;

/*!
 * @brief Message buffers.
 */
typedef enum
{
    ROBOT_IPC_MBF_NAV_CMD = 0,
    ROBOT_IPC_MBF_OBS_PROFILE,
    ROBOT_IPC_MBF_CTRL_CMD,
    ROBOT_IPC_MBF_MOTION_CMD,
    ROBOT_IPC_MBF_TLM_EVENT,
    ROBOT_IPC_MBF_MQTT_TX,
    ROBOT_IPC_MBF_COUNT
} robot_ipc_mbf_t;

/*!
 * @brief Mutexes (priority inheritance) for multi-word shared state.
 */
typedef enum
{
    ROBOT_IPC_LOCK_ENC_STATE = 0,
    ROBOT_IPC_LOCK_COUNT
} robot_ipc_lock_t;

/*!
 * @brief Task entry point, as micro T-Kernel calls it.
 */
typedef void (* robot_ipc_task_fn_t)(INT start_code, void * p_ext_info);

int32_t  robot_ipc_create(void);
int32_t  robot_ipc_start_task(robot_ipc_task_fn_t p_entry, int32_t priority,
                              uint64_t * p_stack, uint32_t stack_bytes);

void     robot_ipc_flag_set(robot_ipc_flag_t flag, uint32_t bits);
void     robot_ipc_flag_clear(robot_ipc_flag_t flag, uint32_t bits);
uint32_t robot_ipc_flag_wait(robot_ipc_flag_t flag, uint32_t bits,
                             int32_t timeout_ms);
uint32_t robot_ipc_flag_peek(robot_ipc_flag_t flag);

void     robot_ipc_raise_fault(uint32_t fault_bits);
uint32_t robot_ipc_faults(void);

bool     robot_ipc_send(robot_ipc_mbf_t mbf, uint8_t const * p_msg,
                        uint32_t size_bytes, int32_t timeout_ms);
uint32_t robot_ipc_receive(robot_ipc_mbf_t mbf, uint8_t * p_msg,
                           uint32_t capacity, int32_t timeout_ms);

void     robot_ipc_lock(robot_ipc_lock_t lock);
void     robot_ipc_unlock(robot_ipc_lock_t lock);

void     robot_ipc_sleep_ms(uint32_t delay_ms);
uint32_t robot_ipc_error_count(void);

#endif /* ROBOT_IPC_H */

/*** end of file ***/


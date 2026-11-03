/** @file imu.c
 *
 * @brief IMU Task: tilt, heading, motion class, humps and impacts.
 *
 * @par
 * The LSM303DLHC has no gyroscope, so pitch comes from the direction of
 * gravity (asin of the forward acceleration over the total), smoothed with
 * a first-order low-pass filter, and turn rate comes from the wheel speed
 * difference (FR15, assumption A6).  Heading is the level magnetometer
 * angle; it is not tilt-compensated and the motors disturb it, which is why
 * it is reported but not used for control.
 *
 * @par
 * CALIBRATING averages IMU_CALIB_SAMPLES readings while the robot is still
 * to remove the mounting pitch and record 1 g.  Three consecutive I2C
 * failures enter SENSOR FAULT: the data is marked invalid, ROBOT_FAULT_IMU
 * is raised and the sensor is re-initialised once a second.
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
#include "telemetry.h"
#include "encoder.h"
#include "lsm303_hw.h"
#include "hump.h"
#include "imu.h"

#define MS_PER_PERIOD           (PERIOD_BASE_MS * PERIOD_IMU_TICKS)
#define PERIODS_PER_SECOND      (1000u / MS_PER_PERIOD)
#define MAG_EVERY_PERIODS       3u
#define FILTER_DIVISOR          4
#define ONE_G_MG                1000
#define MDEG_PER_HALF_DEG       500
#define MDEG_PER_DEG            1000
#define MDEG_PER_RAD            57296
#define IMPACT_UNIT_MG          16
#define MAX_COUNT_4BIT          15u
#define STILL_MM_S              20
#define IMPACT_DATA_WIDTH       8u
#define FULL_TURN_DEG           360

/*!
 * @brief IMU Task working state.
 */
typedef struct
{
    bool        b_sensor_ok;
    uint32_t    errors;
    uint32_t    retry_countdown;
    uint32_t    calib_count;
    int32_t     calib_pitch_sum;
    int32_t     calib_mag_sum;
    int32_t     pitch_bias_mdeg;
    int32_t     one_g_mg;
    int32_t     pitch_mdeg;
    int32_t     heading_mdeg;
    int32_t     prev_speed_mm_s;
    uint32_t    mag_countdown;
    uint32_t    cooldown_ms;
    uint32_t    impacts;
    uint32_t    last_hump_mm;
    imu_class_t motion_class;
    hump_t      hump;
} imu_ctx_t;

static imu_ctx_t g_ctx;
static uint64_t  g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void    imu_task(INT start_code, void * p_ext_info);
static bool    sample(int32_t * p_accel);
static int32_t raw_pitch(int32_t const * p_accel, int32_t magnitude);
static void    check_impact(int32_t magnitude);
static void    read_heading(void);
static void    report_init(bool b_ok);
static void    classify(encoder_state_t const * p_enc);
static void    track_hump(encoder_state_t const * p_enc, bool b_valid);
static void    publish(bool b_valid);


/*!
 * @brief Start the IMU Task (the sensor is initialised by the task).
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
imu_start (void)
{
    int32_t result = 0;
    int32_t h_task = 0;

    g_ctx.b_sensor_ok     = false;
    g_ctx.retry_countdown = 0u;
    g_ctx.one_g_mg        = ONE_G_MG;
    g_ctx.motion_class    = IMU_CLASS_STATIONARY;
    hump_init(&g_ctx.hump, HUMP_ENTER_MDEG, HUMP_EXIT_MDEG, HUMP_HOLD_MS);

    h_task = robot_ipc_start_task(imu_task, PRIO_IMU, g_stack,
                                  sizeof(g_stack));

    if (!period_tick_register(h_task, PERIOD_IMU_TICKS))
    {
        result = -1;
    }

    return (result);
}


/*!
 * @brief IMU Task, released every 20 ms.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
imu_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        encoder_state_t enc;
        bool            b_valid = false;

        (void) period_tick_wait();  // Missed periods are counted centrally.

        if (!g_ctx.b_sensor_ok)
        {
            // SENSOR FAULT: re-initialise once a second.
            if (0u == g_ctx.retry_countdown)
            {
                g_ctx.b_sensor_ok     = lsm303_hw_init();
                g_ctx.errors          = 0u;
                g_ctx.calib_count     = 0u;
                g_ctx.calib_pitch_sum = 0;
                g_ctx.calib_mag_sum   = 0;
                g_ctx.retry_countdown = PERIODS_PER_SECOND;
                report_init(g_ctx.b_sensor_ok);
            }

            g_ctx.retry_countdown--;
        }
        else
        {
            int32_t accel[LSM303_HW_AXES] = { 0, 0, 0 };

            b_valid = sample(accel);
        }

        encoder_get_state(&enc);
        classify(&enc);
        track_hump(&enc, b_valid);
        publish(b_valid);
    }
}


/*!
 * @brief Read the sensor and update pitch, heading and impact detection.
 *
 * @param[out] p_accel  Acceleration in milli-g.
 *
 * @return true if this sample is valid (sensor read and calibrated).
 */
static bool
sample (int32_t * p_accel)
{
    bool b_valid = false;

    if (!lsm303_hw_read_accel(p_accel))
    {
        g_ctx.errors++;

        if (g_ctx.errors >= IMU_FAULT_ERRORS)
        {
            g_ctx.b_sensor_ok     = false;
            g_ctx.retry_countdown = PERIODS_PER_SECOND;
            robot_ipc_raise_fault(ROBOT_FAULT_IMU);
        }
    }
    else
    {
        // Each axis is within +/-2048 mg, so the squares sum below 2^24
        // and the conversions are exact.
        uint32_t const sum_sq    = (uint32_t) ((p_accel[0] * p_accel[0]) +
                                               (p_accel[1] * p_accel[1]) +
                                               (p_accel[2] * p_accel[2]));
        int32_t const  magnitude = (int32_t) fix_math_isqrt(sum_sq);  // <4096.
        int32_t const  pitch     = raw_pitch(p_accel, magnitude);

        g_ctx.errors = 0u;

        if (g_ctx.calib_count < IMU_CALIB_SAMPLES)
        {
            // CALIBRATING: the robot is held still in READY.
            g_ctx.calib_pitch_sum += pitch;
            g_ctx.calib_mag_sum   += magnitude;
            g_ctx.calib_count++;
            g_ctx.pitch_bias_mdeg  = g_ctx.calib_pitch_sum /
                                     (int32_t) g_ctx.calib_count;  // <= 50.
            g_ctx.one_g_mg         = g_ctx.calib_mag_sum /
                                     (int32_t) g_ctx.calib_count;  // <= 50.
            g_ctx.pitch_mdeg       = 0;
        }
        else
        {
            g_ctx.pitch_mdeg += ((pitch - g_ctx.pitch_bias_mdeg) -
                                 g_ctx.pitch_mdeg) / FILTER_DIVISOR;
            check_impact(magnitude);
            b_valid = true;
        }

        read_heading();
    }

    return (b_valid);
}


/*!
 * @brief Pitch from the gravity direction.
 *
 * @param[in] p_accel    Acceleration in milli-g.
 * @param[in] magnitude  Total acceleration in milli-g.
 *
 * @return Pitch in millidegrees, positive nose-up, mounting bias included.
 */
static int32_t
raw_pitch (int32_t const * p_accel, int32_t magnitude)
{
    int32_t const forward = p_accel[IMU_FORWARD_AXIS] * IMU_FORWARD_SIGN;
    int32_t       ratio   = 0;

    if (magnitude > 0)
    {
        ratio = (forward * FIX_MATH_Q15_ONE) / magnitude;
    }

    return (fix_math_asin_mdeg(ratio));
}


/*!
 * @brief Detect a sudden impact from the total acceleration.
 *
 * @param[in] magnitude  Total acceleration in milli-g.
 *
 * @par
 * IMPACT is raised when the total departs from 1 g by more than IMPACT_MG,
 * followed by a cooldown so one collision is reported once.
 */
static void
check_impact (int32_t magnitude)
{
    int32_t const deviation = fix_math_abs(magnitude - g_ctx.one_g_mg);

    if (g_ctx.cooldown_ms > 0u)
    {
        g_ctx.cooldown_ms = (g_ctx.cooldown_ms > MS_PER_PERIOD) ?
                            (g_ctx.cooldown_ms - MS_PER_PERIOD) : 0u;
    }
    else if (deviation > (int32_t) IMPACT_MG)  // 1500: exact conversion.
    {
        // deviation is positive here, so the conversion is exact.
        uint32_t const peak = (uint32_t) (deviation / IMPACT_UNIT_MG);

        g_ctx.cooldown_ms = IMPACT_COOLDOWN_MS;
        g_ctx.impacts     = (g_ctx.impacts < MAX_COUNT_4BIT) ?
                            (g_ctx.impacts + 1u) : MAX_COUNT_4BIT;
        robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_IMPACT);
        telemetry_post_event(TELEMETRY_EVT_IMPACT,
                             bitpack_saturate(peak, IMPACT_DATA_WIDTH));
    }
    else
    {
        // No impact.
    }
}


/*!
 * @brief Raise or clear ROBOT_FAULT_IMU after an initialisation attempt.
 *
 * @param[in] b_ok  true if the sensor answered and accepted its settings.
 */
static void
report_init (bool b_ok)
{
    if (b_ok)
    {
        robot_ipc_flag_clear(ROBOT_IPC_FLAG_FAULT, ROBOT_FAULT_IMU);
    }
    else
    {
        robot_ipc_raise_fault(ROBOT_FAULT_IMU);
    }
}


/*!
 * @brief Update the heading from the magnetometer every third sample.
 */
static void
read_heading (void)
{
    if (0u == g_ctx.mag_countdown)
    {
        int32_t field[LSM303_HW_AXES] = { 0, 0, 0 };

        g_ctx.mag_countdown = MAG_EVERY_PERIODS;

        if (lsm303_hw_read_mag(field))
        {
            g_ctx.heading_mdeg = fix_math_atan2_mdeg(field[LSM303_HW_AXIS_Y],
                                                     field[LSM303_HW_AXIS_X]);
        }
    }

    g_ctx.mag_countdown--;
}


/*!
 * @brief Classify the vehicle's motion.
 *
 * @param[in] p_enc  Latest encoder state.
 */
static void
classify (encoder_state_t const * p_enc)
{
    int32_t const      speed = (p_enc->left_speed_mm_s +
                                p_enc->right_speed_mm_s) / 2;
    int32_t const      accel = ((speed - g_ctx.prev_speed_mm_s) *
                                MDEG_PER_DEG) /
                               (int32_t) MS_PER_PERIOD;  // 20 ms: exact.
    int32_t const      turn  = ((p_enc->right_speed_mm_s -
                                 p_enc->left_speed_mm_s) * MDEG_PER_RAD) /
                               (int32_t) TRACK_WIDTH_MM;  // 120: exact.
    hump_state_t const hump  = hump_state(&g_ctx.hump);

    g_ctx.prev_speed_mm_s = speed;

    if (g_ctx.cooldown_ms > 0u)
    {
        g_ctx.motion_class = IMU_CLASS_IMPACT;
    }
    else if (HUMP_STATE_LEVEL != hump)
    {
        g_ctx.motion_class = (HUMP_STATE_CLIMBING == hump) ?
                             IMU_CLASS_CLIMBING : IMU_CLASS_DESCENDING;
    }
    else if (fix_math_abs(turn) > TURNING_MDEG_S)
    {
        g_ctx.motion_class = IMU_CLASS_TURNING;
    }
    else if (fix_math_abs(accel) > ACCEL_EVENT_MM_S2)
    {
        g_ctx.motion_class = (accel > 0) ? IMU_CLASS_ACCELERATING :
                                           IMU_CLASS_DECELERATING;
    }
    else
    {
        g_ctx.motion_class = (fix_math_abs(speed) < STILL_MM_S) ?
                             IMU_CLASS_STATIONARY : IMU_CLASS_CRUISING;
    }
}


/*!
 * @brief Feed the hump detector and report completed humps.
 *
 * @param[in] p_enc    Latest encoder state.
 * @param[in] b_valid  Whether this IMU sample is valid.
 */
static void
track_hump (encoder_state_t const * p_enc, bool b_valid)
{
    uint32_t height_mm = 0u;

    if (hump_update(&g_ctx.hump, g_ctx.pitch_mdeg, p_enc->distance_mm,
                    b_valid, MS_PER_PERIOD, &height_mm))
    {
        uint32_t data = 0u;

        g_ctx.last_hump_mm = height_mm;
        data = bitpack_word_set(data, IMU_HUMP_LAST_SHIFT, IMU_HUMP_MM_WIDTH,
                                bitpack_saturate(height_mm,
                                                 IMU_HUMP_MM_WIDTH));
        data = bitpack_word_set(data, IMU_HUMP_MAX_SHIFT, IMU_HUMP_MM_WIDTH,
                                bitpack_saturate(hump_max_mm(&g_ctx.hump),
                                                 IMU_HUMP_MM_WIDTH));
        telemetry_post_event(TELEMETRY_EVT_HUMP, data);
    }
}


/*!
 * @brief Publish the IMU and HUMP status words.
 *
 * @param[in] b_valid  Whether the latest sample was valid.
 */
static void
publish (bool b_valid)
{
    int32_t const  heading_deg = fix_math_wrap_mdeg(g_ctx.heading_mdeg) /
                                 MDEG_PER_DEG;

    // Heading in [0, 360) after the wrap correction: exact conversion.
    uint32_t const heading     = (uint32_t) ((heading_deg < 0) ?
                                             (heading_deg + FULL_TURN_DEG) :
                                             heading_deg);
    uint32_t       imu         = 0u;
    uint32_t       hump        = 0u;

    // The enumeration values are below 8, so the casts are exact.
    imu  = bitpack_word_set(imu, IMU_PITCH_SHIFT, IMU_PITCH_WIDTH,
                            bitpack_from_signed(g_ctx.pitch_mdeg /
                                                MDEG_PER_HALF_DEG,
                                                IMU_PITCH_WIDTH));
    imu  = bitpack_word_set(imu, IMU_HEADING_SHIFT, IMU_HEADING_WIDTH,
                            heading);
    imu  = bitpack_word_set(imu, IMU_CLASS_SHIFT, IMU_CLASS_WIDTH,
                            (uint32_t) g_ctx.motion_class);  // Enum < 8.
    imu  = bitpack_word_set(imu, IMU_HUMP_SHIFT, IMU_HUMP_WIDTH,
                            (uint32_t) hump_state(&g_ctx.hump));  // Enum < 4.
    imu  = bitpack_word_set(imu, IMU_VALID_SHIFT, 1u, b_valid ? 1u : 0u);
    imu  = bitpack_word_set(imu, IMU_IMPACTS_SHIFT, IMU_IMPACTS_WIDTH,
                            g_ctx.impacts);
    hump = bitpack_word_set(hump, IMU_HUMP_LAST_SHIFT, IMU_HUMP_MM_WIDTH,
                            bitpack_saturate(g_ctx.last_hump_mm,
                                             IMU_HUMP_MM_WIDTH));
    hump = bitpack_word_set(hump, IMU_HUMP_MAX_SHIFT, IMU_HUMP_MM_WIDTH,
                            bitpack_saturate(hump_max_mm(&g_ctx.hump),
                                             IMU_HUMP_MM_WIDTH));
    hump = bitpack_word_set(hump, IMU_HUMP_COUNT_SHIFT, IMU_HUMP_COUNT_WIDTH,
                            bitpack_saturate(hump_count(&g_ctx.hump),
                                             IMU_HUMP_COUNT_WIDTH));
    sys_status_write(SYS_STATUS_IMU, imu);
    sys_status_write(SYS_STATUS_HUMP, hump);
}

/*** end of file ***/


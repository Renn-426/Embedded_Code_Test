/** @file obs_profile.c
 *
 * @brief Obstacle profiling and avoidance planning from a servo scan.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "robot_cfg.h"
#include "bitpack.h"
#include "fix_math.h"
#include "obs_profile.h"

#define OBSTACLE_DEPTH_MM       150
#define BYPASS_DEPTH_MM         400
#define MDEG_PER_DEG            1000
#define MM_PER_CM               10u
#define ANGLE_STEP_DEG          5u
#define FIELD_CM_WIDTH          8u
#define FIELD_ANGLE_WIDTH       6u
#define FIELD_PLAN_WIDTH        3u

// Signed copies of the tuning constants; each is below 2^16, so the casts
// are exact.
#define CLEAR_MAX_MM            ((int32_t) CLEARANCE_MAX_MM)
#define HALF_WIDTH_MM           ((int32_t) (VEHICLE_WIDTH_MM / 2u))
#define MARGIN_MM               ((int32_t) CLEARANCE_MARGIN_MM)

/*!
 * @brief A scan point in robot coordinates.
 */
typedef struct
{
    int32_t     x_mm;           // Lateral, positive to the left.
    int32_t     y_mm;           // Forward.
    bool        b_echo;
} position_t;

/*!
 * @brief Lateral extent of the obstacle.
 */
typedef struct
{
    uint32_t    closest;        // Index of the closest point.
    uint32_t    run_low;        // First index of the obstacle run.
    uint32_t    run_high;       // Last index of the obstacle run.
    int32_t     closest_y_mm;
    int32_t     left_x_mm;
    int32_t     right_x_mm;
} extent_t;

static position_t         to_position(obs_profile_point_t const * p_point);
static uint32_t           sort_by_angle(obs_profile_point_t const * p_points,
                                        uint32_t count,
                                        obs_profile_point_t * p_sorted);
static bool               belongs(obs_profile_point_t const * p_point,
                                  uint32_t closest_mm);
static bool               find_extent(obs_profile_point_t const * p_points,
                                      uint32_t count, extent_t * p_extent);
static int32_t            side_clearance(obs_profile_point_t const * p_points,
                                         uint32_t count,
                                         extent_t const * p_extent,
                                         bool b_left);
static obs_profile_plan_t plan_route(extent_t const * p_extent,
                                     int32_t left_clear, int32_t right_clear);
static uint32_t           to_cm_field(uint32_t value_mm);


/*!
 * @brief Profile the obstacle and choose an avoidance plan.
 *
 * @param[in]  p_points   Scan samples (sensor faults already removed).
 * @param[in]  count      Number of samples.
 * @param[out] p_profile  Result.
 */
void
obs_profile_compute (obs_profile_point_t const * p_points, uint32_t count,
                     obs_profile_t * p_profile)
{
    obs_profile_point_t sorted[OBS_PROFILE_MAX_POINTS];
    extent_t            extent = { 0u, 0u, 0u, 0, 0, 0 };
    uint32_t const      used   = sort_by_angle(p_points, count, sorted);

    p_profile->distance_mm    = 0u;
    p_profile->angle_deg      = SERVO_CENTRE_DEG;
    p_profile->width_mm       = 0u;
    p_profile->left_clear_mm  = CLEARANCE_MAX_MM;
    p_profile->right_clear_mm = CLEARANCE_MAX_MM;
    p_profile->plan           = OBS_PLAN_STOP;
    p_profile->b_valid        = (used > 0u);

    if ((p_profile->b_valid) && (!find_extent(sorted, used, &extent)))
    {
        // Nothing within profiling range: the path is clear.
        p_profile->plan = OBS_PLAN_CONTINUE;
    }
    else if (p_profile->b_valid)
    {
        int32_t const left  = side_clearance(sorted, used, &extent, true);
        int32_t const right = side_clearance(sorted, used, &extent, false);

        // Every value below is non-negative and below 2^16, so each
        // conversion to unsigned is exact.
        p_profile->distance_mm    = sorted[extent.closest].distance_mm;
        p_profile->angle_deg      = sorted[extent.closest].angle_deg;
        p_profile->width_mm       = (uint32_t) (extent.left_x_mm -  // >= 0.
                                                extent.right_x_mm);
        p_profile->left_clear_mm  = (uint32_t) left;    // 0..1000.
        p_profile->right_clear_mm = (uint32_t) right;   // 0..1000.
        p_profile->plan           = plan_route(&extent, left, right);
    }
    else
    {
        // No usable samples at all: stop.
    }
}


/*!
 * @brief Pack a profile into its 6-byte message.
 *
 * @param[in]  p_profile  Profile.
 * @param[out] p_msg      OBS_PROFILE_MSG_BYTES bytes.
 */
void
obs_profile_pack (obs_profile_t const * p_profile, uint8_t * p_msg)
{
    bitpack_t pack;

    bitpack_init(&pack, p_msg, OBS_PROFILE_MSG_BYTES);
    bitpack_put(&pack, to_cm_field(p_profile->distance_mm), FIELD_CM_WIDTH);
    bitpack_put(&pack, to_cm_field(p_profile->width_mm), FIELD_CM_WIDTH);
    bitpack_put(&pack, p_profile->angle_deg / ANGLE_STEP_DEG,
                FIELD_ANGLE_WIDTH);
    bitpack_put(&pack, to_cm_field(p_profile->left_clear_mm),
                FIELD_CM_WIDTH);
    bitpack_put(&pack, to_cm_field(p_profile->right_clear_mm),
                FIELD_CM_WIDTH);

    // The plan enumeration is below 8, so the cast is exact.
    bitpack_put(&pack, (uint32_t) p_profile->plan, FIELD_PLAN_WIDTH);
    bitpack_put(&pack, p_profile->b_valid ? 1u : 0u, 1u);
}


/*!
 * @brief Unpack a 6-byte profile message.
 *
 * @param[in]  p_msg      OBS_PROFILE_MSG_BYTES bytes (not modified).
 * @param[out] p_profile  Profile, distances rounded down to whole cm.
 */
void
obs_profile_unpack (uint8_t const * p_msg, obs_profile_t * p_profile)
{
    bitpack_t      pack;
    uint32_t       plan = 0u;

    // bitpack_init clears its buffer, so read from a local copy.
    uint8_t        copy[OBS_PROFILE_MSG_BYTES];

    bitpack_init(&pack, copy, OBS_PROFILE_MSG_BYTES);

    for (uint32_t idx = 0u; idx < OBS_PROFILE_MSG_BYTES; idx++)
    {
        copy[idx] = p_msg[idx];
    }

    p_profile->distance_mm    = bitpack_get(&pack, FIELD_CM_WIDTH) *
                                MM_PER_CM;
    p_profile->width_mm       = bitpack_get(&pack, FIELD_CM_WIDTH) *
                                MM_PER_CM;
    p_profile->angle_deg      = bitpack_get(&pack, FIELD_ANGLE_WIDTH) *
                                ANGLE_STEP_DEG;
    p_profile->left_clear_mm  = bitpack_get(&pack, FIELD_CM_WIDTH) *
                                MM_PER_CM;
    p_profile->right_clear_mm = bitpack_get(&pack, FIELD_CM_WIDTH) *
                                MM_PER_CM;
    plan                      = bitpack_get(&pack, FIELD_PLAN_WIDTH);
    p_profile->b_valid        = (1u == bitpack_get(&pack, 1u));

    switch (plan)
    {
        case OBS_PLAN_CONTINUE:
            p_profile->plan = OBS_PLAN_CONTINUE;
        break;

        case OBS_PLAN_PASS_LEFT:
            p_profile->plan = OBS_PLAN_PASS_LEFT;
        break;

        case OBS_PLAN_PASS_RIGHT:
            p_profile->plan = OBS_PLAN_PASS_RIGHT;
        break;

        case OBS_PLAN_REVERSE:
            p_profile->plan = OBS_PLAN_REVERSE;
        break;

        default:
            p_profile->plan = OBS_PLAN_STOP;
        break;
    }
}


/*!
 * @brief Convert a scan point to robot coordinates.
 *
 * @param[in] p_point  Scan point.
 *
 * @return Position; meaningless if the point has no echo.
 */
static position_t
to_position (obs_profile_point_t const * p_point)
{
    // Angles are at most 180 and distances at most SONAR_MAX_MM, so both
    // conversions to int32_t are exact.
    int32_t const angle_mdeg = (int32_t) p_point->angle_deg * MDEG_PER_DEG;
    int32_t const distance   = (int32_t) p_point->distance_mm;
    int32_t const lateral    = (distance * fix_math_cos_q15(angle_mdeg)) /
                               FIX_MATH_Q15_ONE;
    position_t    pos;

    pos.x_mm   = (0u != SERVO_LEFT_IS_HIGH) ? -lateral : lateral;
    pos.y_mm   = (distance * fix_math_sin_q15(angle_mdeg)) / FIX_MATH_Q15_ONE;
    pos.b_echo = p_point->b_echo;

    return (pos);
}


/*!
 * @brief Copy the scan sorted by servo angle (insertion sort).
 *
 * @param[in]  p_points  Scan samples in any order.
 * @param[in]  count     Number of samples.
 * @param[out] p_sorted  OBS_PROFILE_MAX_POINTS slots.
 *
 * @return Number of samples copied (at most OBS_PROFILE_MAX_POINTS).
 */
static uint32_t
sort_by_angle (obs_profile_point_t const * p_points, uint32_t count,
               obs_profile_point_t * p_sorted)
{
    uint32_t const used = (count < OBS_PROFILE_MAX_POINTS) ?
                          count : OBS_PROFILE_MAX_POINTS;

    for (uint32_t idx = 0u; idx < used; idx++)
    {
        obs_profile_point_t const item = p_points[idx];
        uint32_t          slot = idx;

        while ((slot > 0u) &&
               (p_sorted[slot - 1u].angle_deg > item.angle_deg))
        {
            p_sorted[slot] = p_sorted[slot - 1u];
            slot--;
        }

        p_sorted[slot] = item;
    }

    return (used);
}


/*!
 * @brief Report whether a point continues the obstacle surface.
 *
 * @param[in] p_point     Scan point.
 * @param[in] closest_mm  Range of the obstacle's closest point.
 *
 * @return true if it has an echo no more than OBSTACLE_DEPTH_MM further.
 */
static bool
belongs (obs_profile_point_t const * p_point, uint32_t closest_mm)
{
    // OBSTACLE_DEPTH_MM is a small positive constant: exact conversion.
    return ((p_point->b_echo) &&
            (p_point->distance_mm <= (closest_mm +
                                      (uint32_t) OBSTACLE_DEPTH_MM)));
}


/*!
 * @brief Find the closest in-range point and the obstacle around it.
 *
 * @param[in]  p_points  Scan samples sorted by angle.
 * @param[in]  count     Number of samples.
 * @param[out] p_extent  Obstacle run and lateral edges.
 *
 * @return false if no point lies within OBSTACLE_PROFILE_MM.
 *
 * @par
 * The obstacle is the run of neighbouring samples, in angle order, that
 * continue its surface.  A gap (no echo, or a much longer range) ends the
 * run, so a wall beside the obstacle is not mistaken for part of it.
 */
static bool
find_extent (obs_profile_point_t const * p_points, uint32_t count,
             extent_t * p_extent)
{
    bool     b_found = false;
    uint32_t best    = UINT32_MAX;

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        if ((p_points[idx].b_echo) &&
            (p_points[idx].distance_mm < OBSTACLE_PROFILE_MM) &&
            (p_points[idx].distance_mm < best))
        {
            best              = p_points[idx].distance_mm;
            p_extent->closest = idx;
            b_found           = true;
        }
    }

    if (b_found)
    {
        position_t const near = to_position(&p_points[p_extent->closest]);

        p_extent->run_low      = p_extent->closest;
        p_extent->run_high     = p_extent->closest;
        p_extent->closest_y_mm = near.y_mm;
        p_extent->left_x_mm    = near.x_mm;
        p_extent->right_x_mm   = near.x_mm;

        while ((p_extent->run_low > 0u) &&
               (belongs(&p_points[p_extent->run_low - 1u], best)))
        {
            p_extent->run_low--;
        }

        while (((p_extent->run_high + 1u) < count) &&
               (belongs(&p_points[p_extent->run_high + 1u], best)))
        {
            p_extent->run_high++;
        }

        for (uint32_t idx = p_extent->run_low; idx <= p_extent->run_high;
             idx++)
        {
            position_t const pos = to_position(&p_points[idx]);

            p_extent->left_x_mm  = (pos.x_mm > p_extent->left_x_mm) ?
                                   pos.x_mm : p_extent->left_x_mm;
            p_extent->right_x_mm = (pos.x_mm < p_extent->right_x_mm) ?
                                   pos.x_mm : p_extent->right_x_mm;
        }
    }

    return (b_found);
}


/*!
 * @brief Free lateral space beyond one edge of the obstacle.
 *
 * @param[in] p_points  Scan samples sorted by angle.
 * @param[in] count     Number of samples.
 * @param[in] p_extent  Obstacle run and edges.
 * @param[in] b_left    true for the left side.
 *
 * @return Clearance in mm, 0 to CLEARANCE_MAX_MM.
 *
 * @par
 * Only echoes outside the obstacle run, on the requested side, and close
 * enough to matter for the bypass (BYPASS_DEPTH_MM beyond the closest
 * point) limit the clearance.
 */
static int32_t
side_clearance (obs_profile_point_t const * p_points, uint32_t count,
                extent_t const * p_extent, bool b_left)
{
    bool const b_high_side = (b_left == (0u != SERVO_LEFT_IS_HIGH));
    int32_t    clearance   = CLEAR_MAX_MM;

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        position_t const pos      = to_position(&p_points[idx]);
        bool const       b_beyond = b_high_side ?
                                    (idx > p_extent->run_high) :
                                    (idx < p_extent->run_low);
        int32_t const    gap      = b_left ?
                                    (pos.x_mm - p_extent->left_x_mm) :
                                    (p_extent->right_x_mm - pos.x_mm);

        if (b_beyond && (pos.b_echo) &&
            (pos.y_mm <= (p_extent->closest_y_mm + BYPASS_DEPTH_MM)) &&
            (gap < clearance))
        {
            clearance = (gap > 0) ? gap : 0;
        }
    }

    return (clearance);
}


/*!
 * @brief Choose the side that fits the vehicle and deviates least.
 *
 * @param[in] p_extent     Obstacle extent.
 * @param[in] left_clear   Clearance on the left.
 * @param[in] right_clear  Clearance on the right.
 *
 * @return The avoidance plan.
 */
static obs_profile_plan_t
plan_route (extent_t const * p_extent, int32_t left_clear,
            int32_t right_clear)
{
    int32_t const      needed     = (2 * HALF_WIDTH_MM) + MARGIN_MM;
    int32_t const      corridor   = HALF_WIDTH_MM + MARGIN_MM;
    int32_t const      left_move  = p_extent->left_x_mm + corridor;
    int32_t const      right_move = corridor - p_extent->right_x_mm;
    bool const         b_left_ok  = (left_clear >= needed);
    bool const         b_right_ok = (right_clear >= needed);
    obs_profile_plan_t plan       = OBS_PLAN_REVERSE;

    if ((p_extent->right_x_mm > corridor) ||
        (p_extent->left_x_mm < -corridor))
    {
        // The obstacle is beside the path, not on it.
        plan = OBS_PLAN_CONTINUE;
    }
    else if (b_left_ok && b_right_ok)
    {
        plan = (left_move <= right_move) ? OBS_PLAN_PASS_LEFT :
                                           OBS_PLAN_PASS_RIGHT;
    }
    else if (b_left_ok)
    {
        plan = OBS_PLAN_PASS_LEFT;
    }
    else if (b_right_ok)
    {
        plan = OBS_PLAN_PASS_RIGHT;
    }
    else
    {
        plan = OBS_PLAN_REVERSE;
    }

    return (plan);
}


/*!
 * @brief Convert millimetres to an 8-bit centimetre field.
 *
 * @param[in] value_mm  Value in millimetres.
 *
 * @return Centimetres, saturated at 255.
 */
static uint32_t
to_cm_field (uint32_t value_mm)
{
    return (bitpack_saturate(value_mm / MM_PER_CM, FIELD_CM_WIDTH));
}

/*** end of file ***/


/** @file fix_math.c
 *
 * @brief Integer (fixed-point) maths helpers.
 *
 * @par
 * Sine uses a 5-degree Q15 table with linear interpolation (error below
 * 0.001).  atan2 uses the rational approximation
 * atan(z) = (pi/4) z + 0.273 z (1 - z) for 0 <= z <= 1, from S. Rajan et al.,
 * "Efficient approximations for the arctangent function", IEEE Signal
 * Processing Magazine, May 2006 (maximum error about 0.22 degrees).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>

#include "fix_math.h"

#define TABLE_STEP_MDEG         5000
#define TABLE_LAST_INDEX        18
#define QUARTER_TURN_MDEG       90000
#define THREE_QUARTER_MDEG      270000
#define ATAN_LINEAR_MDEG        45000
#define ATAN_CURVE_MDEG         15642
#define Q15_SHIFT               15u
#define ATAN_INPUT_LIMIT        65535

// sin(k * 5 degrees) in Q15 for k = 0..18.
static int32_t const g_sine_table[TABLE_LAST_INDEX + 1] =
{
    0, 2856, 5690, 8481, 11207, 13848, 16384, 18795, 21063, 23170,
    25102, 26842, 28378, 29698, 30792, 31651, 32270, 32643, 32768
};

static int32_t sine_first_quadrant(int32_t angle_mdeg);
static int32_t atan_unit_mdeg(int32_t ratio_q15);


/*!
 * @brief Integer square root.
 *
 * @param[in] value  Radicand.
 *
 * @return floor(sqrt(value)).
 */
uint32_t
fix_math_isqrt (uint32_t value)
{
    uint32_t remainder = value;
    uint32_t root      = 0u;
    uint32_t place     = 1u << 30;

    while (place > remainder)
    {
        place >>= 2;
    }

    while (0u != place)
    {
        if (remainder >= (root + place))
        {
            remainder -= (root + place);
            root       = (root >> 1) + place;
        }
        else
        {
            root >>= 1;
        }

        place >>= 2;
    }

    return (root);
}


/*!
 * @brief Absolute value that cannot overflow.
 *
 * @param[in] value  Signed value.
 *
 * @return |value|, with INT32_MIN mapped to INT32_MAX.
 */
int32_t
fix_math_abs (int32_t value)
{
    int32_t result = value;

    if (INT32_MIN == value)
    {
        result = INT32_MAX;
    }
    else if (value < 0)
    {
        result = -value;
    }
    else
    {
        // Already non-negative.
    }

    return (result);
}


/*!
 * @brief Limit a value to a closed range.
 *
 * @param[in] value    Value to limit.
 * @param[in] lowest   Smallest allowed value.
 * @param[in] highest  Largest allowed value (must be >= lowest).
 *
 * @return The clamped value.
 */
int32_t
fix_math_clamp (int32_t value, int32_t lowest, int32_t highest)
{
    int32_t result = value;

    if (value < lowest)
    {
        result = lowest;
    }
    else if (value > highest)
    {
        result = highest;
    }
    else
    {
        // Inside the range.
    }

    return (result);
}


/*!
 * @brief Sine of an angle.
 *
 * @param[in] angle_mdeg  Angle in millidegrees (any value).
 *
 * @return sin(angle) in Q15.
 */
int32_t
fix_math_sin_q15 (int32_t angle_mdeg)
{
    int32_t angle  = angle_mdeg % FIX_MATH_MDEG_FULL;
    int32_t result = 0;

    if (angle < 0)
    {
        angle += FIX_MATH_MDEG_FULL;
    }

    if (angle <= QUARTER_TURN_MDEG)
    {
        result = sine_first_quadrant(angle);
    }
    else if (angle <= FIX_MATH_MDEG_HALF)
    {
        result = sine_first_quadrant(FIX_MATH_MDEG_HALF - angle);
    }
    else if (angle <= THREE_QUARTER_MDEG)
    {
        result = -sine_first_quadrant(angle - FIX_MATH_MDEG_HALF);
    }
    else
    {
        result = -sine_first_quadrant(FIX_MATH_MDEG_FULL - angle);
    }

    return (result);
}


/*!
 * @brief Cosine of an angle.
 *
 * @param[in] angle_mdeg  Angle in millidegrees (any value).
 *
 * @return cos(angle) in Q15.
 */
int32_t
fix_math_cos_q15 (int32_t angle_mdeg)
{
    return (fix_math_sin_q15((angle_mdeg % FIX_MATH_MDEG_FULL) +
                             QUARTER_TURN_MDEG));
}


/*!
 * @brief Four-quadrant arctangent.
 *
 * @param[in] y_value  Opposite component (any integer unit).
 * @param[in] x_value  Adjacent component (same unit as y_value).
 *
 * @return atan2(y, x) in millidegrees, in the range (-180000, 180000].
 *         Returns 0 when both inputs are 0.
 */
int32_t
fix_math_atan2_mdeg (int32_t y_value, int32_t x_value)
{
    int32_t const abs_y  = fix_math_clamp(fix_math_abs(y_value), 0,
                                          ATAN_INPUT_LIMIT);
    int32_t const abs_x  = fix_math_clamp(fix_math_abs(x_value), 0,
                                          ATAN_INPUT_LIMIT);
    int32_t       result = 0;

    if ((0 != abs_x) || (0 != abs_y))
    {
        // Fold into the first octant so the ratio is at most 1.0.
        if (abs_y <= abs_x)
        {
            result = atan_unit_mdeg((abs_y * FIX_MATH_Q15_ONE) / abs_x);
        }
        else
        {
            result = QUARTER_TURN_MDEG -
                     atan_unit_mdeg((abs_x * FIX_MATH_Q15_ONE) / abs_y);
        }

        // Unfold into the correct quadrant.
        if (x_value < 0)
        {
            result = FIX_MATH_MDEG_HALF - result;
        }

        if (y_value < 0)
        {
            result = -result;
        }
    }

    return (result);
}


/*!
 * @brief Arcsine of a ratio.
 *
 * @param[in] ratio_q15  Sine value in Q15; clamped to [-1.0, 1.0].
 *
 * @return asin(ratio) in millidegrees, in the range [-90000, 90000].
 */
int32_t
fix_math_asin_mdeg (int32_t ratio_q15)
{
    int32_t const  ratio  = fix_math_clamp(ratio_q15, -FIX_MATH_Q15_ONE,
                                           FIX_MATH_Q15_ONE);
    int32_t const  square = ratio * ratio;  // At most 2^30: no overflow.

    // One minus the square is in [0, 2^30], so the casts are exact.
    uint32_t const one_sq = (uint32_t) (FIX_MATH_Q15_ONE * FIX_MATH_Q15_ONE);
    uint32_t const cosine = fix_math_isqrt(one_sq - (uint32_t) square);

    // cosine is at most 32768, so it converts to int32_t exactly.
    return (fix_math_atan2_mdeg(ratio, (int32_t) cosine));
}


/*!
 * @brief Wrap an angle into (-180000, 180000] millidegrees.
 *
 * @param[in] angle_mdeg  Angle in millidegrees.
 *
 * @return The equivalent angle in the half-open range.
 */
int32_t
fix_math_wrap_mdeg (int32_t angle_mdeg)
{
    int32_t angle = angle_mdeg % FIX_MATH_MDEG_FULL;

    if (angle > FIX_MATH_MDEG_HALF)
    {
        angle -= FIX_MATH_MDEG_FULL;
    }
    else if (angle <= -FIX_MATH_MDEG_HALF)
    {
        angle += FIX_MATH_MDEG_FULL;
    }
    else
    {
        // Already in range.
    }

    return (angle);
}


/*!
 * @brief Interpolated table sine for 0 to 90 degrees.
 *
 * @param[in] angle_mdeg  Angle in [0, 90000] millidegrees.
 *
 * @return sin(angle) in Q15.
 */
static int32_t
sine_first_quadrant (int32_t angle_mdeg)
{
    int32_t const index    = angle_mdeg / TABLE_STEP_MDEG;
    int32_t const fraction = angle_mdeg % TABLE_STEP_MDEG;
    int32_t       result   = g_sine_table[TABLE_LAST_INDEX];

    if (index < TABLE_LAST_INDEX)
    {
        int32_t const low  = g_sine_table[index];
        int32_t const high = g_sine_table[index + 1];

        result = low + (((high - low) * fraction) / TABLE_STEP_MDEG);
    }

    return (result);
}


/*!
 * @brief Arctangent of a ratio in [0, 1].
 *
 * @param[in] ratio_q15  Ratio in Q15, 0 to 32768.
 *
 * @return atan(ratio) in millidegrees, 0 to 45000.
 */
static int32_t
atan_unit_mdeg (int32_t ratio_q15)
{
    int32_t const linear = (ATAN_LINEAR_MDEG * ratio_q15) / FIX_MATH_Q15_ONE;

    // z (1 - z) in Q15, kept below 2^13 before scaling to avoid overflow.
    int32_t const bend   = (ratio_q15 * (FIX_MATH_Q15_ONE - ratio_q15)) /
                           FIX_MATH_Q15_ONE;

    return (linear + ((ATAN_CURVE_MDEG * bend) / FIX_MATH_Q15_ONE));
}

/*** end of file ***/


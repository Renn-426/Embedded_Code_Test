/** @file fix_math.h
 *
 * @brief Integer (fixed-point) maths helpers.
 *
 * @par
 * The RP2040's Cortex-M0+ cores have no floating-point unit, and Barr-C rule
 * 5.4.a asks for floating point to be avoided.  Angles are therefore carried
 * in millidegrees (mdeg) and trigonometric ratios in Q15 (1.0 = 32768).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef FIX_MATH_H
#define FIX_MATH_H

#include <stdint.h>

// Q15 representation of 1.0.
#define FIX_MATH_Q15_ONE        32768

// Millidegrees in a full and a half turn.
#define FIX_MATH_MDEG_FULL      360000
#define FIX_MATH_MDEG_HALF      180000

uint32_t fix_math_isqrt(uint32_t value);
int32_t  fix_math_abs(int32_t value);
int32_t  fix_math_clamp(int32_t value, int32_t lowest, int32_t highest);
int32_t  fix_math_sin_q15(int32_t angle_mdeg);
int32_t  fix_math_cos_q15(int32_t angle_mdeg);
int32_t  fix_math_atan2_mdeg(int32_t y_value, int32_t x_value);
int32_t  fix_math_asin_mdeg(int32_t ratio_q15);
int32_t  fix_math_wrap_mdeg(int32_t angle_mdeg);

#endif /* FIX_MATH_H */

/*** end of file ***/


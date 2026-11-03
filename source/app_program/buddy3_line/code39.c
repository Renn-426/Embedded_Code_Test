/** @file code39.c
 *
 * @brief Code 39 decoder for single-letter navigation barcodes "*X*".
 *
 * @par
 * Reference: ISO/IEC 16388:2007, Code 39 bar code symbology specification.
 * The pattern table lists each character's 9 elements from first bar to last
 * bar, most significant bit first, 1 = wide.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "code39.h"

#define WIDE_BARS               2u      // Wide bars in a letter or '*'.
#define WIDE_SPACES             1u      // Wide spaces in a letter or '*'.
#define CHAR_STRIDE             (CODE39_ELEMENTS_PER_CHAR + 1u)
#define TABLE_SIZE              44u
#define RATIO_NUMERATOR         13u     // Wide must be >= 1.3 x narrow.
#define RATIO_DENOMINATOR       10u     // (Was 1.5; blur widens narrow bars.)
#define FRAME_SYMBOL            ((uint8_t) '*')   // ASCII 42: exact.
#define FIRST_LETTER            ((uint8_t) 'A')   // ASCII 65: exact.
#define LAST_LETTER             ((uint8_t) 'Z')   // ASCII 90: exact.
#define START_CHAR              0u
#define DATA_CHAR               1u
#define STOP_CHAR               2u

/*!
 * @brief One Code 39 character.
 */
typedef struct
{
    uint16_t    pattern;
    uint8_t     symbol;
} code39_entry_t;

static code39_entry_t const g_table[TABLE_SIZE] =
{
    { 0x034u, '0' },
    { 0x121u, '1' },
    { 0x061u, '2' },
    { 0x160u, '3' },
    { 0x031u, '4' },
    { 0x130u, '5' },
    { 0x070u, '6' },
    { 0x025u, '7' },
    { 0x124u, '8' },
    { 0x064u, '9' },
    { 0x109u, 'A' },
    { 0x049u, 'B' },
    { 0x148u, 'C' },
    { 0x019u, 'D' },
    { 0x118u, 'E' },
    { 0x058u, 'F' },
    { 0x00Du, 'G' },
    { 0x10Cu, 'H' },
    { 0x04Cu, 'I' },
    { 0x01Cu, 'J' },
    { 0x103u, 'K' },
    { 0x043u, 'L' },
    { 0x142u, 'M' },
    { 0x013u, 'N' },
    { 0x112u, 'O' },
    { 0x052u, 'P' },
    { 0x007u, 'Q' },
    { 0x106u, 'R' },
    { 0x046u, 'S' },
    { 0x016u, 'T' },
    { 0x181u, 'U' },
    { 0x0C1u, 'V' },
    { 0x1C0u, 'W' },
    { 0x091u, 'X' },
    { 0x190u, 'Y' },
    { 0x0D0u, 'Z' },
    { 0x085u, '-' },
    { 0x184u, '.' },
    { 0x0C4u, ' ' },
    { 0x094u, '*' },
    { 0x0A8u, '$' },
    { 0x0A2u, '/' },
    { 0x08Au, '+' },
    { 0x02Au, '%' }
};

static code39_result_t decode_frame(uint32_t const * p_widths,
                                    uint8_t * p_symbol);
static code39_result_t decode_reversed(uint32_t const * p_widths,
                                       uint8_t * p_symbol);
static code39_result_t classify(uint32_t const * p_elements,
                               uint32_t * p_pattern, uint32_t * p_min_wide);


/*!
 * @brief Find and decode a 29-element frame in a capture.
 *
 * @param[in]  p_widths      Element widths in capture order, starting with
 *                           the first bar (any time unit).
 * @param[in]  count         Number of elements.
 * @param[out] p_symbol      ASCII data letter 'A'..'Z' on success.
 * @param[out] p_b_reversed  true if the barcode was read backwards.
 *
 * @return CODE39_OK, or the reason the capture was rejected.
 */
code39_result_t
code39_decode (uint32_t const * p_widths, uint32_t count, uint8_t * p_symbol,
               bool * p_b_reversed)
{
    code39_result_t result = CODE39_BAD_COUNT;

    *p_b_reversed = false;

    // Stray edges before or after the barcode (such as the edge of its paper)
    // only add elements, so try the frame at each position.  The reason
    // reported on failure is that of the first position.
    for (uint32_t start = 0u; (CODE39_OK != result) &&
         ((start + CODE39_FRAME_ELEMENTS) <= count); start++)
    {
        code39_result_t attempt = decode_frame(&p_widths[start], p_symbol);

        if ((CODE39_OK != attempt) &&
            (CODE39_OK == decode_reversed(&p_widths[start], p_symbol)))
        {
            attempt       = CODE39_OK;
            *p_b_reversed = true;
        }

        if ((0u == start) || (CODE39_OK == attempt))
        {
            result = attempt;
        }
    }

    return (result);
}


/*!
 * @brief Look up the character for a 9-bit wide/narrow pattern.
 *
 * @param[in] pattern  Element pattern, first element in bit 8, 1 = wide.
 *
 * @return ASCII symbol, or 0 if the pattern is not a Code 39 character.
 */
uint8_t
code39_symbol_for (uint32_t pattern)
{
    uint8_t symbol = 0u;

    for (uint32_t idx = 0u; idx < TABLE_SIZE; idx++)
    {
        if (pattern == g_table[idx].pattern)
        {
            symbol = g_table[idx].symbol;
        }
    }

    return (symbol);
}


/*!
 * @brief Decode one direction of a frame.
 *
 * @param[in]  p_widths  29 element widths.
 * @param[out] p_symbol  Data letter on success.
 *
 * @return CODE39_OK or the rejection reason.
 */
static code39_result_t
decode_frame (uint32_t const * p_widths, uint8_t * p_symbol)
{
    code39_result_t result = CODE39_OK;
    uint8_t         symbols[CODE39_CHARS_PER_FRAME];

    for (uint32_t chr = 0u; chr < CODE39_CHARS_PER_FRAME; chr++)
    {
        uint32_t pattern  = 0u;
        uint32_t min_wide = 0u;

        symbols[chr] = 0u;

        if (CODE39_OK == result)
        {
            result = classify(&p_widths[chr * CHAR_STRIDE], &pattern,
                              &min_wide);
        }

        // The inter-character gap after this character is a narrow space,
        // so it must be narrower than this character's wide space.
        if ((CODE39_OK == result) &&
            ((chr + 1u) < CODE39_CHARS_PER_FRAME) &&
            (p_widths[(chr * CHAR_STRIDE) + CODE39_ELEMENTS_PER_CHAR] >=
             min_wide))
        {
            result = CODE39_BAD_RATIO;
        }

        if (CODE39_OK == result)
        {
            symbols[chr] = code39_symbol_for(pattern);
            result       = (0u == symbols[chr]) ? CODE39_NO_MATCH :
                                                  CODE39_OK;
        }
    }

    if ((CODE39_OK == result) &&
        ((FRAME_SYMBOL != symbols[START_CHAR]) ||
         (FRAME_SYMBOL != symbols[STOP_CHAR]) ||
         (symbols[DATA_CHAR] < FIRST_LETTER) ||
         (symbols[DATA_CHAR] > LAST_LETTER)))
    {
        result = CODE39_BAD_FRAME;
    }

    if (CODE39_OK == result)
    {
        *p_symbol = symbols[DATA_CHAR];
    }

    return (result);
}


/*!
 * @brief Decode a frame read backwards.
 *
 * @param[in]  p_widths  29 element widths in capture order.
 * @param[out] p_symbol  Data letter on success.
 *
 * @return CODE39_OK or the rejection reason.
 */
static code39_result_t
decode_reversed (uint32_t const * p_widths, uint8_t * p_symbol)
{
    uint32_t reversed[CODE39_FRAME_ELEMENTS];

    for (uint32_t idx = 0u; idx < CODE39_FRAME_ELEMENTS; idx++)
    {
        reversed[idx] = p_widths[(CODE39_FRAME_ELEMENTS - 1u) - idx];
    }

    return (decode_frame(reversed, p_symbol));
}


/*!
 * @brief Class one character's elements as wide or narrow.
 *
 * @param[in]  p_elements  9 element widths: bar, space, bar ... bar.
 * @param[out] p_pattern   9-bit pattern, first element in bit 8.
 * @param[out] p_min_wide  Width of the narrowest wide space.
 *
 * @return CODE39_OK, or CODE39_BAD_RATIO if the wide elements are not
 *         clearly (RATIO_NUMERATOR / RATIO_DENOMINATOR) wider than the
 *         narrow ones of the same colour.
 *
 * @par
 * Bars and spaces are classed separately: the 2 widest of the 5 bars and
 * the widest of the 4 spaces are wide.  Every letter and the '*' frame
 * character has exactly that mix (only '$', '/', '+' and '%' differ, and
 * they never appear in a "*X*" frame).  On the robot the IR spot blurs the
 * print, which makes every bar read wider and every space narrower than
 * printed; picking the 3 widest of all 9 then takes a narrow bar for a wide
 * space and the read fails.  Comparing each colour only with itself
 * cancels that bias, whichever direction the label is crossed in.
 */
static code39_result_t
classify (uint32_t const * p_elements, uint32_t * p_pattern,
          uint32_t * p_min_wide)
{
    uint32_t        pattern       = 0u;
    uint32_t        min_wide[2]   = { UINT32_MAX, UINT32_MAX };
    uint32_t        max_narrow[2] = { 0u, 0u };
    code39_result_t result        = CODE39_OK;

    // Colour 0 = bars (even positions), 1 = spaces (odd positions).
    for (uint32_t colour = 0u; colour < 2u; colour++)
    {
        uint32_t const wide_count = (0u == colour) ? WIDE_BARS : WIDE_SPACES;

        // Pick the widest elements of this colour, one at a time.
        for (uint32_t pick = 0u; pick < wide_count; pick++)
        {
            uint32_t best_idx = colour;
            uint32_t best     = 0u;

            for (uint32_t idx = colour; idx < CODE39_ELEMENTS_PER_CHAR;
                 idx += 2u)
            {
                uint32_t const bit = 1u << ((CODE39_ELEMENTS_PER_CHAR - 1u) -
                                            idx);

                if ((0u == (pattern & bit)) && (p_elements[idx] >= best))
                {
                    best     = p_elements[idx];
                    best_idx = idx;
                }
            }

            pattern |= 1u << ((CODE39_ELEMENTS_PER_CHAR - 1u) - best_idx);
            min_wide[colour] = (best < min_wide[colour]) ? best :
                                                           min_wide[colour];
        }

        for (uint32_t idx = colour; idx < CODE39_ELEMENTS_PER_CHAR; idx += 2u)
        {
            uint32_t const bit = 1u << ((CODE39_ELEMENTS_PER_CHAR - 1u) - idx);

            if ((0u == (pattern & bit)) &&
                (p_elements[idx] > max_narrow[colour]))
            {
                max_narrow[colour] = p_elements[idx];
            }
        }

        if ((min_wide[colour] * RATIO_DENOMINATOR) <
            (max_narrow[colour] * RATIO_NUMERATOR))
        {
            result = CODE39_BAD_RATIO;
        }
    }

    *p_pattern  = pattern;
    *p_min_wide = min_wide[1];

    return (result);
}

/*** end of file ***/


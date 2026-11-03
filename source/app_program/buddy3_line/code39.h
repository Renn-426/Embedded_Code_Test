/** @file code39.h
 *
 * @brief Code 39 decoder for single-letter navigation barcodes "*X*".
 *
 * @par
 * Owner: Buddy 3.  Pure module with no kernel or hardware dependency, so it
 * is unit-tested on the host (tests/test_main.c).
 *
 * @par
 * A frame is start character, data character, stop character: 3 characters
 * of 9 elements (5 bars, 4 spaces, exactly 3 wide) separated by 2 narrow
 * inter-character gaps, which is 29 elements in total (Week 6 Section 6.4).
 * Each character's three widest elements are classed as wide, so only the
 * ratio between elements matters and an unknown constant speed is
 * irrelevant.  If the forward read fails the elements are reversed and the
 * read is retried, because a barcode may be crossed in either direction
 * (FR2).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>
#include <stdbool.h>

#define CODE39_ELEMENTS_PER_CHAR    9u
#define CODE39_CHARS_PER_FRAME      3u
#define CODE39_FRAME_ELEMENTS       29u

/*!
 * @brief Decode result.
 */
typedef enum
{
    CODE39_OK = 0,
    CODE39_BAD_COUNT,       // Fewer than 29 elements.
    CODE39_BAD_RATIO,       // Wide and narrow elements not separable.
    CODE39_NO_MATCH,        // A pattern is not a Code 39 character.
    CODE39_BAD_FRAME        // Not of the form *X* with X in A..Z.
} code39_result_t;

code39_result_t code39_decode(uint32_t const * p_widths, uint32_t count,
                              uint8_t * p_symbol, bool * p_b_reversed);
uint8_t         code39_symbol_for(uint32_t pattern);

#endif /* CODE39_H */

/*** end of file ***/


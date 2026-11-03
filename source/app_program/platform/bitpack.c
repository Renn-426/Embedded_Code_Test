/** @file bitpack.c
 *
 * @brief Minimum-bit packing of application data (Change Requirement 1).
 *
 * @par
 * Pure functions with no kernel or hardware dependency, so the module is
 * also built and unit-tested on the host (tests/test_main.c).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "bitpack.h"

#define BITS_PER_BYTE           8u

static uint32_t field_mask(uint32_t width);


/*!
 * @brief Return a copy of a word with one field replaced.
 *
 * @param[in] word   Original word.
 * @param[in] shift  Bit position of the field's least significant bit.
 * @param[in] width  Field width in bits (1 to 32).
 * @param[in] value  New field value; bits above the width are discarded.
 *
 * @return The updated word.  The caller publishes it with one store.
 */
uint32_t
bitpack_word_set (uint32_t word, uint32_t shift, uint32_t width,
                  uint32_t value)
{
    uint32_t const mask = field_mask(width) << shift;

    return ((word & ~mask) | ((value << shift) & mask));
}


/*!
 * @brief Extract one field from a word.
 *
 * @param[in] word   Packed word.
 * @param[in] shift  Bit position of the field's least significant bit.
 * @param[in] width  Field width in bits (1 to 32).
 *
 * @return The field value, right-aligned.
 */
uint32_t
bitpack_word_get (uint32_t word, uint32_t shift, uint32_t width)
{
    return ((word >> shift) & field_mask(width));
}


/*!
 * @brief Encode a signed value as an excess-K unsigned field.
 *
 * @param[in] value  Signed value.
 * @param[in] width  Field width in bits (2 to 31).
 *
 * @return value + 2^(width-1), saturated to [0, 2^width - 1].
 */
uint32_t
bitpack_from_signed (int32_t value, uint32_t width)
{
    int32_t const  bias    = (int32_t) (1u << (width - 1u));  // width < 32.
    int32_t const  highest = bias - 1;
    int32_t const  lowest  = -bias;
    int32_t        clamped = value;

    if (clamped > highest)
    {
        clamped = highest;
    }
    else if (clamped < lowest)
    {
        clamped = lowest;
    }
    else
    {
        // In range: stored unchanged.
    }

    // clamped + bias lies in [0, 2^width - 1], so the cast is exact.
    return ((uint32_t) (clamped + bias));
}


/*!
 * @brief Decode an excess-K unsigned field back to a signed value.
 *
 * @param[in] field  Field value produced by bitpack_from_signed().
 * @param[in] width  Field width in bits (2 to 31).
 *
 * @return The signed value.
 */
int32_t
bitpack_to_signed (uint32_t field, uint32_t width)
{
    // Both casts are exact: field and the bias are below 2^31.
    int32_t const bias = (int32_t) (1u << (width - 1u));

    return ((int32_t) (field & field_mask(width)) - bias);
}


/*!
 * @brief Clamp an unsigned value to the largest value a field can hold.
 *
 * @param[in] value  Value to store.
 * @param[in] width  Field width in bits (1 to 32).
 *
 * @return value, or the all-ones field value if value does not fit.
 */
uint32_t
bitpack_saturate (uint32_t value, uint32_t width)
{
    uint32_t const mask   = field_mask(width);
    uint32_t       result = value;

    if (value > mask)
    {
        result = mask;
    }

    return (result);
}


/*!
 * @brief Start a new bit stream over a buffer and clear the buffer.
 *
 * @param[out] p_pack      Cursor to initialise.
 * @param[in]  p_buf       Destination buffer.
 * @param[in]  size_bytes  Buffer capacity in bytes.
 */
void
bitpack_init (bitpack_t * p_pack, uint8_t * p_buf, uint32_t size_bytes)
{
    p_pack->p_buf      = p_buf;
    p_pack->size_bytes = size_bytes;
    p_pack->bit_pos    = 0u;
    p_pack->b_overflow = false;

    for (uint32_t idx = 0u; idx < size_bytes; idx++)
    {
        p_buf[idx] = 0u;
    }
}


/*!
 * @brief Append a field to the stream.
 *
 * @param[in,out] p_pack  Stream cursor.
 * @param[in]     value   Field value; bits above the width are discarded.
 * @param[in]     width   Field width in bits (1 to 32).
 *
 * @par
 * A field that would not fit marks the stream as overflowed and is dropped.
 */
void
bitpack_put (bitpack_t * p_pack, uint32_t value, uint32_t width)
{
    uint32_t const masked = value & field_mask(width);

    if ((p_pack->bit_pos + width) > (p_pack->size_bytes * BITS_PER_BYTE))
    {
        p_pack->b_overflow = true;
    }
    else
    {
        for (uint32_t bit = 0u; bit < width; bit++)
        {
            uint32_t const pos = p_pack->bit_pos + bit;

            if (0u != ((masked >> bit) & 1u))
            {
                p_pack->p_buf[pos / BITS_PER_BYTE] |=
                    (uint8_t) (1u << (pos % BITS_PER_BYTE));  // Shift < 8.
            }
        }

        p_pack->bit_pos += width;
    }
}


/*!
 * @brief Read the next field from the stream.
 *
 * @param[in,out] p_pack  Stream cursor.
 * @param[in]     width   Field width in bits (1 to 32).
 *
 * @return The field value, or 0 if the stream holds too few bits (the
 *         stream is then marked as overflowed).
 */
uint32_t
bitpack_get (bitpack_t * p_pack, uint32_t width)
{
    uint32_t value = 0u;

    if ((p_pack->bit_pos + width) > (p_pack->size_bytes * BITS_PER_BYTE))
    {
        p_pack->b_overflow = true;
    }
    else
    {
        for (uint32_t bit = 0u; bit < width; bit++)
        {
            uint32_t const pos  = p_pack->bit_pos + bit;
            uint32_t const byte = p_pack->p_buf[pos / BITS_PER_BYTE];

            if (0u != ((byte >> (pos % BITS_PER_BYTE)) & 1u))
            {
                value |= (1u << bit);
            }
        }

        p_pack->bit_pos += width;
    }

    return (value);
}


/*!
 * @brief Number of whole bytes the stream occupies so far.
 *
 * @param[in] p_pack  Stream cursor.
 *
 * @return Bytes used, rounded up to a whole byte.
 */
uint32_t
bitpack_bytes_used (bitpack_t const * p_pack)
{
    return ((p_pack->bit_pos + (BITS_PER_BYTE - 1u)) / BITS_PER_BYTE);
}


/*!
 * @brief Report whether any field failed to fit.
 *
 * @param[in] p_pack  Stream cursor.
 *
 * @return true if a put or get ran past the buffer.
 */
bool
bitpack_is_overflow (bitpack_t const * p_pack)
{
    return (p_pack->b_overflow);
}


/*!
 * @brief All-ones mask of a given width.
 *
 * @param[in] width  Width in bits (1 to 32).
 *
 * @return The mask, e.g. 0x7 for width 3.
 */
static uint32_t
field_mask (uint32_t width)
{
    uint32_t mask = 0xFFFFFFFFu;

    if (width < BITPACK_MAX_WIDTH)
    {
        mask = (1u << width) - 1u;
    }

    return (mask);
}

/*** end of file ***/


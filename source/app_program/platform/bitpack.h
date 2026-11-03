/** @file bitpack.h
 *
 * @brief Minimum-bit packing of application data (Change Requirement 1).
 *
 * @par
 * Two facilities are provided.  Word fields pack several small values into
 * one 32-bit word with explicit masks and shifts, so a shared word can be
 * published with a single aligned store.  Bit streams pack a sequence of
 * fields of any width into a byte buffer for message buffers and MQTT.
 * Bits are packed least-significant first in both cases; tools/tlm_decode.py
 * mirrors this order.  C bit-fields are deliberately not used, because their
 * bit order is implementation-defined (Barr-C rule 5.5.b).
 *
 * @par
 * Signed values are stored with excess-K (offset) encoding: a signed value v
 * in a w-bit field is stored as v + 2^(w-1), saturated to the field range.
 * This keeps every bitwise operation on unsigned data (Barr-C rule 5.3.b).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>
#include <stdbool.h>

// Widest field supported by the word and stream helpers.
#define BITPACK_MAX_WIDTH       32u

/*!
 * @brief Write cursor over a caller-owned byte buffer.
 */
typedef struct
{
    uint8_t *   p_buf;
    uint32_t    size_bytes;
    uint32_t    bit_pos;
    bool        b_overflow;
} bitpack_t;

uint32_t bitpack_word_set(uint32_t word, uint32_t shift, uint32_t width,
                          uint32_t value);
uint32_t bitpack_word_get(uint32_t word, uint32_t shift, uint32_t width);
uint32_t bitpack_from_signed(int32_t value, uint32_t width);
int32_t  bitpack_to_signed(uint32_t field, uint32_t width);
uint32_t bitpack_saturate(uint32_t value, uint32_t width);

void     bitpack_init(bitpack_t * p_pack, uint8_t * p_buf,
                      uint32_t size_bytes);
void     bitpack_put(bitpack_t * p_pack, uint32_t value, uint32_t width);
uint32_t bitpack_get(bitpack_t * p_pack, uint32_t width);
uint32_t bitpack_bytes_used(bitpack_t const * p_pack);
bool     bitpack_is_overflow(bitpack_t const * p_pack);

#endif /* BITPACK_H */

/*** end of file ***/


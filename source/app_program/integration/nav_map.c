/** @file nav_map.c
 *
 * @brief Barcode letter to navigation action map.
 *
 * @par
 * Default map (project brief): A = turn left, B = turn right,
 * C = go straight, D = U-turn; every other letter goes straight, which is
 * also what the robot does at a junction with no barcode latched.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "bitpack.h"
#include "nav_map.h"

#define LETTERS_PER_WORD        16u
#define LETTER_A                0u
#define LETTER_B                1u
#define LETTER_C                2u
#define LETTER_D                3u
#define ACTION_COUNT            4u

static uint32_t word_index(uint32_t letter);
static uint32_t field_shift(uint32_t letter);


/*!
 * @brief Load the default map, version 0.
 *
 * @param[out] p_map  Map.
 */
void
nav_map_init (nav_map_t * p_map)
{
    p_map->version = 0u;

    for (uint32_t idx = 0u; idx < NAV_MAP_WORDS; idx++)
    {
        p_map->words[idx] = 0u;
    }

    for (uint32_t letter = 0u; letter < NAV_MAP_LETTERS; letter++)
    {
        p_map->words[word_index(letter)] =
            bitpack_word_set(p_map->words[word_index(letter)],
                             field_shift(letter), NAV_MAP_ACTION_WIDTH,
                             (uint32_t) NAV_ACTION_STRAIGHT);  // Enum < 4.
    }

    // (void): these constant letters and actions are always valid.
    (void) nav_map_set(p_map, LETTER_A, NAV_ACTION_LEFT, 0u);
    (void) nav_map_set(p_map, LETTER_B, NAV_ACTION_RIGHT, 0u);
    (void) nav_map_set(p_map, LETTER_C, NAV_ACTION_STRAIGHT, 0u);
    (void) nav_map_set(p_map, LETTER_D, NAV_ACTION_UTURN, 0u);
}


/*!
 * @brief Action for a letter.
 *
 * @param[in] p_map   Map.
 * @param[in] letter  0 = A .. 25 = Z.
 *
 * @return The mapped action; NAV_ACTION_STRAIGHT for an invalid letter.
 */
nav_map_action_t
nav_map_get (nav_map_t const * p_map, uint32_t letter)
{
    nav_map_action_t action = NAV_ACTION_STRAIGHT;

    if (letter < NAV_MAP_LETTERS)
    {
        uint32_t const field = bitpack_word_get(p_map->words[word_index(
                                                    letter)],
                                                field_shift(letter),
                                                NAV_MAP_ACTION_WIDTH);

        switch (field)
        {
            case NAV_ACTION_LEFT:
                action = NAV_ACTION_LEFT;
            break;

            case NAV_ACTION_RIGHT:
                action = NAV_ACTION_RIGHT;
            break;

            case NAV_ACTION_UTURN:
                action = NAV_ACTION_UTURN;
            break;

            default:
                action = NAV_ACTION_STRAIGHT;
            break;
        }
    }

    return (action);
}


/*!
 * @brief Change one letter's action and record the map version.
 *
 * @param[in,out] p_map    Map.
 * @param[in]     letter   0 = A .. 25 = Z.
 * @param[in]     action   nav_map_action_t value, 0..3.
 * @param[in]     version  New map version (4 bits).
 *
 * @return true if applied; false (map unchanged) for an invalid letter or
 *         action.
 */
bool
nav_map_set (nav_map_t * p_map, uint32_t letter, uint32_t action,
             uint32_t version)
{
    bool b_applied = false;

    if ((letter < NAV_MAP_LETTERS) && (action < ACTION_COUNT))
    {
        uint32_t const idx = word_index(letter);

        p_map->words[idx] = bitpack_word_set(p_map->words[idx],
                                             field_shift(letter),
                                             NAV_MAP_ACTION_WIDTH, action);
        p_map->version    = bitpack_saturate(version, NAV_MAP_VERSION_WIDTH);
        b_applied         = true;
    }

    return (b_applied);
}


/*!
 * @brief Current map version.
 *
 * @param[in] p_map  Map.
 *
 * @return Version, 0..15.
 */
uint32_t
nav_map_version (nav_map_t const * p_map)
{
    return (p_map->version);
}


/*!
 * @brief Which word holds a letter.
 *
 * @param[in] letter  Valid letter index.
 *
 * @return 0 or 1.
 */
static uint32_t
word_index (uint32_t letter)
{
    return (letter / LETTERS_PER_WORD);
}


/*!
 * @brief Bit position of a letter's field inside its word.
 *
 * @param[in] letter  Valid letter index.
 *
 * @return Shift in bits.
 */
static uint32_t
field_shift (uint32_t letter)
{
    return ((letter % LETTERS_PER_WORD) * NAV_MAP_ACTION_WIDTH);
}

/*** end of file ***/


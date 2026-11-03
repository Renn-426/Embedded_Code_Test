/** @file nav_map.h
 *
 * @brief Barcode letter to navigation action map (FR3, FR10).
 *
 * @par
 * Owner: Vehicle Control (integration).  Pure module, unit-tested on the
 * host.  26 letters x 2 bits are packed into two 32-bit words (Change
 * Requirement 1).  The map is used only by the Vehicle Control task, so it
 * needs no lock; MQTT updates reach it through mbf_ctrl_cmd.
 *
 * @par
 * Activation rule (Tribunal 1): a letter is resolved to its action when the
 * barcode is decoded, so a map update applies from the next barcode and
 * never changes a command already latched for the coming junction.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef NAV_MAP_H
#define NAV_MAP_H

#include <stdint.h>
#include <stdbool.h>

#define NAV_MAP_LETTERS             26u
#define NAV_MAP_WORDS               2u
#define NAV_MAP_ACTION_WIDTH        2u
#define NAV_MAP_VERSION_WIDTH       4u

/*!
 * @brief Navigation actions (2 bits).
 */
typedef enum
{
    NAV_ACTION_LEFT = 0,
    NAV_ACTION_RIGHT,
    NAV_ACTION_STRAIGHT,
    NAV_ACTION_UTURN
} nav_map_action_t;

/*!
 * @brief The packed map.
 */
typedef struct
{
    uint32_t    words[NAV_MAP_WORDS];
    uint32_t    version;
} nav_map_t;

void             nav_map_init(nav_map_t * p_map);
nav_map_action_t nav_map_get(nav_map_t const * p_map, uint32_t letter);
bool             nav_map_set(nav_map_t * p_map, uint32_t letter,
                             uint32_t action, uint32_t version);
uint32_t         nav_map_version(nav_map_t const * p_map);

#endif /* NAV_MAP_H */

/*** end of file ***/


/** @file module.h
 *
 * @brief One-line description of what the module does.
 *
 * @par
 * Longer description: the module's job, which buddy owns it, which task
 * calls it, and any rules a caller must follow.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef MODULE_H
#define MODULE_H

#include <stdint.h>
#include <stdbool.h>

// Public constants, named MODULE_SOMETHING.
#define MODULE_MAX_ITEMS        8u

/*!
 * @brief Public type, named with the module prefix and the _t suffix.
 */
typedef enum
{
    MODULE_STATE_IDLE = 0,
    MODULE_STATE_BUSY,
} module_state_t;

// Public functions, named module_verb_noun().  No storage in headers.
bool           module_init(void);
module_state_t module_get_state(void);

#endif /* MODULE_H */

/*** end of file ***/


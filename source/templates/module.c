/** @file module.c
 *
 * @brief One-line description of what the module does.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include "module.h"

#include <stdint.h>
#include <stdbool.h>

// Private constants.
#define MODULE_TIMEOUT_MS       100u

// Private types.
typedef struct
{
    module_state_t state;
    uint32_t       count;
} module_ctx_t;

// Private data: file scope, static, prefixed g_ only when truly global.
static module_ctx_t g_module_ctx;

// Private function prototypes.
static void module_reset(void);

/*!
 * @brief Prepare the module for use.
 *
 * @return true on success, false otherwise.
 */
bool
module_init (void)
{
    module_reset();

    return (true);
}

/*!
 * @brief Report the module's current state.
 *
 * @return The current state.
 */
module_state_t
module_get_state (void)
{
    return (g_module_ctx.state);
}

/*!
 * @brief Return the private state to its power-on values.
 */
static void
module_reset (void)
{
    g_module_ctx.state = MODULE_STATE_IDLE;
    g_module_ctx.count = 0u;
}

/*** end of file ***/


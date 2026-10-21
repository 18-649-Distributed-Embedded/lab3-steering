#include "sys/health.h"
#include <zephyr/sys/atomic.h>
#include "core/cmd.h"

/* Safety interlock flags — initialized to WATCHDOG_TIMEOUT (fail-safe until cleared) */
atomic_t system_safety_flags = ATOMIC_INIT(BIT(SAFETY_FLAG_WATCHDOG_TIMEOUT));

void health_init(void)
{
    /* Thread removed for Lab 3 Part 1 — will be re-added in Part 2 */
}

#ifndef SYS_DEBUG_STATUS_H
#define SYS_DEBUG_STATUS_H

#include <stdint.h>
#include <stdbool.h>

/*
 * STAT logger: one compact console line per heartbeat (every 20 ms).
 * HB_TX calls debug_status_on_heartbeat() each period; if no heartbeat
 * tick arrives for 100 ms (HB_TX blocked), the line is printed anyway
 * tagged "HB STALLED". Compiled out unless CONFIG_APP_DEBUG_LOGGING=y.
 */
#ifdef CONFIG_APP_DEBUG_LOGGING
void debug_status_init(void);
void debug_status_on_heartbeat(uint8_t hb_seq, bool sent);
#else
static inline void debug_status_init(void) {}
static inline void debug_status_on_heartbeat(uint8_t hb_seq, bool sent) {}
#endif

#endif /* SYS_DEBUG_STATUS_H */

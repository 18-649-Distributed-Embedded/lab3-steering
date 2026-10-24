#ifndef SYS_HB_MON_H
#define SYS_HB_MON_H

#include <zephyr/sys/atomic.h>

/*
 * Connection state at the 66 ms threshold, updated every 20 ms by HB_MON.
 *   conn_bits_slow: CONN_N1A | CONN_N1B | CONN_N3A | CONN_N3B (core/conn_state.h)
 *   conn_row_slow:  enum conn_row (1..12), read by STEER and the STAT line
 */
extern atomic_t conn_bits_slow;
extern atomic_t conn_row_slow;

/**
 * @brief Start the HB_MON task: monitors N1 + N3 heartbeats on both buses,
 *        classifies the connection state (12 rows), updates ZONE_FAIL flags,
 *        drives the DEG_A / DEG_B LED patterns and the ERR test point.
 */
void hb_mon_init(void);

#endif /* SYS_HB_MON_H */

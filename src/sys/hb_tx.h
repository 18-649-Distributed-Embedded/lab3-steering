#ifndef SYS_HB_TX_H
#define SYS_HB_TX_H

#include <stdbool.h>

/**
 * @brief Start the HB_TX task: sends HB_N2 (0x101) on both CAN buses every 20ms.
 */
void hb_tx_init(void);

/**
 * @brief Re-send the most recent HB_N2 on both buses (relay ACK probe).
 *        Does nothing while self-failed or before the first heartbeat.
 * @return true if a probe was queued.
 */
bool hb_tx_send_probe(void);

#endif /* SYS_HB_TX_H */

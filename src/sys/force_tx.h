#ifndef SYS_FORCE_TX_H
#define SYS_FORCE_TX_H

/**
 * @brief Start the FORCE_TX task: sends FORCE (0x300) on both buses every 50ms.
 *        Payload byte 1 = force int8 in [-100, 100]. Lab 3: dummy counter.
 */
void force_tx_init(void);

#endif /* SYS_FORCE_TX_H */

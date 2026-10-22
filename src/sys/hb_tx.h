#ifndef SYS_HB_TX_H
#define SYS_HB_TX_H

/**
 * @brief Start the HB_TX task: sends HB_N2 (0x101) on both CAN buses every 20ms.
 */
void hb_tx_init(void);

#endif /* SYS_HB_TX_H */


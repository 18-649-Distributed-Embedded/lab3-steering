#ifndef SYS_CAN_RX_H
#define SYS_CAN_RX_H

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

/*
 * IPC primitives produced by the CAN RX dispatch.
 * All consumers include this header and use these directly.
 */

/* Semaphores — max count 1, given on message arrival, taken by handler tasks */
extern struct k_sem sem_ctrl;    /* given on CTRL (0x010) */
extern struct k_sem sem_blink;   /* given on BLINK (0x200) */

/*
 * Mailboxes (atomic_t, latest-value, non-blocking read)
 *
 * mailbox_ctrl:    [buttons:8][throttle:8][brake:8]
 * mailbox_blinker: [0:16][blink_state:8][phase_bit:8]
 *
 * HB timestamps: k_uptime_get_32() value at last valid HB reception.
 * Initialized to 0 (far in the past — HB_MON will see them as stale).
 */
extern atomic_t mailbox_ctrl;
extern atomic_t mailbox_blinker;

extern atomic_t mailbox_hb_n1_a;   /* last HB_N1 on bus A */
extern atomic_t mailbox_hb_n1_b;   /* last HB_N1 on bus B */
extern atomic_t mailbox_hb_n2_a;   /* last HB_N2 on bus A */
extern atomic_t mailbox_hb_n2_b;   /* last HB_N2 on bus B */

extern atomic_t mailbox_n1_state;  /* last known zone state of N1 */
extern atomic_t mailbox_n2_state;  /* last known zone state of N2 */

/* Watchdog: last valid CTRL reception time */
extern volatile uint32_t last_ctrl_time;

/**
 * @brief Register RX filters on both CAN buses and start dispatching.
 *        Must be called after can_bus_init().
 */
void can_rx_init(void);

#endif /* SYS_CAN_RX_H */


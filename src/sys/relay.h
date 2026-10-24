#ifndef SYS_RELAY_H
#define SYS_RELAY_H

#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>

/*
 * RELAY task (R4.8 scenario (c), Lab 3 Part 4).
 *
 * Every CONN_RELAY_CHECK_MS (5 ms) the RELAY thread classifies the four
 * connection bits at the relay threshold (30 ms = 1.5 heartbeats). If the
 * pattern is a relay row (8 or 9) on CONN_RELAY_STABLE_N (2) consecutive
 * checks, relay mode becomes active:
 *   row 8 (0 1 1 0): N1 frames B -> A, N3 frames A -> B
 *   row 9 (1 0 0 1): N1 frames A -> B, N3 frames B -> A
 * Relayed frames are bit-identical copies (same ID, data, seq).
 *
 * ACK gate: on the first check that sees a relay row, N2 re-sends its last
 * heartbeat on both buses (probe) and confirms relay mode at the next check
 * only if BOTH buses ACKed something after the probe. This rejects the brief
 * relay-looking pattern N2 sees while losing one of its OWN buses.
 * A self-failed zone (test button) never relays.
 *
 * On activation the relay is PRIMED: the latest HB_N1, HB_N3 and CTRL that
 * N2 already holds are sent immediately, so the stranded node refreshes its
 * heartbeat timestamps at once instead of waiting up to one more period for
 * the next frame. This is what keeps the stranded node from declaring the
 * other zone failed (see tests/host race simulation).
 *
 * RELAY_TX test point (PA8 / Arduino D7) toggles once per forwarded frame,
 * right after the send on the destination bus returns.
 */

/* CONN_ROW_NONE, CONN_ROW_RELAY_1 or CONN_ROW_RELAY_2 */
extern atomic_t relay_active_row;
extern atomic_t relay_tx_count;     /* frames forwarded since boot */
extern atomic_t relay_drop_count;   /* frames not forwarded (queue/buffer full) */

/** Start the RELAY thread and configure the RELAY_TX test point. */
void relay_init(void);

/**
 * Called from the CAN RX dispatch for EVERY received frame, before dedup.
 * Caches the latest HB_N1 / HB_N3 / CTRL (for priming) and, if relay mode is
 * active and the frame qualifies, queues it for forwarding. Never blocks.
 * @param bus 0 = A, 1 = B
 */
void relay_on_rx(int bus, const struct can_frame *frame);

#endif /* SYS_RELAY_H */

#ifndef HW_CAN_H
#define HW_CAN_H

#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>
#include <stdint.h>

/* Max wait for a free TX buffer per bus (about one frame time at 500 kbit/s).
 * Absorbs short bursts; a bus that is not draining costs at most this. */
#define CAN_TX_BUF_WAIT  K_USEC(300)

/* k_cycle_get_32() of the last frame ACKed on bus A [0] / bus B [1] */
extern atomic_t can_last_ack_cyc[2];

/* Opaque handles for both MCP2515 controllers */
extern const struct device *can_a_dev;   /* CAN0: SPI2 */
extern const struct device *can_b_dev;   /* CAN1: SPI3 */

/**
 * @brief Initialize and start both MCP2515 CAN controllers.
 * @return 0 on success, negative errno on failure.
 */
int can_bus_init(void);

/**
 * @brief Log controller state + error counters for both buses.
 */
void can_log_state(void);

/**
 * @brief Queue a frame on one CAN bus (non-blocking: completion via callback).
 *        Pulses that bus's CAN_TX test point around the queue call.
 * @param timeout Max wait for a free controller TX buffer.
 * @return 0 if queued, -EAGAIN if no buffer freed up in time.
 */
int can_send_bus(const struct device *dev, struct can_frame *frame, k_timeout_t timeout);

/**
 * @brief Queue an identical frame on BOTH CAN buses (non-blocking, independent).
 * @param id   11-bit CAN standard frame ID
 * @param dlc  Data length (0-8)
 * @param data Payload bytes
 * @return 0 if both were queued, -EIO if either was not
 */
int can_send_both(uint32_t id, uint8_t dlc, const uint8_t *data);

#endif /* HW_CAN_H */

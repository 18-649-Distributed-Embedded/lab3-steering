#ifndef HW_CAN_H
#define HW_CAN_H

#include <zephyr/drivers/can.h>
#include <stdint.h>

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
 * @brief Transmit an identical frame on BOTH CAN buses.
 * @param id   11-bit CAN standard frame ID
 * @param dlc  Data length (0-8)
 * @param data Payload bytes
 * @return 0 if both sends succeed, -EIO if either fails
 */
int can_send_both(uint32_t id, uint8_t dlc, const uint8_t *data);

#endif /* HW_CAN_H */

#ifndef HW_CAN_H
#define HW_CAN_H

#include <zephyr/drivers/can.h>

/* Opaque handles — use these everywhere else in the codebase */
extern const struct device *can_a_dev;  /* CAN0: SPI2, CAN bus A */
extern const struct device *can_b_dev;  /* CAN1: SPI3, CAN bus B */

/**
 * @brief Initialize both MCP2515 CAN controllers and start them.
 * @return 0 on success, negative errno on failure.
 */
int can_bus_init(void);

/**
 * @brief Log the current controller state and error counters for both buses.
 *        Used at boot to verify the checkpoint ("controller is active").
 */
void can_log_state(void);

#endif /* HW_CAN_H */


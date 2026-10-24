#include "hw/can.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include "core/log.h"

const struct device *can_a_dev = DEVICE_DT_GET(DT_ALIAS(can_a));
const struct device *can_b_dev = DEVICE_DT_GET(DT_ALIAS(can_b));

static const struct gpio_dt_spec can0_tx_tp = GPIO_DT_SPEC_GET(DT_ALIAS(can0_tx), gpios);
static const struct gpio_dt_spec can1_tx_tp = GPIO_DT_SPEC_GET(DT_ALIAS(can1_tx), gpios);

/* k_cycle_get_32() of the last frame ACKed on each bus (0 = A, 1 = B) */
atomic_t can_last_ack_cyc[2];

int can_bus_init(void)
{
    int ret;

    /* Initialize TX test point GPIOs */
    if (device_is_ready(can0_tx_tp.port)) {
        gpio_pin_configure_dt(&can0_tx_tp, GPIO_OUTPUT_INACTIVE);
    }
    if (device_is_ready(can1_tx_tp.port)) {
        gpio_pin_configure_dt(&can1_tx_tp, GPIO_OUTPUT_INACTIVE);
    }

    if (!device_is_ready(can_a_dev)) {
        DEBUG_PRINT("Error: CAN A device not ready\n");
        return -ENODEV;
    }
    ret = can_start(can_a_dev);
    if (ret != 0 && ret != -EALREADY) {
        DEBUG_PRINT("Error: CAN A start failed (%d)\n", ret);
        return ret;
    }
    DEBUG_PRINT("CAN A initialized and started\n");

    if (!device_is_ready(can_b_dev)) {
        DEBUG_PRINT("Error: CAN B device not ready\n");
        return -ENODEV;
    }
    ret = can_start(can_b_dev);
    if (ret != 0 && ret != -EALREADY) {
        DEBUG_PRINT("Error: CAN B start failed (%d)\n", ret);
        return ret;
    }
    DEBUG_PRINT("CAN B initialized and started\n");

    return 0;
}

void can_log_state(void)
{
    enum can_state state_a, state_b;
    struct can_bus_err_cnt err_a, err_b;

    can_get_state(can_a_dev, &state_a, &err_a);
    can_get_state(can_b_dev, &state_b, &err_b);

    DEBUG_PRINT("CAN A state=%d (TEC=%u REC=%u)\n",
                state_a, err_a.tx_err_cnt, err_a.rx_err_cnt);
    DEBUG_PRINT("CAN B state=%d (TEC=%u REC=%u)\n",
                state_b, err_b.tx_err_cnt, err_b.rx_err_cnt);
}

/* TX completion callback (runs in the MCP2515 driver thread once the frame
 * is ACKed or fails). Passing a callback makes can_send() non-blocking: it
 * returns as soon as the frame is in a controller TX buffer, instead of
 * waiting for it to go out on the bus. Required by the Lab 3 clarification
 * so CAN_TX / RELAY_TX test points exclude bus time.
 * error == 0 means another node ACKed the frame: record when, per bus
 * (used by the relay ACK gate). user_data: 0 = bus A, 1 = bus B. */
static void tx_done_cb(const struct device *dev, int error, void *user_data)
{
    ARG_UNUSED(dev);
    if (error == 0) {
        atomic_set(&can_last_ack_cyc[(uintptr_t)user_data ? 1 : 0],
                   (atomic_val_t)k_cycle_get_32());
    }
}

int can_send_bus(const struct device *dev, struct can_frame *frame, k_timeout_t timeout)
{
    const struct gpio_dt_spec *tx_tp = (dev == can_a_dev) ? &can0_tx_tp : &can1_tx_tp;
    gpio_pin_set_dt(tx_tp, 1);
    /* timeout only bounds the wait for a free TX buffer (3 per MCP2515) */
    int ret = can_send(dev, frame, timeout, tx_done_cb,
                       (void *)(uintptr_t)((dev == can_a_dev) ? 0 : 1));
    gpio_pin_set_dt(tx_tp, 0);
    return ret;
}

int can_send_both(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    struct can_frame f = {
        .id  = id,
        .dlc = dlc,
    };
    memcpy(f.data, data, dlc);

    /* Queued independently: a dead / un-ACKed bus whose buffers are full
     * delays the other bus by at most CAN_TX_BUF_WAIT, never blocks it. */
    int ra = can_send_bus(can_a_dev, &f, CAN_TX_BUF_WAIT);
    int rb = can_send_bus(can_b_dev, &f, CAN_TX_BUF_WAIT);

    return (ra == 0 && rb == 0) ? 0 : -EIO;
}

#include "hw/can.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <string.h>
#include "core/log.h"

const struct device *can_a_dev = DEVICE_DT_GET(DT_ALIAS(can_a));
const struct device *can_b_dev = DEVICE_DT_GET(DT_ALIAS(can_b));

int can_bus_init(void)
{
    int ret;

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

int can_send_both(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    struct can_frame f = {
        .id  = id,
        .dlc = dlc,
    };
    memcpy(f.data, data, dlc);

    int ra = can_send(can_a_dev, &f, K_MSEC(5), NULL, NULL);
    int rb = can_send(can_b_dev, &f, K_MSEC(5), NULL, NULL);

    return (ra == 0 && rb == 0) ? 0 : -EIO;
}

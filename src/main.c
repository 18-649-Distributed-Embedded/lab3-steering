#include <zephyr/kernel.h>
#include "hw/servo.h"
#include "hw/adc.h"
#include "hw/blinkers.h"
#include "hw/can.h"
#include "sys/steering.h"
#include "sys/blinkers_control.h"
#include "sys/health.h"
#include "sys/uart_parser.h"
#include "core/log.h"

int main(void)
{
    DEBUG_PRINT("=== Lab 3 Steering Node: Initializing ===\n");

    /* --- Hardware layer --- */
    DEBUG_PRINT("Initializing hardware...\n");
    if (servo_init() != 0) return -1;
    if (adc_init()      != 0) return -1;
    if (blinkers_init() != 0) return -1;
    if (can_bus_init()  != 0) return -1;

    /* --- System layer (stub inits only — threads added in Part 2) --- */
    DEBUG_PRINT("Initializing system stubs...\n");
    uart_parser_init();
    steering_init();
    blinkers_control_init();
    health_init();

    DEBUG_PRINT("=== Initialization complete. Checkpoint reached. ===\n");

    /* Verify both CAN controllers came up in ERROR-ACTIVE state */
    can_log_state();

    return 0;
}

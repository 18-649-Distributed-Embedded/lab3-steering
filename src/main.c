#include <zephyr/kernel.h>
#include "hw/servo.h"
#include "hw/adc.h"
#include "hw/blinkers.h"
#include "hw/can.h"
#include "sys/can_rx.h"
#include "sys/steering.h"
#include "sys/blinkers_control.h"
#include "sys/hb_tx.h"
#include "sys/hb_mon.h"
#include "sys/health.h"
#include "core/log.h"

int main(void)
{
    DEBUG_PRINT("=== Lab 3 N3 (Drivetrain): Initializing ===\n");

    /* ── Hardware layer ──────────────────────────────────── */
    DEBUG_PRINT("[HW] Initializing...\n");
    if (servo_init()    != 0) return -1;
    if (adc_init()      != 0) return -1;
    if (blinkers_init() != 0) return -1;
    if (can_bus_init()  != 0) return -1;

    can_log_state();

    /* ── System tasks (in priority order, lowest number = highest prio) ── */
    DEBUG_PRINT("[SYS] Starting tasks...\n");

    can_rx_init();           /* prio 2 (MCP2515 driver thread) — RX dispatch  */
    health_init();           /* prio 3 — TEST_BTN + watchdog                  */
    steering_init();            /* prio 3 — DRIVE (throttle/brake on CTRL)        */
    blinkers_control_init(); /* prio 4 — BLINK_R (rear LEDs on BLINK)          */
    hb_tx_init();            /* prio 5 — HB_TX (0x102 every 20ms)              */
    hb_mon_init();           /* prio 6 — HB_MON + DEG_A/B LEDs (every 20ms)   */

    DEBUG_PRINT("=== Initialization complete. Zephyr scheduler active. ===\n");

    return 0;    /* main() exits; all work happens in the tasks above */
}

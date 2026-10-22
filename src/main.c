#include <zephyr/kernel.h>
#include "hw/servo.h"
#include "hw/adc.h"
#include "hw/blinkers.h"
#include "hw/can.h"
#include "sys/can_rx.h"
#include "sys/steering.h"
#include "sys/blinkers_control.h"
#include "sys/hb_tx.h"
#include "sys/force_tx.h"
#include "sys/hb_mon.h"
#include "sys/health.h"
#include "sys/debug_status.h"
#include "core/log.h"

int main(void)
{
    DEBUG_PRINT("=== Lab 3 N2 (Steering): Initializing ===\n");

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
    steering_init();         /* prio 3 — STEER (servo PWM on CTRL)             */
    blinkers_control_init(); /* prio 4 — BLINK_F (front LEDs on BLINK)         */
    hb_tx_init();            /* prio 5 — HB_TX (0x101 every 20ms)              */
    force_tx_init();         /* prio 5 — FORCE_TX (0x300 every 50ms, dummy)    */
    hb_mon_init();           /* prio 6 — HB_MON + DEG_A/B LEDs (every 20ms)   */
    debug_status_init();     /* prio 10 — STAT line per heartbeat (debug)     */

    DEBUG_PRINT("=== Initialization complete. Zephyr scheduler active. ===\n");

    return 0;    /* main() exits; all work happens in the tasks above */
}

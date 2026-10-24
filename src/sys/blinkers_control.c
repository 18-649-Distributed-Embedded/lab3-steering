#include "sys/blinkers_control.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "sys/can_rx.h"
#include "core/can_catalog.h"
#include "core/cmd.h"
#include "hw/blinkers.h"

#define BLINK_F_THREAD_STACK  512
#define BLINK_F_THREAD_PRIO   3      /* deadline 0.5 ms from reception */

static void blink_f_thread_func(void *p1, void *p2, void *p3)
{
    while (1) {
        /*
         * Wait for BLINK message, or timeout after 50ms so local fail-safe
         * hazards can flash if communication is lost.
         */
        k_sem_take(&sem_blink, K_MSEC(50));

        bool is_failsafe = (atomic_get(&system_safety_flags) != 0);

        if (is_failsafe) {
            /* Fail-safe / error state: local 2 Hz hazard flashing (500 ms period, 50 %) */
            uint32_t now = k_uptime_get_32();
            bool on = (now % 500U) < 250U;
            blinkers_set_left(on);
            blinkers_set_right(on);
        } else {
            uint32_t val        = (uint32_t)atomic_get(&mailbox_blinker);
            uint8_t blink_state = (val >> 8) & 0xFF;
            uint8_t phase_bit   = (val     ) & 0xFF;

            bool on = (phase_bit != 0);

            blinkers_set_left(false);
            blinkers_set_right(false);

            switch (blink_state) {
            case CAN_BLINK_LEFT:
                blinkers_set_left(on);
                break;
            case CAN_BLINK_RIGHT:
                blinkers_set_right(on);
                break;
            case CAN_BLINK_HAZARD:
                blinkers_set_left(on);
                blinkers_set_right(on);
                break;
            case CAN_BLINK_OFF:
            default:
                break;
            }
        }
    }
}

K_THREAD_STACK_DEFINE(blink_f_stack, BLINK_F_THREAD_STACK);
static struct k_thread blink_f_thread_data;

void blinkers_control_init(void)
{
    blinkers_set_left(false);
    blinkers_set_right(false);

    k_thread_create(&blink_f_thread_data, blink_f_stack,
                    K_THREAD_STACK_SIZEOF(blink_f_stack),
                    blink_f_thread_func,
                    NULL, NULL, NULL,
                    BLINK_F_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&blink_f_thread_data, "BLINK_F");
}

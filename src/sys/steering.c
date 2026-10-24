#include "sys/steering.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "sys/can_rx.h"
#include "sys/hb_mon.h"
#include "core/conn_state.h"
#include "core/cmd.h"
#include "hw/servo.h"

#define STEERING_THREAD_STACK_SIZE 1024
#define STEERING_THREAD_PRIORITY   4       /* local deadline 1 ms (DM order) */
#define STEERING_MAX_WAIT_MS       20      /* re-check connection row at HB_MON rate */

/*
 * Steer -> servo mapping (team-defined values table: keep these in sync).
 * Linear and monotonic: STEER_IN_MIN -> SERVO_MIN_PULSE_NS (full left),
 *                       STEER_IN_MAX -> SERVO_MAX_PULSE_NS (full right).
 * Narrow the servo.h pulse limits if the chassis linkage hits its stops
 * (servo buzzing at an end = stalled against the mechanical limit).
 */
#define STEER_IN_MIN   (-32768)
#define STEER_IN_MAX   (32767)

static uint32_t steer_to_pulse_ns(int16_t steer)
{
    int32_t s = steer;
    if (s < STEER_IN_MIN) s = STEER_IN_MIN;
    if (s > STEER_IN_MAX) s = STEER_IN_MAX;

    int64_t span_in  = (int64_t)STEER_IN_MAX - STEER_IN_MIN;
    int64_t span_out = (int64_t)SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS;
    return (uint32_t)(SERVO_MIN_PULSE_NS + ((int64_t)(s - STEER_IN_MIN) * span_out) / span_in);
}

static void steering_thread_func(void *p1, void *p2, void *p3)
{
    bool holding = true;   /* boot = row 10 (isolated); servo_init centered it */

    while (1) {
        /* Wake on each new CTRL, or every 20 ms to re-check the row */
        int got_ctrl = (k_sem_take(&sem_ctrl, K_MSEC(STEERING_MAX_WAIT_MS)) == 0);

        /* Rows 6 (N1 lost) and 10 (N2 isolated): no valid wheel input.
         * Center the servo ONCE, then hold it there and ignore CTRL. */
        enum conn_row row = (enum conn_row)atomic_get(&conn_row_slow);
        if (conn_action_for(row).center_hold) {
            if (!holding) {
                servo_set_pulse_ns(SERVO_CENTER_PULSE_NS);
                holding = true;
            }
            continue;
        }

        /* Leaving the hold: discard any CTRL that arrived before the release
         * and take the position from the NEXT fresh CTRL. */
        if (holding) {
            holding = false;
            k_sem_reset(&sem_ctrl);
            continue;
        }

        if (!got_ctrl) {
            continue;   /* no new command: hold current position */
        }

        /* R3: steering keeps tracking in the system error state, unless
         * the steering zone itself is the failed zone. */
        if (atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
            continue;   /* self-failed: hold position */
        }

        uint32_t val   = (uint32_t)atomic_get(&mailbox_ctrl);
        int16_t  steer = (int16_t)(val & 0xFFFF);
        servo_set_pulse_ns(steer_to_pulse_ns(steer));
    }
}

K_THREAD_STACK_DEFINE(steering_stack, STEERING_THREAD_STACK_SIZE);
static struct k_thread steering_thread_data;

void steering_init(void)
{
    k_thread_create(&steering_thread_data, steering_stack,
                    K_THREAD_STACK_SIZEOF(steering_stack),
                    steering_thread_func,
                    NULL, NULL, NULL,
                    STEERING_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&steering_thread_data, "STEER");
}

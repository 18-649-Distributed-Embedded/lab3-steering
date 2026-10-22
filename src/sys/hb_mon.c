#include "sys/hb_mon.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include "sys/can_rx.h"
#include "core/can_catalog.h"
#include "core/cmd.h"
#include "core/log.h"

/* DEG LEDs */
static const struct gpio_dt_spec deg_a = GPIO_DT_SPEC_GET(DT_ALIAS(deg_a), gpios);
static const struct gpio_dt_spec deg_b = GPIO_DT_SPEC_GET(DT_ALIAS(deg_b), gpios);

/*
 * A peer is "missing on bus X" if its last HB on that bus was > 70ms ago.
 * 70ms = 3.5 × 20ms period — comfortably covers 3 missed heartbeats (60ms)
 * with margin for timer jitter.
 */
#define HB_MISS_TIMEOUT_MS   70U
#define HB_MON_PERIOD_MS     20U

#define HB_MON_THREAD_STACK  512
#define HB_MON_THREAD_PRIO   6

K_SEM_DEFINE(hb_mon_sem, 0, 1);

static void hb_mon_timer_isr(struct k_timer *t)
{
    k_sem_give(&hb_mon_sem);
}

K_TIMER_DEFINE(hb_mon_timer, hb_mon_timer_isr, NULL);

static inline bool peer_missing(atomic_t *mailbox)
{
    uint32_t last = (uint32_t)atomic_get(mailbox);
    uint32_t now  = k_uptime_get_32();
    return (last == 0) || ((now - last) > HB_MISS_TIMEOUT_MS);
}

static void hb_mon_thread_func(void *p1, void *p2, void *p3)
{
    while (1) {
        k_sem_take(&hb_mon_sem, K_FOREVER);

        bool n1_miss_a = peer_missing(&mailbox_hb_n1_a);
        bool n1_miss_b = peer_missing(&mailbox_hb_n1_b);
        bool n3_miss_a = peer_missing(&mailbox_hb_n3_a);
        bool n3_miss_b = peer_missing(&mailbox_hb_n3_b);

        /* DEG_A: any peer missing on bus A */
        bool deg_a_on = n1_miss_a || n3_miss_a;
        /* DEG_B: any peer missing on bus B */
        bool deg_b_on = n1_miss_b || n3_miss_b;

        /* ZONE_FAIL: peer missing on BOTH buses */
        bool zone_fail_n1 = n1_miss_a && n1_miss_b;
        bool zone_fail_n3 = n3_miss_a && n3_miss_b;

        /* Update bus health flags */
        if (deg_a_on) {
            atomic_set_bit(&bus_health_flags, BUS_FLAG_DEG_A);
        } else {
            atomic_clear_bit(&bus_health_flags, BUS_FLAG_DEG_A);
        }
        if (deg_b_on) {
            atomic_set_bit(&bus_health_flags, BUS_FLAG_DEG_B);
        } else {
            atomic_clear_bit(&bus_health_flags, BUS_FLAG_DEG_B);
        }

        /* Update safety flags */
        if (zone_fail_n1) {
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_N1);
        } else {
            atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_N1);
        }
        if (zone_fail_n3) {
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_N3);
        } else {
            atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_N3);
        }

        /* Check peer reported states */
        uint8_t n1_state = (uint8_t)atomic_get(&mailbox_n1_state);
        uint8_t n3_state = (uint8_t)atomic_get(&mailbox_n3_state);
        if (n1_state == CAN_ZONE_STATE_FAULT || n3_state == CAN_ZONE_STATE_FAULT) {
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_PEER_FAULT);
        } else if (!zone_fail_n1 && !zone_fail_n3 &&
                   n1_state != CAN_ZONE_STATE_FAULT && n3_state != CAN_ZONE_STATE_FAULT) {
            /* DEGRADED (one bus down) is not a fault: it must not block recovery */
            /* If peers are alive and reporting OK, clear peer fault */
            atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_PEER_FAULT);
        }

        /* Drive DEG LEDs */
        gpio_pin_set_dt(&deg_a, deg_a_on ? 1 : 0);
        gpio_pin_set_dt(&deg_b, deg_b_on ? 1 : 0);
    }
}

K_THREAD_STACK_DEFINE(hb_mon_stack, HB_MON_THREAD_STACK);
static struct k_thread hb_mon_thread_data;

void hb_mon_init(void)
{
    if (!device_is_ready(deg_a.port) || !device_is_ready(deg_b.port)) {
        DEBUG_PRINT("Error: DEG LED device not ready\n");
    } else {
        gpio_pin_configure_dt(&deg_a, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&deg_b, GPIO_OUTPUT_INACTIVE);
    }

    k_timer_start(&hb_mon_timer, K_MSEC(HB_MON_PERIOD_MS), K_MSEC(HB_MON_PERIOD_MS));

    k_thread_create(&hb_mon_thread_data, hb_mon_stack,
                    K_THREAD_STACK_SIZEOF(hb_mon_stack),
                    hb_mon_thread_func,
                    NULL, NULL, NULL,
                    HB_MON_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&hb_mon_thread_data, "HB_MON");
}


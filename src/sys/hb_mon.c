#include "sys/hb_mon.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include "sys/can_rx.h"
#include "core/can_catalog.h"
#include "core/cmd.h"
#include "core/conn_state.h"
#include "core/log.h"

/* DEG LEDs and ERR test point */
static const struct gpio_dt_spec deg_a  = GPIO_DT_SPEC_GET(DT_ALIAS(deg_a), gpios);
static const struct gpio_dt_spec deg_b  = GPIO_DT_SPEC_GET(DT_ALIAS(deg_b), gpios);
static const struct gpio_dt_spec err_tp = GPIO_DT_SPEC_GET(DT_ALIAS(err_tp), gpios);

/*
 * A connection (peer, bus) is lost when no heartbeat has been heard on it for
 * more than CONN_LOST_MS = 66 ms (3 missed heartbeats at 20 ms + 10 %).
 * The resulting 4-bit pattern picks one of the 12 rows in core/conn_state.h.
 */
#define HB_MON_PERIOD_MS     20U

#define HB_MON_THREAD_STACK  768
#define HB_MON_THREAD_PRIO   7

/* Boot: nothing heard yet = isolated (servo already centered by servo_init) */
atomic_t conn_bits_slow = ATOMIC_INIT(0);
atomic_t conn_row_slow  = ATOMIC_INIT(CONN_ROW_ISOLATED);

K_SEM_DEFINE(hb_mon_sem, 0, 1);

static void hb_mon_timer_isr(struct k_timer *t)
{
    k_sem_give(&hb_mon_sem);
}

K_TIMER_DEFINE(hb_mon_timer, hb_mon_timer_isr, NULL);

static uint32_t age_of(uint32_t now, atomic_t *mb)
{
    uint32_t last = (uint32_t)atomic_get(mb);
    return (last == 0) ? CONN_AGE_NEVER : (now - last);
}

static void set_flag(atomic_t *flags, int bit, bool on)
{
    if (on) {
        atomic_set_bit(flags, bit);
    } else {
        atomic_clear_bit(flags, bit);
    }
}

static void hb_mon_thread_func(void *p1, void *p2, void *p3)
{
    while (1) {
        k_sem_take(&hb_mon_sem, K_FOREVER);

        uint32_t now = k_uptime_get_32();
        uint8_t bits = conn_bits_from_ages(age_of(now, &mailbox_hb_n1_a),
                                           age_of(now, &mailbox_hb_n1_b),
                                           age_of(now, &mailbox_hb_n3_a),
                                           age_of(now, &mailbox_hb_n3_b),
                                           CONN_LOST_MS);
        enum conn_row row = conn_classify(bits);
        struct conn_action act = conn_action_for(row);

        atomic_set(&conn_bits_slow, (atomic_val_t)bits);
        atomic_set(&conn_row_slow,  (atomic_val_t)row);

        /* ZONE_FAIL: peer missing on BOTH buses (rows 6, 7, 10) -> error state */
        bool zone_fail_n1 = (bits & (CONN_N1A | CONN_N1B)) == 0u;
        bool zone_fail_n3 = (bits & (CONN_N3A | CONN_N3B)) == 0u;
        set_flag(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_N1, zone_fail_n1);
        set_flag(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_N3, zone_fail_n3);

        /* bus_health_flags = N2's OWN bus connection (R4.7), reported in HB_N2:
         * degraded when N2 hears nobody on that bus. */
        set_flag(&bus_health_flags, BUS_FLAG_DEG_A, conn_own_bus_a_out(bits));
        set_flag(&bus_health_flags, BUS_FLAG_DEG_B, conn_own_bus_b_out(bits));

        /* Check peer reported states */
        uint8_t n1_state = (uint8_t)atomic_get(&mailbox_n1_state);
        uint8_t n3_state = (uint8_t)atomic_get(&mailbox_n3_state);
        if (n1_state == CAN_ZONE_STATE_FAULT || n3_state == CAN_ZONE_STATE_FAULT) {
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_PEER_FAULT);
        } else if (!zone_fail_n1 && !zone_fail_n3) {
            /* Peers alive and neither reports FAULT (DEGRADED is not a fault) */
            atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_PEER_FAULT);
        }

        /* DEG LED patterns (1 Hz / 0.5 Hz phase from uptime) */
        gpio_pin_set_dt(&deg_a, deg_led_output(act.led_a, now) ? 1 : 0);
        gpio_pin_set_dt(&deg_b, deg_led_output(act.led_b, now) ? 1 : 0);

        /* ERR test point: high while this zone is in the error state */
        gpio_pin_set_dt(&err_tp, (atomic_get(&system_safety_flags) != 0) ? 1 : 0);
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
    if (device_is_ready(err_tp.port)) {
        gpio_pin_configure_dt(&err_tp, GPIO_OUTPUT_ACTIVE);   /* boot = error state */
    }

    k_timer_start(&hb_mon_timer, K_MSEC(HB_MON_PERIOD_MS), K_MSEC(HB_MON_PERIOD_MS));

    k_thread_create(&hb_mon_thread_data, hb_mon_stack,
                    K_THREAD_STACK_SIZEOF(hb_mon_stack),
                    hb_mon_thread_func,
                    NULL, NULL, NULL,
                    HB_MON_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&hb_mon_thread_data, "HB_MON");
}

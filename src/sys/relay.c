#include "sys/relay.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include "hw/can.h"
#include "sys/can_rx.h"
#include "core/can_catalog.h"
#include "core/conn_state.h"
#include "core/cmd.h"
#include "sys/hb_tx.h"
#include "core/log.h"

#define RELAY_THREAD_STACK  1024
#define RELAY_THREAD_PRIO   2      /* below the MCP2515 RX threads (1), above BLINK_F (3) */
#define RELAY_QUEUE_LEN     8

atomic_t relay_active_row = ATOMIC_INIT(CONN_ROW_NONE);
atomic_t relay_tx_count   = ATOMIC_INIT(0);
atomic_t relay_drop_count = ATOMIC_INIT(0);

static const struct gpio_dt_spec relay_tx_tp = GPIO_DT_SPEC_GET(DT_ALIAS(relay_tx), gpios);

struct relay_item {
    struct can_frame frame;
    uint8_t dest_bus;              /* CONN_BUS_A / CONN_BUS_B */
};

K_MSGQ_DEFINE(relay_q, sizeof(struct relay_item), RELAY_QUEUE_LEN, 4);

/* ── Prime cache: latest frame of each priming ID, PER BUS ──
 * Per bus matters: priming must replay the copy received on the source bus.
 * With a single latest-copy slot, the latest copy is often the one from the
 * other bus, priming skips it, and the stranded node waits one more period
 * (found by tests/host/sim_network). */
#define CACHE_HB_N1  0
#define CACHE_HB_N3  1
#define CACHE_CTRL   2
#define CACHE_COUNT  3

struct cache_entry {
    struct can_frame frame;
    uint32_t rx_time;              /* 0 = empty */
};

static struct cache_entry cache[CACHE_COUNT][2];   /* [id][bus] */
static struct k_spinlock cache_lock;   /* two RX threads write, RELAY reads */

static int cache_idx(uint32_t id)
{
    switch (id) {
    case CAN_ID_HB_N1: return CACHE_HB_N1;
    case CAN_ID_HB_N3: return CACHE_HB_N3;
    case CAN_ID_CTRL:  return CACHE_CTRL;
    default:           return -1;
    }
}

void relay_on_rx(int bus, const struct can_frame *frame)
{
    int ci = cache_idx(frame->id);
    if (ci >= 0) {
        k_spinlock_key_t key = k_spin_lock(&cache_lock);
        cache[ci][bus ? 1 : 0].frame   = *frame;
        cache[ci][bus ? 1 : 0].rx_time = k_uptime_get_32();
        k_spin_unlock(&cache_lock, key);
    }

    enum conn_row row = (enum conn_row)atomic_get(&relay_active_row);
    if (!conn_row_is_relay(row)) {
        return;
    }
    /* R4.5 / S5: a self-failed zone stops transmitting, relaying included */
    if (atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
        return;
    }

    int dest = conn_relay_dest_bus(row, frame->id, frame->data[1], bus);
    if (dest < 0) {
        return;
    }

    struct relay_item it = { .frame = *frame, .dest_bus = (uint8_t)dest };
    if (k_msgq_put(&relay_q, &it, K_NO_WAIT) != 0) {
        atomic_inc(&relay_drop_count);
    }
}

static void relay_send(const struct relay_item *it)
{
    /* Frames already queued when the test button fails the zone are dropped */
    if (atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
        atomic_inc(&relay_drop_count);
        return;
    }

    const struct device *dev = (it->dest_bus == CONN_BUS_A) ? can_a_dev : can_b_dev;
    struct can_frame f = it->frame;

    int ret = can_send_bus(dev, &f, CAN_TX_BUF_WAIT);
    gpio_pin_toggle_dt(&relay_tx_tp);       /* #6: edge after the send returns */
    atomic_inc((ret == 0) ? &relay_tx_count : &relay_drop_count);
}

/* On activation, forward the latest cached frames right away (see relay.h). */
static void relay_prime(enum conn_row row, uint32_t now)
{
    for (int ci = 0; ci < CACHE_COUNT; ci++) {
        for (int bus = 0; bus <= 1; bus++) {
            struct cache_entry e;
            k_spinlock_key_t key = k_spin_lock(&cache_lock);
            e = cache[ci][bus];
            k_spin_unlock(&cache_lock, key);

            if (e.rx_time == 0 || (now - e.rx_time) > CONN_LOST_MS) {
                continue;                   /* nothing fresh enough to replay */
            }
            int dest = conn_relay_dest_bus(row, e.frame.id, e.frame.data[1], bus);
            if (dest < 0) {
                continue;                   /* not from this source's bus */
            }
            struct relay_item it = { .frame = e.frame, .dest_bus = (uint8_t)dest };
            relay_send(&it);
        }
    }
}

static uint32_t age_of(uint32_t now, atomic_t *mb)
{
    uint32_t last = (uint32_t)atomic_get(mb);
    return (last == 0) ? CONN_AGE_NEVER : (now - last);
}

static void relay_thread_func(void *p1, void *p2, void *p3)
{
    struct relay_fsm fsm;
    relay_fsm_init(&fsm);

    uint32_t next_check = k_uptime_get_32() + CONN_RELAY_CHECK_MS;
    uint32_t probe_cyc = 0;

    while (1) {
        int32_t wait = (int32_t)(next_check - k_uptime_get_32());
        struct relay_item it;

        if (k_msgq_get(&relay_q, &it, (wait > 0) ? K_MSEC(wait) : K_NO_WAIT) == 0) {
            relay_send(&it);
        }

        uint32_t now = k_uptime_get_32();
        if ((int32_t)(now - next_check) < 0) {
            continue;
        }
        next_check += CONN_RELAY_CHECK_MS;
        if ((int32_t)(now - next_check) >= 0) {
            next_check = now + CONN_RELAY_CHECK_MS;   /* fell behind: resync */
        }

        uint8_t bits = conn_bits_from_ages(age_of(now, &mailbox_hb_n1_a),
                                           age_of(now, &mailbox_hb_n1_b),
                                           age_of(now, &mailbox_hb_n3_a),
                                           age_of(now, &mailbox_hb_n3_b),
                                           CONN_RELAY_TRIGGER_MS);
        enum conn_row fast_row = conn_classify(bits);

        /* A self-failed zone never relays: feed NONE (deactivates in 2 checks;
         * relay_on_rx already stops forwarding immediately). */
        if (atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
            fast_row = CONN_ROW_NONE;
        }

        /* ACK gate: both buses must ACK something after the probe */
        bool acked_both =
            (int32_t)((uint32_t)atomic_get(&can_last_ack_cyc[0]) - probe_cyc) > 0 &&
            (int32_t)((uint32_t)atomic_get(&can_last_ack_cyc[1]) - probe_cyc) > 0;
        bool probe;
        enum conn_row gated = relay_gate(&fsm, fast_row, acked_both, &probe);
        if (probe) {
            probe_cyc = k_cycle_get_32();
            hb_tx_send_probe();
        }

        enum conn_row prev = fsm.active;
        bool activated = relay_fsm_update(&fsm, gated);
        atomic_set(&relay_active_row, (atomic_val_t)fsm.active);

        if (activated) {
            relay_prime(fsm.active, now);   /* prime first: printk is slow */
            DEBUG_PRINT("RELAY: mode row %d active\n", (int)fsm.active);
        } else if (prev != CONN_ROW_NONE && fsm.active == CONN_ROW_NONE) {
            DEBUG_PRINT("RELAY: off\n");
        }
    }
}

K_THREAD_STACK_DEFINE(relay_stack, RELAY_THREAD_STACK);
static struct k_thread relay_thread_data;

void relay_init(void)
{
    if (device_is_ready(relay_tx_tp.port)) {
        gpio_pin_configure_dt(&relay_tx_tp, GPIO_OUTPUT_INACTIVE);
    }

    k_thread_create(&relay_thread_data, relay_stack,
                    K_THREAD_STACK_SIZEOF(relay_stack),
                    relay_thread_func,
                    NULL, NULL, NULL,
                    RELAY_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&relay_thread_data, "RELAY");
}

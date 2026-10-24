#include "sys/hb_tx.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "hw/can.h"
#include "core/can_catalog.h"
#include "core/cmd.h"
#include "core/log.h"
#include "sys/debug_status.h"
#include <zephyr/spinlock.h>
#include <string.h>

#define HB_TX_THREAD_STACK  512
#define HB_TX_THREAD_PRIO   6      /* deadline 20 ms */
#define HB_TX_PERIOD_MS     20

K_SEM_DEFINE(hb_tx_sem, 0, 1);

/* Last HB_N2 payload, re-sent by hb_tx_send_probe() (relay ACK gate) */
static uint8_t last_payload[8];
static bool last_payload_valid;
static struct k_spinlock last_payload_lock;

static void hb_tx_timer_isr(struct k_timer *t)
{
    k_sem_give(&hb_tx_sem);
}

K_TIMER_DEFINE(hb_tx_timer, hb_tx_timer_isr, NULL);

static void hb_tx_thread_func(void *p1, void *p2, void *p3)
{
    uint8_t seq = 0;

    while (1) {
        k_sem_take(&hb_tx_sem, K_FOREVER);

        /* Report THIS zone's own health, not the system error state.
         * A zone in error only because of a peer reports OK about itself;
         * otherwise every node echoes FAULT back and they keep each other
         * latched in PEER_FAULT forever. */
        atomic_val_t busfl = atomic_get(&bus_health_flags);

        uint8_t zone_state;
        if (atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
            zone_state = CAN_ZONE_STATE_FAULT;   /* (HB is suppressed in this state) */
        } else if (busfl != 0) {
            zone_state = CAN_ZONE_STATE_DEGRADED;
        } else {
            zone_state = CAN_ZONE_STATE_OK;
        }

        /* bus_health: bit0=A ok if DEG_A not set, bit1=B ok if DEG_B not set */
        uint8_t bus_health = 0;
        if (!atomic_test_bit(&bus_health_flags, BUS_FLAG_DEG_A)) {
            bus_health |= CAN_BUS_HEALTH_A;
        }
        if (!atomic_test_bit(&bus_health_flags, BUS_FLAG_DEG_B)) {
            bus_health |= CAN_BUS_HEALTH_B;
        }

        uint8_t payload[8] = {
            can_byte0(zone_state, seq),
            zone_state,
            bus_health,
            0x00, 0x00, 0x00, 0x00, 0x00,
        };

        /* R4.5 / S5: a self-failed zone (test button) stops its heartbeat
         * on both buses. Every other error state keeps transmitting. */
        bool sent = false;
        if (!atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
            k_spinlock_key_t key = k_spin_lock(&last_payload_lock);
            memcpy(last_payload, payload, sizeof(last_payload));
            last_payload_valid = true;
            k_spin_unlock(&last_payload_lock, key);

            can_send_both(CAN_ID_HB_N2, 8, payload);
            sent = true;
        }
        debug_status_on_heartbeat(seq, sent);

        seq = (seq + 1) & 0x0F;
    }
}

bool hb_tx_send_probe(void)
{
    uint8_t p[8];

    if (atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
        return false;              /* a failed zone stays silent */
    }
    k_spinlock_key_t key = k_spin_lock(&last_payload_lock);
    bool ok = last_payload_valid;
    memcpy(p, last_payload, sizeof(p));
    k_spin_unlock(&last_payload_lock, key);

    if (ok) {
        /* bit-identical to a heartbeat already sent: receivers' dedup drops
         * it as a duplicate, but its ACK proves the bus is alive */
        can_send_both(CAN_ID_HB_N2, 8, p);
    }
    return ok;
}

K_THREAD_STACK_DEFINE(hb_tx_stack, HB_TX_THREAD_STACK);
static struct k_thread hb_tx_thread_data;

void hb_tx_init(void)
{
    k_timer_start(&hb_tx_timer, K_MSEC(HB_TX_PERIOD_MS), K_MSEC(HB_TX_PERIOD_MS));

    k_thread_create(&hb_tx_thread_data, hb_tx_stack,
                    K_THREAD_STACK_SIZEOF(hb_tx_stack),
                    hb_tx_thread_func,
                    NULL, NULL, NULL,
                    HB_TX_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&hb_tx_thread_data, "HB_TX");
}

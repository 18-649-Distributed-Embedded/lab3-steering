#include "sys/debug_status.h"

#ifdef CONFIG_APP_DEBUG_LOGGING

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>
#include "hw/can.h"
#include "hw/servo.h"
#include "sys/can_rx.h"
#include "sys/hb_mon.h"
#include "sys/relay.h"
#include "core/conn_state.h"
#include "core/cmd.h"

#define STAT_STALL_MS  100   /* print anyway if no heartbeat tick for this long */
#define STAT_STACK     1024
#define STAT_PRIO      10    /* lowest app priority: never delays real work */
#define AGE_CAP        9999U /* ages shown capped; 9999 also = never heard */

K_SEM_DEFINE(stat_sem, 0, 1);
static atomic_t stat_hb = ATOMIC_INIT(0);   /* [seq << 1 | sent] of last heartbeat */

void debug_status_on_heartbeat(uint8_t hb_seq, bool sent)
{
    atomic_set(&stat_hb, ((atomic_val_t)hb_seq << 1) | (sent ? 1 : 0));
    k_sem_give(&stat_sem);
}

static const char *st(enum can_state s)
{
    switch (s) {
    case CAN_STATE_ERROR_ACTIVE:  return "ACT";
    case CAN_STATE_ERROR_WARNING: return "WRN";
    case CAN_STATE_ERROR_PASSIVE: return "PAS";
    case CAN_STATE_BUS_OFF:       return "OFF";
    case CAN_STATE_STOPPED:       return "STP";
    default:                      return "???";
    }
}

static uint32_t age(uint32_t now, uint32_t last)
{
    if (last == 0) return AGE_CAP;
    uint32_t a = now - last;
    return (a > AGE_CAP) ? AGE_CAP : a;
}

static void stat_thread(void *p1, void *p2, void *p3)
{
    atomic_val_t prev_a = 0, prev_b = 0;

    while (1) {
        bool stalled = (k_sem_take(&stat_sem, K_MSEC(STAT_STALL_MS)) != 0);
        uint32_t now = k_uptime_get_32();

        enum can_state sa = CAN_STATE_STOPPED, sb = CAN_STATE_STOPPED;
        struct can_bus_err_cnt ea = {0}, eb = {0};
        can_get_state(can_a_dev, &sa, &ea);
        can_get_state(can_b_dev, &sb, &eb);

        atomic_val_t ra = atomic_get(&can_rx_count_a);
        atomic_val_t rb = atomic_get(&can_rx_count_b);
        atomic_val_t hb = atomic_get(&stat_hb);
        uint32_t ctrl   = (uint32_t)atomic_get(&mailbox_ctrl);

        uint8_t bits = (uint8_t)atomic_get(&conn_bits_slow);
        enum conn_row row = (enum conn_row)atomic_get(&conn_row_slow);
        struct conn_action act = conn_action_for(row);
        int rly = (int)atomic_get(&relay_active_row);

        /* conn = N1A N1B N3A N3B (1 heard / 0 lost), row = spec row 1..12,
         * DEG = LED mode (- off, f 1 Hz, s 0.5 Hz, S steady),
         * rly = active relay row (0 = none) / frames forwarded since boot */
        printk("[HB %2u t=%u] A:%s %u/%u B:%s %u/%u rx %ld/%ld sf=%02lx"
               " conn %c%c%c%c row %2d DEG %c%c rly %d/%ld"
               " | CTRL %u st=%d srv=%u | N1 %u/%u N3 %u/%u%s\n",
               (unsigned)((hb >> 1) & 0x0F), now,
               st(sa), ea.tx_err_cnt, ea.rx_err_cnt,
               st(sb), eb.tx_err_cnt, eb.rx_err_cnt,
               (long)(ra - prev_a), (long)(rb - prev_b),
               (unsigned long)atomic_get(&system_safety_flags),
               (bits & CONN_N1A) ? '1' : '0', (bits & CONN_N1B) ? '1' : '0',
               (bits & CONN_N3A) ? '1' : '0', (bits & CONN_N3B) ? '1' : '0',
               (int)row, deg_led_char(act.led_a), deg_led_char(act.led_b),
               rly, (long)atomic_get(&relay_tx_count),
               age(now, last_ctrl_time), (int)(int16_t)(ctrl & 0xFFFF),
               servo_get_pulse_ns() / 1000U,
               age(now, (uint32_t)atomic_get(&mailbox_hb_n1_a)),
               age(now, (uint32_t)atomic_get(&mailbox_hb_n1_b)),
               age(now, (uint32_t)atomic_get(&mailbox_hb_n3_a)),
               age(now, (uint32_t)atomic_get(&mailbox_hb_n3_b)),
               stalled ? " HB STALLED" : ((hb & 1) ? "" : " SILENT"));

        prev_a = ra;
        prev_b = rb;
    }
}

K_THREAD_STACK_DEFINE(stat_stack, STAT_STACK);
static struct k_thread stat_data;

void debug_status_init(void)
{
    k_thread_create(&stat_data, stat_stack, K_THREAD_STACK_SIZEOF(stat_stack),
                    stat_thread, NULL, NULL, NULL, STAT_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&stat_data, "STAT");
}

#endif /* CONFIG_APP_DEBUG_LOGGING */

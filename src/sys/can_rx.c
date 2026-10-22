#include "sys/can_rx.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include "hw/can.h"
#include "core/can_catalog.h"
#include "core/cmd.h"
#include "core/log.h"

static const struct gpio_dt_spec can0_rx_tp = GPIO_DT_SPEC_GET(DT_ALIAS(can0_rx), gpios);
static const struct gpio_dt_spec can1_rx_tp = GPIO_DT_SPEC_GET(DT_ALIAS(can1_rx), gpios);

/* ── IPC primitives ─────────────────────────────────────── */
K_SEM_DEFINE(sem_ctrl,  0, 1);
K_SEM_DEFINE(sem_blink, 0, 1);

atomic_t mailbox_ctrl     = ATOMIC_INIT(0);         /* steer=0 (center), buttons=0 */
atomic_t mailbox_blinker  = ATOMIC_INIT(0);
atomic_t mailbox_hb_n1_a  = ATOMIC_INIT(0);
atomic_t mailbox_hb_n1_b  = ATOMIC_INIT(0);
atomic_t mailbox_hb_n3_a  = ATOMIC_INIT(0);
atomic_t mailbox_hb_n3_b  = ATOMIC_INIT(0);
atomic_t mailbox_n1_state = ATOMIC_INIT(CAN_ZONE_STATE_OK);
atomic_t mailbox_n3_state = ATOMIC_INIT(CAN_ZONE_STATE_OK);

atomic_t can_rx_count_a = ATOMIC_INIT(0);
atomic_t can_rx_count_b = ATOMIC_INIT(0);

volatile uint32_t last_ctrl_time = 0;

/* ── Dedup table ─────────────────────────────────────────── */
#define DEDUP_CTRL    0
#define DEDUP_FAULT   1
#define DEDUP_HB_N1   2
#define DEDUP_HB_N3   3
#define DEDUP_BLINK   4
#define DEDUP_COUNT   5

static uint8_t last_seq[DEDUP_COUNT];
static uint32_t last_rx_time[DEDUP_COUNT];

static int id_to_dedup_idx(uint32_t id)
{
    switch (id) {
    case CAN_ID_CTRL:   return DEDUP_CTRL;
    case CAN_ID_FAULT:  return DEDUP_FAULT;
    case CAN_ID_HB_N1:  return DEDUP_HB_N1;
    case CAN_ID_HB_N3:  return DEDUP_HB_N3;
    case CAN_ID_BLINK:  return DEDUP_BLINK;
    default:            return -1;
    }
}

/* Returns true if frame should be accepted (not a duplicate) */
static bool dedup_accept(uint32_t id, uint8_t seq)
{
    int idx = id_to_dedup_idx(id);
    if (idx < 0) {
        return false;   /* unknown ID — ignore */
    }

    uint32_t now = k_uptime_get_32();
    /* If more than 100ms since last frame of this ID, reset dedup state */
    if ((now - last_rx_time[idx]) > 100U) {
        last_seq[idx] = 0xFF;
    }

    /* Accept only frames NEWER than the last accepted one (4-bit modular).
     * diff 0     = duplicate copy from the other bus
     * diff 8..15 = an OLDER copy arriving late on the other bus; accepting
     *              it would roll state back (e.g. the servo snapping back).
     * After a gap > 100 ms the state is reset above, so a rebooted sender's
     * new sequence is always accepted. */
    if (last_seq[idx] != 0xFF) {
        uint8_t diff = (uint8_t)(seq - last_seq[idx]) & 0x0F;
        if (diff == 0 || diff > 7) {
            return false;
        }
    }

    last_seq[idx] = seq;
    last_rx_time[idx] = now;
    return true;
}

/* ── Dispatch Callback ───────────────────────────────────── */
/*
 * Called from the MCP2515 driver thread (priority 2).
 * user_data: 0 = bus A, 1 = bus B.
 */
static void rx_dispatch(const struct device *dev,
                        struct can_frame *frame,
                        void *user_data)
{
    int bus = (int)(uintptr_t)user_data;    /* 0 = A (CAN0), 1 = B (CAN1) */
    const struct gpio_dt_spec *rx_tp = (bus == 0) ? &can0_rx_tp : &can1_rx_tp;

    /* Set RX TP HIGH immediately after SPI read callback fires */
    gpio_pin_set_dt(rx_tp, 1);

    atomic_inc((bus == 0) ? &can_rx_count_a : &can_rx_count_b);

    uint32_t now = k_uptime_get_32();

    /* Track reception timestamp per bus before deduplication */
    if (frame->id == CAN_ID_HB_N1) {
        if (bus == 0) atomic_set(&mailbox_hb_n1_a, (atomic_val_t)now);
        else          atomic_set(&mailbox_hb_n1_b, (atomic_val_t)now);
    } else if (frame->id == CAN_ID_HB_N3) {
        if (bus == 0) atomic_set(&mailbox_hb_n3_a, (atomic_val_t)now);
        else          atomic_set(&mailbox_hb_n3_b, (atomic_val_t)now);
    }

    uint8_t seq = can_seq(frame->data[0]);
    if (dedup_accept(frame->id, seq)) {
        switch (frame->id) {

        case CAN_ID_CTRL: {
            /* CTRL payload: [0]=flags|seq [1..2]=steer int16 [3]=throttle
             * [4]=brake [5]=buttons [6]=CRC-8 [7]=unused
             * Steer is BIG-endian on the wire (byte 1 = high, byte 2 = low),
             * confirmed against N1 with candump.
             * TODO: CRC-8 + range check -> FAULT (R3, checkoff item 10). */
            int16_t steer   = (int16_t)sys_get_be16(&frame->data[1]);
            uint8_t buttons = frame->data[5];
            uint32_t val = ((uint32_t)buttons << 16) |
                           (uint32_t)(uint16_t)steer;
            atomic_set(&mailbox_ctrl, (atomic_val_t)val);
            last_ctrl_time = now;
            k_sem_give(&sem_ctrl);
            break;
        }

        case CAN_ID_FAULT: {
            uint8_t zone_id = frame->data[1];
            uint8_t reason  = frame->data[2];
            if (reason == 0) {
                /* Fault cleared */
                atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_PEER_FAULT);
                DEBUG_PRINT("FAULT CLEARED: zone=%u\n", zone_id);
            } else {
                atomic_set_bit(&system_safety_flags, SAFETY_FLAG_PEER_FAULT);
                DEBUG_PRINT("FAULT RX: zone=%u reason=%u\n", zone_id, reason);
            }
            break;
        }

        case CAN_ID_HB_N1: {
            uint8_t zone_state = frame->data[1];
            atomic_set(&mailbox_n1_state, (atomic_val_t)zone_state);
            break;
        }

        case CAN_ID_HB_N3: {
            uint8_t zone_state = frame->data[1];
            atomic_set(&mailbox_n3_state, (atomic_val_t)zone_state);
            break;
        }

        case CAN_ID_BLINK: {
            uint8_t blink_state = frame->data[1];
            uint8_t phase_bit   = frame->data[2];
            uint32_t val = ((uint32_t)blink_state << 8) | (uint32_t)phase_bit;
            atomic_set(&mailbox_blinker, (atomic_val_t)val);
            k_sem_give(&sem_blink);
            break;
        }

        default:
            break;
        }
    }

    /* Set RX TP LOW once ingestion/dispatch is finished */
    gpio_pin_set_dt(rx_tp, 0);
}

/* ── Init ────────────────────────────────────────────────── */
void can_rx_init(void)
{
    /* Initialize RX test point GPIOs */
    if (device_is_ready(can0_rx_tp.port)) {
        gpio_pin_configure_dt(&can0_rx_tp, GPIO_OUTPUT_INACTIVE);
    }
    if (device_is_ready(can1_rx_tp.port)) {
        gpio_pin_configure_dt(&can1_rx_tp, GPIO_OUTPUT_INACTIVE);
    }

    for (int i = 0; i < DEDUP_COUNT; i++) {
        last_seq[i] = 0xFF;   /* "never seen" sentinel */
        last_rx_time[i] = 0;
    }

    static const struct can_filter f_all = {
        .id    = 0x000,
        .mask  = 0x000,
        .flags = 0,
    };

    int ret;

    ret = can_add_rx_filter(can_a_dev, rx_dispatch, (void *)0, &f_all);
    if (ret < 0) {
        DEBUG_PRINT("Error: CAN A RX filter failed (%d)\n", ret);
    }

    ret = can_add_rx_filter(can_b_dev, rx_dispatch, (void *)1, &f_all);
    if (ret < 0) {
        DEBUG_PRINT("Error: CAN B RX filter failed (%d)\n", ret);
    }

    DEBUG_PRINT("CAN RX dispatch registered on both buses\n");
}


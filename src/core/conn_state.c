#include "core/conn_state.h"
#include "core/can_catalog.h"

uint8_t conn_bits_from_ages(uint32_t n1a_age, uint32_t n1b_age,
                            uint32_t n3a_age, uint32_t n3b_age,
                            uint32_t threshold_ms)
{
    uint8_t bits = 0;

    /* CONN_AGE_NEVER (UINT32_MAX) is always > threshold -> 0 */
    if (n1a_age <= threshold_ms) bits |= CONN_N1A;
    if (n1b_age <= threshold_ms) bits |= CONN_N1B;
    if (n3a_age <= threshold_ms) bits |= CONN_N3A;
    if (n3b_age <= threshold_ms) bits |= CONN_N3B;
    return bits;
}

enum conn_row conn_classify(uint8_t bits)
{
    bits &= CONN_ALL;

    if (bits == 0u) {
        return CONN_ROW_ISOLATED;                       /* row 10 */
    }
    if ((bits & (CONN_N1A | CONN_N1B)) == 0u) {
        return CONN_ROW_N1_LOST;                        /* row 6  */
    }
    if ((bits & (CONN_N3A | CONN_N3B)) == 0u) {
        return CONN_ROW_N3_LOST;                        /* row 7  */
    }

    /* Both peers heard on at least one bus: 9 remaining patterns */
    switch (bits) {
    case 0xF: return CONN_ROW_NORMAL;                   /* 1 1 1 1 */
    case 0x7: return CONN_ROW_N1A_OUT;                  /* 0 1 1 1 */
    case 0xB: return CONN_ROW_N1B_OUT;                  /* 1 0 1 1 */
    case 0xD: return CONN_ROW_N3A_OUT;                  /* 1 1 0 1 */
    case 0xE: return CONN_ROW_N3B_OUT;                  /* 1 1 1 0 */
    case 0x6: return CONN_ROW_RELAY_1;                  /* 0 1 1 0 */
    case 0x9: return CONN_ROW_RELAY_2;                  /* 1 0 0 1 */
    case 0x5: return CONN_ROW_OWN_A_OUT;                /* 0 1 0 1 */
    case 0xA: return CONN_ROW_OWN_B_OUT;                /* 1 0 1 0 */
    default:  return CONN_ROW_NORMAL;                   /* unreachable */
    }
}

struct conn_action conn_action_for(enum conn_row row)
{
    struct conn_action a = { DEG_LED_OFF, DEG_LED_OFF, false };

    switch (row) {
    case CONN_ROW_NORMAL:
        break;
    case CONN_ROW_N1A_OUT:
    case CONN_ROW_N3A_OUT:
        a.led_a = DEG_LED_FLASH_1HZ;
        break;
    case CONN_ROW_N1B_OUT:
    case CONN_ROW_N3B_OUT:
        a.led_b = DEG_LED_FLASH_1HZ;
        break;
    case CONN_ROW_N1_LOST:
        a.led_a = DEG_LED_STEADY;
        a.led_b = DEG_LED_STEADY;
        a.center_hold = true;
        break;
    case CONN_ROW_N3_LOST:
        a.led_a = DEG_LED_STEADY;
        a.led_b = DEG_LED_STEADY;
        break;
    case CONN_ROW_RELAY_1:
    case CONN_ROW_RELAY_2:
        a.led_a = DEG_LED_FLASH_0P5HZ;
        a.led_b = DEG_LED_FLASH_0P5HZ;
        break;
    case CONN_ROW_ISOLATED:
        a.led_a = DEG_LED_STEADY;
        a.led_b = DEG_LED_STEADY;
        a.center_hold = true;
        break;
    case CONN_ROW_OWN_A_OUT:
        a.led_a = DEG_LED_STEADY;
        break;
    case CONN_ROW_OWN_B_OUT:
        a.led_b = DEG_LED_STEADY;
        break;
    case CONN_ROW_NONE:
    default:
        break;
    }
    return a;
}

bool deg_led_output(enum deg_led_mode mode, uint32_t now_ms)
{
    switch (mode) {
    case DEG_LED_FLASH_1HZ:   return (now_ms % 1000u) < 500u;
    case DEG_LED_FLASH_0P5HZ: return (now_ms % 2000u) < 1000u;
    case DEG_LED_STEADY:      return true;
    case DEG_LED_OFF:
    default:                  return false;
    }
}

char deg_led_char(enum deg_led_mode mode)
{
    switch (mode) {
    case DEG_LED_FLASH_1HZ:   return 'f';
    case DEG_LED_FLASH_0P5HZ: return 's';
    case DEG_LED_STEADY:      return 'S';
    case DEG_LED_OFF:
    default:                  return '-';
    }
}

/* ── Relay ──────────────────────────────────────────────── */

#define ORIGIN_NONE 0
#define ORIGIN_N1   1
#define ORIGIN_N3   3

static int frame_origin(uint32_t id, uint8_t zone)
{
    switch (id) {
    case CAN_ID_CTRL:
    case CAN_ID_BLINK:
    case CAN_ID_HB_N1:
        return ORIGIN_N1;
    case CAN_ID_HB_N3:
        return ORIGIN_N3;
    case CAN_ID_FAULT:
        if (zone == CAN_ZONE_N1) return ORIGIN_N1;
        if (zone == CAN_ZONE_N3) return ORIGIN_N3;
        return ORIGIN_NONE;                 /* N2's own FAULT, or unknown */
    default:
        return ORIGIN_NONE;                 /* HB_N2, FORCE, anything else */
    }
}

int conn_relay_dest_bus(enum conn_row active_row, uint32_t id,
                        uint8_t zone, int rx_bus)
{
    int origin = frame_origin(id, zone);
    int n1_from, n3_from;   /* bus each source is still heard on */

    if (origin == ORIGIN_NONE) {
        return -1;
    }

    if (active_row == CONN_ROW_RELAY_2) {          /* 1 0 0 1 */
        n1_from = CONN_BUS_A;                      /* N1 only on A */
        n3_from = CONN_BUS_B;                      /* N3 only on B */
    } else if (active_row == CONN_ROW_RELAY_1) {   /* 0 1 1 0 */
        n1_from = CONN_BUS_B;                      /* N1 only on B */
        n3_from = CONN_BUS_A;                      /* N3 only on A */
    } else {
        return -1;                                 /* not in a relay mode */
    }

    int from = (origin == ORIGIN_N1) ? n1_from : n3_from;
    if (rx_bus != from) {
        return -1;      /* never relay back onto the bus it came from */
    }
    return (from == CONN_BUS_A) ? CONN_BUS_B : CONN_BUS_A;
}

void relay_fsm_init(struct relay_fsm *f)
{
    f->active = CONN_ROW_NONE;
    f->candidate = CONN_ROW_NONE;
    f->streak = 0;
}

enum conn_row relay_gate(const struct relay_fsm *f, enum conn_row fast_row,
                         bool acked_both_since_probe, bool *probe)
{
    *probe = false;
    if (!conn_row_is_relay(fast_row) || f->active == fast_row) {
        return fast_row;
    }
    if (f->candidate != fast_row) {
        *probe = true;                 /* first sighting: probe both buses */
        return fast_row;
    }
    return acked_both_since_probe ? fast_row : CONN_ROW_NONE;
}

bool relay_fsm_update(struct relay_fsm *f, enum conn_row fast_row)
{
    enum conn_row cand = conn_row_is_relay(fast_row) ? fast_row : CONN_ROW_NONE;

    if (cand == f->candidate) {
        if (f->streak < 255u) {
            f->streak++;
        }
    } else {
        f->candidate = cand;
        f->streak = 1;
    }

    if (f->streak >= CONN_RELAY_STABLE_N && cand != f->active) {
        f->active = cand;
        return (cand != CONN_ROW_NONE);
    }
    return false;
}

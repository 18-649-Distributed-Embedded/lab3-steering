#ifndef CORE_CONN_STATE_H
#define CORE_CONN_STATE_H

/*
 * N2 connection-state logic (signed-off spec, 2026-10-24).
 *
 * Pure C: no Zephyr calls, so the same code runs on the STM32 and in the
 * host unit tests (tests/host).
 *
 * Inputs are four connection bits, one per (peer, bus):
 *   N1A N1B N3A N3B   1 = heartbeat heard recently, 0 = not heard
 *
 * Two thresholds use the same classifier:
 *   - display/state  (HB_MON, 20 ms):  0 = no HB for > 66 ms (3 missed)
 *   - relay trigger  (RELAY, 5 ms):    0 = no HB for > 30 ms (1.5 HB)
 *
 * The 16 bit patterns map onto 12 rows:
 *   #  N1A N1B N3A N3B  meaning                     relay       DEG_A   DEG_B   steering
 *   1   1   1   1   1   normal                      none        off     off     track
 *   2   0   1   1   1   N1's A connection faulty    none        1 Hz    off     track
 *   3   1   0   1   1   N1's B connection faulty    none        off     1 Hz    track
 *   4   1   1   0   1   N3's A connection faulty    none        1 Hz    off     track
 *   5   1   1   1   0   N3's B connection faulty    none        off     1 Hz    track
 *   6   0   0   x   x   N1 lost (+hazards)          none        steady  steady  center, hold
 *       (x x = 1 1, 0 1 or 1 0; 0 0 is row 10)
 *   7   x   x   0   0   N3 lost (+hazards)          none        steady  steady  track
 *       (x x = 1 1, 0 1 or 1 0)
 *   8   0   1   1   0   relay mode 1                N1 B->A,    0.5 Hz  0.5 Hz  track
 *                                                   N3 A->B
 *   9   1   0   0   1   relay mode 2                N1 A->B,    0.5 Hz  0.5 Hz  track
 *                                                   N3 B->A
 *  10   0   0   0   0   N2 isolated (+hazards)      none        steady  steady  center, hold
 *  11   0   1   0   1   N2's own A out / A lost     none        steady  off     track
 *  12   1   0   1   0   N2's own B out / B lost     none        off     steady  track
 *
 * 1 Hz   = 500 ms on / 500 ms off.   0.5 Hz = 1000 ms on / 1000 ms off.
 * Hazards in rows 6, 7, 10 come from the ZONE_FAIL flags (system error state).
 */

#include <stdint.h>
#include <stdbool.h>

/* ── Connection bits ─────────────────────────────────────── */
#define CONN_N1A   0x8u
#define CONN_N1B   0x4u
#define CONN_N3A   0x2u
#define CONN_N3B   0x1u
#define CONN_ALL   0xFu

/* Age value meaning "never heard since boot" */
#define CONN_AGE_NEVER   UINT32_MAX

/* Thresholds (ms) */
#define CONN_LOST_MS          66u   /* 3 missed heartbeats -> 0 (state/LEDs) */
#define CONN_RELAY_TRIGGER_MS 30u   /* 1.5 heartbeats      -> 0 (relay)      */
#define CONN_RELAY_CHECK_MS    5u   /* relay pattern check period            */
#define CONN_RELAY_STABLE_N    2u   /* consecutive checks before (de)activate */

/* ── Rows ───────────────────────────────────────────────── */
enum conn_row {
    CONN_ROW_NONE       = 0,   /* "no relay" for the relay FSM */
    CONN_ROW_NORMAL     = 1,
    CONN_ROW_N1A_OUT    = 2,
    CONN_ROW_N1B_OUT    = 3,
    CONN_ROW_N3A_OUT    = 4,
    CONN_ROW_N3B_OUT    = 5,
    CONN_ROW_N1_LOST    = 6,
    CONN_ROW_N3_LOST    = 7,
    CONN_ROW_RELAY_1    = 8,
    CONN_ROW_RELAY_2    = 9,
    CONN_ROW_ISOLATED   = 10,
    CONN_ROW_OWN_A_OUT  = 11,
    CONN_ROW_OWN_B_OUT  = 12,
};

enum deg_led_mode {
    DEG_LED_OFF = 0,
    DEG_LED_FLASH_1HZ,     /* 500 ms on / 500 ms off   */
    DEG_LED_FLASH_0P5HZ,   /* 1000 ms on / 1000 ms off */
    DEG_LED_STEADY,
};

struct conn_action {
    enum deg_led_mode led_a;
    enum deg_led_mode led_b;
    bool center_hold;      /* rows 6 and 10: center the servo once, then hold */
};

/* Bus indices used by the relay helpers */
#define CONN_BUS_A  0
#define CONN_BUS_B  1

/* ── Classification ─────────────────────────────────────── */

/** One bit per (peer, bus): set if age <= threshold_ms. CONN_AGE_NEVER = 0. */
uint8_t conn_bits_from_ages(uint32_t n1a_age, uint32_t n1b_age,
                            uint32_t n3a_age, uint32_t n3b_age,
                            uint32_t threshold_ms);

/** Map the 4-bit pattern (0..15) to its row (1..12). */
enum conn_row conn_classify(uint8_t bits);

/** LED modes and steering behavior for a row. */
struct conn_action conn_action_for(enum conn_row row);

/** True if N2 itself has lost bus A (hears nobody on A). */
static inline bool conn_own_bus_a_out(uint8_t bits)
{
    return (bits & (CONN_N1A | CONN_N3A)) == 0u;
}

/** True if N2 itself has lost bus B (hears nobody on B). */
static inline bool conn_own_bus_b_out(uint8_t bits)
{
    return (bits & (CONN_N1B | CONN_N3B)) == 0u;
}

/** LED level for a mode at time now_ms (free-running, phase from uptime). */
bool deg_led_output(enum deg_led_mode mode, uint32_t now_ms);

/** Short tag for the STAT line: '-' off, 'f' 1 Hz, 's' 0.5 Hz, 'S' steady. */
char deg_led_char(enum deg_led_mode mode);

/* ── Relay ──────────────────────────────────────────────── */

static inline bool conn_row_is_relay(enum conn_row row)
{
    return (row == CONN_ROW_RELAY_1) || (row == CONN_ROW_RELAY_2);
}

/**
 * Destination bus for a received frame, or -1 if it must not be relayed.
 *   active_row: CONN_ROW_RELAY_1 / CONN_ROW_RELAY_2, anything else = no relay
 *   id:         CAN ID of the received frame
 *   zone:       data[1] of the frame (used only for FAULT, to find its origin)
 *   rx_bus:     CONN_BUS_A or CONN_BUS_B (where N2 received it)
 * Only N1-originated (CTRL, BLINK, HB_N1, FAULT from N1) and N3-originated
 * (HB_N3, FAULT from N3) frames are relayed, and only from the bus that
 * node is still on, to the bus it is missing from. N2's own frames never.
 */
int conn_relay_dest_bus(enum conn_row active_row, uint32_t id,
                        uint8_t zone, int rx_bus);

/**
 * Relay activation with hysteresis: the candidate relay state (relay row or
 * NONE) must be seen on CONN_RELAY_STABLE_N consecutive checks before it is
 * committed, both to activate and to deactivate.
 */
struct relay_fsm {
    enum conn_row active;     /* committed: RELAY_1, RELAY_2 or NONE */
    enum conn_row candidate;  /* last observed candidate              */
    uint8_t       streak;     /* consecutive checks with candidate    */
};

void relay_fsm_init(struct relay_fsm *f);

/**
 * Feed one check. fast_row = conn_classify(bits at the 30 ms threshold).
 * Returns true exactly when a relay mode becomes active (NONE -> RELAY_x or
 * RELAY_x -> RELAY_y), so the caller can prime the relay.
 */
bool relay_fsm_update(struct relay_fsm *f, enum conn_row fast_row);

/**
 * ACK gate in front of relay_fsm_update().
 *
 * Why: when N2 loses one of its OWN buses, its two peers' heartbeats on that
 * bus age out up to one heartbeat period apart, and in between the pattern
 * looks exactly like a relay row (e.g. 1 0 0 1). A truly split network still
 * ACKs N2's frames on BOTH buses (each bus has another node); a bus N2 has
 * lost does not. So:
 *   - first sighting of a relay row: *probe = true -> caller sends a probe
 *     (repeat of the last HB_N2) on both buses and records the time
 *   - next check: the relay row is passed through only if both buses ACKed
 *     something after the probe (acked_both_since_probe); otherwise it is
 *     fed as CONN_ROW_NONE, so the next sighting probes again
 *   - non-relay rows and an already-active relay row pass straight through
 * Activation still takes CONN_RELAY_STABLE_N (2) checks.
 */
enum conn_row relay_gate(const struct relay_fsm *f, enum conn_row fast_row,
                         bool acked_both_since_probe, bool *probe);

#endif /* CORE_CONN_STATE_H */

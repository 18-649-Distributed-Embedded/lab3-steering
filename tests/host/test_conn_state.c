/*
 * Host unit tests for src/core/conn_state.c (the exact file that runs on N2).
 * Expected values are written out independently from the signed-off table,
 * not derived from the implementation.
 *
 * Build/run: tests/host/run_tests.sh
 */
#include <stdio.h>
#include <stdlib.h>
#include "core/conn_state.h"
#include "core/can_catalog.h"

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ── 1. All 16 patterns -> row (signed-off table, written by hand) ─────── */
static void test_classify_all_16(void)
{
    /* index = bits N1A N1B N3A N3B as a 4-bit binary number */
    static const int expected_row[16] = {
        /* 0000 */ 10,  /* N2 isolated            */
        /* 0001 */  6,  /* N1 lost (N3 only on B) */
        /* 0010 */  6,  /* N1 lost (N3 only on A) */
        /* 0011 */  6,  /* N1 lost                */
        /* 0100 */  7,  /* N3 lost (N1 only on B) */
        /* 0101 */ 11,  /* N2's own A out         */
        /* 0110 */  8,  /* relay mode 1           */
        /* 0111 */  2,  /* N1's A connection out  */
        /* 1000 */  7,  /* N3 lost (N1 only on A) */
        /* 1001 */  9,  /* relay mode 2           */
        /* 1010 */ 12,  /* N2's own B out         */
        /* 1011 */  3,  /* N1's B connection out  */
        /* 1100 */  7,  /* N3 lost                */
        /* 1101 */  4,  /* N3's A connection out  */
        /* 1110 */  5,  /* N3's B connection out  */
        /* 1111 */  1,  /* normal                 */
    };
    for (int b = 0; b < 16; b++) {
        int r = (int)conn_classify((uint8_t)b);
        CHECK(r == expected_row[b], "bits %d%d%d%d -> row %d, expected %d",
              (b >> 3) & 1, (b >> 2) & 1, (b >> 1) & 1, b & 1, r, expected_row[b]);
    }
    /* upper bits must be ignored */
    CHECK(conn_classify(0xF0 | 0x9) == CONN_ROW_RELAY_2, "upper bits not masked");
    /* every row 1..12 is reachable */
    int seen[13] = {0};
    for (int b = 0; b < 16; b++) seen[conn_classify((uint8_t)b)]++;
    for (int r = 1; r <= 12; r++) CHECK(seen[r] > 0, "row %d unreachable", r);
}

/* ── 2. Row -> LED modes + steering (signed-off table, by hand) ────────── */
static void test_actions(void)
{
    struct { int row; enum deg_led_mode a, b; bool hold; } exp[] = {
        { 1, DEG_LED_OFF,         DEG_LED_OFF,         false },
        { 2, DEG_LED_FLASH_1HZ,   DEG_LED_OFF,         false },
        { 3, DEG_LED_OFF,         DEG_LED_FLASH_1HZ,   false },
        { 4, DEG_LED_FLASH_1HZ,   DEG_LED_OFF,         false },
        { 5, DEG_LED_OFF,         DEG_LED_FLASH_1HZ,   false },
        { 6, DEG_LED_STEADY,      DEG_LED_STEADY,      true  },
        { 7, DEG_LED_STEADY,      DEG_LED_STEADY,      false },
        { 8, DEG_LED_FLASH_0P5HZ, DEG_LED_FLASH_0P5HZ, false },
        { 9, DEG_LED_FLASH_0P5HZ, DEG_LED_FLASH_0P5HZ, false },
        {10, DEG_LED_STEADY,      DEG_LED_STEADY,      true  },
        {11, DEG_LED_STEADY,      DEG_LED_OFF,         false },
        {12, DEG_LED_OFF,         DEG_LED_STEADY,      false },
    };
    for (unsigned i = 0; i < sizeof exp / sizeof exp[0]; i++) {
        struct conn_action a = conn_action_for((enum conn_row)exp[i].row);
        CHECK(a.led_a == exp[i].a && a.led_b == exp[i].b && a.center_hold == exp[i].hold,
              "row %d: got A=%c B=%c hold=%d, expected A=%c B=%c hold=%d", exp[i].row,
              deg_led_char(a.led_a), deg_led_char(a.led_b), a.center_hold,
              deg_led_char(exp[i].a), deg_led_char(exp[i].b), exp[i].hold);
    }
    struct conn_action none = conn_action_for(CONN_ROW_NONE);
    CHECK(none.led_a == DEG_LED_OFF && none.led_b == DEG_LED_OFF && !none.center_hold,
          "ROW_NONE must be inert");
}

/* ── 3. Own-bus-out helpers vs the hazard/zone-fail rows ───────────────── */
static void test_own_bus_and_zone_fail(void)
{
    for (int b = 0; b < 16; b++) {
        bool a_out = !(b & CONN_N1A) && !(b & CONN_N3A);
        bool b_out = !(b & CONN_N1B) && !(b & CONN_N3B);
        CHECK(conn_own_bus_a_out((uint8_t)b) == a_out, "own A out wrong for %x", b);
        CHECK(conn_own_bus_b_out((uint8_t)b) == b_out, "own B out wrong for %x", b);

        /* hazards (system error) exactly in rows 6, 7, 10 via ZONE_FAIL flags */
        bool zf1 = !(b & (CONN_N1A | CONN_N1B));
        bool zf3 = !(b & (CONN_N3A | CONN_N3B));
        int r = conn_classify((uint8_t)b);
        bool hazard_row = (r == 6 || r == 7 || r == 10);
        CHECK((zf1 || zf3) == hazard_row, "bits %x: zone-fail=%d but row %d", b, zf1 || zf3, r);
    }
}

/* ── 4. Age -> bits thresholds ──────────────────────────────────────────── */
static void test_thresholds(void)
{
    CHECK(conn_bits_from_ages(66, 66, 66, 66, CONN_LOST_MS) == 0xF, "66 ms must still be heard");
    CHECK(conn_bits_from_ages(67, 66, 66, 66, CONN_LOST_MS) == 0x7, "67 ms on N1A must be lost");
    CHECK(conn_bits_from_ages(0, 0, 0, 0, CONN_LOST_MS) == 0xF, "age 0 heard");
    CHECK(conn_bits_from_ages(CONN_AGE_NEVER, CONN_AGE_NEVER, CONN_AGE_NEVER, CONN_AGE_NEVER,
                              CONN_LOST_MS) == 0x0, "never heard must be lost");
    CHECK(conn_bits_from_ages(30, 31, 30, 31, CONN_RELAY_TRIGGER_MS) == (CONN_N1A | CONN_N3A),
          "relay threshold 30 ms");
    CHECK(conn_bits_from_ages(0, CONN_AGE_NEVER, CONN_AGE_NEVER, 0, CONN_LOST_MS) == 0x9,
          "bit order N1A N1B N3A N3B");
    CHECK(CONN_LOST_MS == 66 && CONN_RELAY_TRIGGER_MS == 30 && CONN_RELAY_CHECK_MS == 5 &&
          CONN_RELAY_STABLE_N == 2, "constants differ from the signed-off spec");
}

/* ── 5. LED timing: exact on/off times over 4 s at 1 ms resolution ─────── */
static void test_led_timing(void)
{
    for (uint32_t t = 0; t < 4000; t++) {
        CHECK(deg_led_output(DEG_LED_FLASH_1HZ, t)   == ((t % 1000) < 500),  "1 Hz at %u", t);
        CHECK(deg_led_output(DEG_LED_FLASH_0P5HZ, t) == ((t % 2000) < 1000), "0.5 Hz at %u", t);
        CHECK(deg_led_output(DEG_LED_STEADY, t) == true,  "steady at %u", t);
        CHECK(deg_led_output(DEG_LED_OFF, t)    == false, "off at %u", t);
    }
    /* count edges and duty in a 10 s window */
    int on1 = 0, on05 = 0, e1 = 0, e05 = 0;
    bool p1 = deg_led_output(DEG_LED_FLASH_1HZ, 0), p05 = deg_led_output(DEG_LED_FLASH_0P5HZ, 0);
    for (uint32_t t = 0; t < 10000; t++) {
        bool v1 = deg_led_output(DEG_LED_FLASH_1HZ, t), v05 = deg_led_output(DEG_LED_FLASH_0P5HZ, t);
        on1 += v1; on05 += v05; e1 += (v1 != p1); e05 += (v05 != p05); p1 = v1; p05 = v05;
    }
    CHECK(on1 == 5000 && on05 == 5000, "duty not 50%% (1Hz %d, 0.5Hz %d of 10000)", on1, on05);
    CHECK(e1 == 19 && e05 == 9, "edge count wrong (1Hz %d, 0.5Hz %d)", e1, e05);
    /* wrap of the 32-bit uptime does not stall the pattern */
    CHECK(deg_led_output(DEG_LED_FLASH_1HZ, UINT32_MAX) != deg_led_output(DEG_LED_FLASH_1HZ, UINT32_MAX - 500),
          "pattern frozen near uptime wrap");
}

/* ── 6. Relay destination: exhaustive over rows x IDs x zones x buses ─── */
static int expected_dest(int row, uint32_t id, uint8_t zone, int rx)
{
    int origin; /* 1 = N1, 3 = N3, 0 = never relayed */
    if (id == CAN_ID_CTRL || id == CAN_ID_BLINK || id == CAN_ID_HB_N1) origin = 1;
    else if (id == CAN_ID_HB_N3) origin = 3;
    else if (id == CAN_ID_FAULT) origin = (zone == CAN_ZONE_N1) ? 1 : (zone == CAN_ZONE_N3) ? 3 : 0;
    else origin = 0;
    if (origin == 0) return -1;
    if (row == 9) {          /* 1001: N1 on A only, N3 on B only */
        if (origin == 1) return rx == CONN_BUS_A ? CONN_BUS_B : -1;
        else             return rx == CONN_BUS_B ? CONN_BUS_A : -1;
    }
    if (row == 8) {          /* 0110: N1 on B only, N3 on A only */
        if (origin == 1) return rx == CONN_BUS_B ? CONN_BUS_A : -1;
        else             return rx == CONN_BUS_A ? CONN_BUS_B : -1;
    }
    return -1;
}

static void test_relay_dest(void)
{
    const uint32_t ids[] = { CAN_ID_CTRL, CAN_ID_FAULT, CAN_ID_HB_N1, CAN_ID_HB_N2, CAN_ID_HB_N3,
                             CAN_ID_BLINK, CAN_ID_FORCE, 0x000, 0x011, 0x7FF };
    const uint8_t zones[] = { 0, CAN_ZONE_N1, CAN_ZONE_N2, CAN_ZONE_N3, 0xFF };
    int relayed = 0;
    for (int row = 0; row <= 12; row++)
        for (unsigned i = 0; i < sizeof ids / sizeof ids[0]; i++)
            for (unsigned z = 0; z < sizeof zones; z++)
                for (int rx = 0; rx <= 1; rx++) {
                    int got = conn_relay_dest_bus((enum conn_row)row, ids[i], zones[z], rx);
                    int exp = expected_dest(row, ids[i], zones[z], rx);
                    CHECK(got == exp, "row %d id %03x zone %u rx %d: got %d expected %d",
                          row, ids[i], zones[z], rx, got, exp);
                    if (got >= 0) {
                        relayed++;
                        CHECK(got != rx, "relayed back onto the bus it came from");
                        /* loop guard: the forwarded copy is never forwarded again */
                        CHECK(conn_relay_dest_bus((enum conn_row)row, ids[i], zones[z], got) == -1,
                              "row %d id %03x: forwarded copy would bounce", row, ids[i]);
                    }
                }
    CHECK(relayed > 0, "nothing was ever relayed");
    /* N2's own frames are never relayed in any mode */
    for (int row = 8; row <= 9; row++)
        for (int rx = 0; rx <= 1; rx++) {
            CHECK(conn_relay_dest_bus((enum conn_row)row, CAN_ID_HB_N2, 0, rx) == -1, "HB_N2 relayed");
            CHECK(conn_relay_dest_bus((enum conn_row)row, CAN_ID_FORCE, 0, rx) == -1, "FORCE relayed");
            CHECK(conn_relay_dest_bus((enum conn_row)row, CAN_ID_FAULT, CAN_ZONE_N2, rx) == -1, "N2 FAULT relayed");
        }
}

/* ── 7. Relay FSM hysteresis ────────────────────────────────────────────── */
static void test_relay_fsm(void)
{
    struct relay_fsm f;
    const enum conn_row N = CONN_ROW_NORMAL, R8 = CONN_ROW_RELAY_1, R9 = CONN_ROW_RELAY_2;

    relay_fsm_init(&f);
    CHECK(!relay_fsm_update(&f, R9) && f.active == CONN_ROW_NONE, "activated after 1 check");
    CHECK(relay_fsm_update(&f, R9) && f.active == R9, "not activated after 2 checks");
    CHECK(!relay_fsm_update(&f, R9) && f.active == R9, "re-reported activation");

    /* one glitch does not deactivate, and does not re-fire the activation edge */
    CHECK(!relay_fsm_update(&f, N) && f.active == R9, "deactivated after 1 normal check");
    CHECK(!relay_fsm_update(&f, R9) && f.active == R9, "glitch re-fired activation");

    /* two normal checks deactivate (no activation edge reported) */
    relay_fsm_update(&f, N);
    CHECK(!relay_fsm_update(&f, N) && f.active == CONN_ROW_NONE, "not deactivated after 2 checks");

    /* every non-relay row counts as NONE */
    relay_fsm_init(&f);
    for (int r = 1; r <= 12; r++) {
        if (r == 8 || r == 9) continue;
        relay_fsm_update(&f, (enum conn_row)r);
        CHECK(f.active == CONN_ROW_NONE, "row %d activated relay", r);
    }

    /* alternating pattern never activates */
    relay_fsm_init(&f);
    for (int i = 0; i < 100; i++) relay_fsm_update(&f, (i & 1) ? R9 : N);
    CHECK(f.active == CONN_ROW_NONE, "flapping input activated relay");
    relay_fsm_init(&f);
    for (int i = 0; i < 100; i++) relay_fsm_update(&f, (i & 1) ? R9 : R8);
    CHECK(f.active == CONN_ROW_NONE, "8/9 flapping activated relay");

    /* direct switch between relay modes re-fires activation (re-prime) */
    relay_fsm_init(&f);
    relay_fsm_update(&f, R9); relay_fsm_update(&f, R9);
    CHECK(!relay_fsm_update(&f, R8) && f.active == R9, "switched mode after 1 check");
    CHECK(relay_fsm_update(&f, R8) && f.active == R8, "mode switch not reported as activation");

    /* streak counter saturates instead of wrapping (long uptime) */
    relay_fsm_init(&f);
    for (int i = 0; i < 1000; i++) relay_fsm_update(&f, N);
    CHECK(f.streak == 255, "streak did not saturate (%u)", f.streak);
    CHECK(!relay_fsm_update(&f, R9), "activated after saturation + 1");
    CHECK(relay_fsm_update(&f, R9), "not activated after saturation + 2");
}

/* ── 8. ACK gate in front of the FSM ───────────────────────────────────── */
static void test_relay_gate(void)
{
    struct relay_fsm f; bool probe;
    const enum conn_row N = CONN_ROW_NORMAL, R8 = CONN_ROW_RELAY_1, R9 = CONN_ROW_RELAY_2;

    /* non-relay rows pass through, never probe */
    relay_fsm_init(&f);
    for (int r = 1; r <= 12; r++) {
        if (r == 8 || r == 9) continue;
        CHECK(relay_gate(&f, (enum conn_row)r, false, &probe) == (enum conn_row)r && !probe, "row %d gated", r);
    }

    /* split network: probe on first sighting, ACKed -> activates on check 2 */
    relay_fsm_init(&f);
    enum conn_row g = relay_gate(&f, R9, false, &probe);
    CHECK(g == R9 && probe, "first sighting must probe and pass the row");
    relay_fsm_update(&f, g);
    g = relay_gate(&f, R9, true, &probe);
    CHECK(g == R9 && !probe, "acked second sighting must pass without a new probe");
    CHECK(relay_fsm_update(&f, g) && f.active == R9, "did not activate after ACKed probe");
    /* once active, no ACK requirement and no probing */
    CHECK(relay_gate(&f, R9, false, &probe) == R9 && !probe, "active relay re-gated");

    /* own-bus loss: probe never ACKed -> never activates, re-probes each time */
    relay_fsm_init(&f);
    int probes = 0;
    for (int i = 0; i < 20; i++) {
        g = relay_gate(&f, R9, false, &probe); probes += probe;
        CHECK(!relay_fsm_update(&f, g), "activated without ACK (iteration %d)", i);
    }
    CHECK(f.active == CONN_ROW_NONE && probes == 10, "unacked: active=%d probes=%d", f.active, probes);

    /* mode switch 9 -> 8 needs a fresh ACKed probe */
    relay_fsm_init(&f);
    g = relay_gate(&f, R9, false, &probe); relay_fsm_update(&f, g);
    g = relay_gate(&f, R9, true, &probe);  relay_fsm_update(&f, g);
    g = relay_gate(&f, R8, true, &probe);
    CHECK(probe && g == R8, "mode switch did not probe");
    relay_fsm_update(&f, g);
    g = relay_gate(&f, R8, false, &probe);
    CHECK(g == CONN_ROW_NONE, "mode switch passed without ACK");
    (void)N;
}

int main(void)
{
    printf("conn_state unit tests\n");
    test_classify_all_16();       printf("  [1] 16 patterns -> rows\n");
    test_actions();               printf("  [2] rows -> LED modes / steering\n");
    test_own_bus_and_zone_fail(); printf("  [3] own-bus flags, hazards only in rows 6/7/10\n");
    test_thresholds();            printf("  [4] age thresholds 66 / 30 ms\n");
    test_led_timing();            printf("  [5] LED timing 1 Hz / 0.5 Hz\n");
    test_relay_dest();            printf("  [6] relay destinations (exhaustive) + loop guard\n");
    test_relay_fsm();             printf("  [7] relay hysteresis\n");
    test_relay_gate();            printf("  [8] relay ACK gate\n");
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}

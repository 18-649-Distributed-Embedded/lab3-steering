/*
 * Monte Carlo simulation of the 3-node dual-CAN network for R4.8 (a)-(f),
 * using the REAL src/core/conn_state.c for the classifier, the relay
 * hysteresis and the relay destination rule on both STM32 nodes.
 *
 * Model (all times in microseconds, firmware sees milliseconds):
 *  - Every node sends a heartbeat on each bus it is attached to; period is
 *    random per frame in [18, 22] ms (R4.5: 20 ms +/- 10 %).
 *  - Each receiver gets each copy after a random delay in [0.3, 0.8] ms
 *    (frame time + MCP2515 INT + SPI read + dispatch).
 *  - N2 and N3 run the relay: check every 5 ms (random phase), 30 ms
 *    trigger, 2 stable checks, PRIME on activation, forward with a random
 *    delay in [0.4, 1.2] ms. N3 uses the mirrored mapping (its "N3 slot" is N2).
 *  - Every node declares a peer failed at its 20 ms monitor tick (random
 *    phase) when that peer's freshest copy on BOTH buses is older than
 *    FAIL_MS (66 ms, the clarification's 3 missed heartbeats).
 *  - A forwarded frame can itself be forwarded by another relayer, exactly
 *    as on the real bus (no relay flag), so loops would show up.
 *
 * Checks per trial:
 *  1. no false zone failure (a peer that is still reachable, directly or via
 *     a relay, is never declared failed), except the isolated node in (d)
 *  2. N2's steady-state row at the end of the fault matches the spec table
 *  3. the expected node relays and the other does not
 *  4. after all jumpers are restored: N2 back to row 1, no relay, within 500 ms
 *  5. forwarded-frame count stays bounded (no relay loop)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "core/conn_state.h"
#include "core/can_catalog.h"

#define US(ms) ((int64_t)(ms) * 1000)
enum { N1 = 0, N2 = 1, N3 = 2, NODES = 3 };
enum { BA = 0, BB = 1 };

static uint64_t rng = 88172645463325252ull;
static double urand(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (rng >> 11) * (1.0 / 9007199254740992.0); }
static int64_t uni(int64_t lo, int64_t hi) { return lo + (int64_t)(urand() * (double)(hi - lo)); }

/* ── configuration under test ── */
static int     FAIL_MS     = 66;
static int     PRIME       = 1;
static int     GATE        = 1;
static int64_t RX_DLY_MIN  = 300, RX_DLY_MAX = 800;
static int64_t RLY_DLY_MIN = 400, RLY_DLY_MAX = 1200;

/* ── jumper schedule ── */
struct jump { int64_t t; int node, bus, attached; };

struct scenario {
    const char *name;
    struct jump ev[6]; int nev;
    int expect_n2_row;          /* N2 row at end of fault */
    int expect_relayer;         /* -1 none, N2 or N3      */
    int isolated;               /* node expected to be declared failed by others, or -1 */
};

/* ── pending deliveries ── */
struct delivery { int64_t t; int to, bus, origin; };
#define MAXD 4096
static struct delivery dq[MAXD]; static int nd;

/* ── node state ── */
static int     att[NODES][2];
static int64_t last_rx[NODES][NODES][2];   /* [receiver][origin][bus], -1 = never */
static int64_t next_hb[NODES];
static int64_t relay_phase[NODES], mon_phase[NODES];
static struct relay_fsm fsm[NODES];
static int     declared_fail[NODES][NODES];
static int64_t last_ack[NODES][2], probe_t[NODES];
static long    forwarded;

/* conn_state uses N2's view: slot "N1" and slot "N3". Map per node. */
static int slot_n1(int me) { (void)me; return N1; }
static int slot_n3(int me) { return me == N2 ? N3 : N2; }

static uint32_t ms_age(int64_t now, int64_t last)
{
    if (last < 0) return CONN_AGE_NEVER;
    return (uint32_t)(now / 1000 - last / 1000);   /* k_uptime_get_32 granularity */
}

static uint8_t bits_of(int me, int64_t now, uint32_t thr)
{
    int a = slot_n1(me), b = slot_n3(me);
    return conn_bits_from_ages(ms_age(now, last_rx[me][a][BA]), ms_age(now, last_rx[me][a][BB]),
                               ms_age(now, last_rx[me][b][BA]), ms_age(now, last_rx[me][b][BB]), thr);
}

/* ID the relay rule sees for a heartbeat of `origin`, from node `me`'s view */
static uint32_t view_id(int me, int origin)
{
    return origin == slot_n1(me) ? CAN_ID_HB_N1 : CAN_ID_HB_N3;
}

static void push(int64_t t, int to, int bus, int origin)
{
    if (nd >= MAXD) { fprintf(stderr, "delivery queue overflow (relay loop?)\n"); exit(2); }
    dq[nd++] = (struct delivery){ t, to, bus, origin };
}

/* transmit a copy of a heartbeat of `origin` from `sender` on `bus` at time t */
static void transmit(int64_t t, int sender, int bus, int origin, int64_t dmin, int64_t dmax)
{
    if (!att[sender][bus]) return;               /* jumper pulled: nobody ACKs */
    int acked = 0;
    for (int r = 0; r < NODES; r++)
        if (r != sender && att[r][bus]) { push(t + uni(dmin, dmax), r, bus, origin); acked = 1; }
    if (acked && t + 300 > last_ack[sender][bus]) last_ack[sender][bus] = t + 300;
}

static void receive(const struct delivery *d)
{
    int me = d->to;
    if (d->origin == me) return;                   /* own HB echoed back: ignored */
    if (!att[me][d->bus]) return;                  /* jumper pulled in flight */
    last_rx[me][d->origin][d->bus] = d->t;

    if (me == N1) return;                          /* the Pi never relays */
    enum conn_row row = fsm[me].active;
    int dest = conn_relay_dest_bus(row, view_id(me, d->origin), 0, d->bus);
    if (dest >= 0) {
        forwarded++;
        transmit(d->t + uni(RLY_DLY_MIN, RLY_DLY_MAX) - RX_DLY_MIN, me, dest, d->origin,
                 RX_DLY_MIN, RX_DLY_MIN + 1);
    }
}

static void prime(int me, int64_t now)
{
    /* latest cached HB of each origin PER BUS, if fresh and from the source bus */
    for (int o = 0; o < NODES; o++) {
        if (o == me) continue;
        for (int bus = BA; bus <= BB; bus++) {
            int64_t t = last_rx[me][o][bus];
            if (t < 0 || ms_age(now, t) > CONN_LOST_MS) continue;
            int dest = conn_relay_dest_bus(fsm[me].active, view_id(me, o), 0, bus);
            if (dest >= 0) {
                forwarded++;
                transmit(now + uni(RLY_DLY_MIN, RLY_DLY_MAX) - RX_DLY_MIN, me, dest, o,
                         RX_DLY_MIN, RX_DLY_MIN + 1);
            }
        }
    }
}

static int run_trial(const struct scenario *s, int verbose, double *worst_margin_ms)
{
    const int64_t T_END = US(2200), T_RESTORE = US(1500);
    int errors = 0;

    nd = 0; forwarded = 0;
    for (int i = 0; i < NODES; i++) {
        att[i][BA] = att[i][BB] = 1;
        next_hb[i] = uni(0, US(20));
        relay_phase[i] = US(uni(0, 5));     /* timers run on the 1 ms tick grid */
        mon_phase[i] = US(uni(0, 20));
        relay_fsm_init(&fsm[i]);
        last_ack[i][BA] = last_ack[i][BB] = -1; probe_t[i] = 0;
        for (int j = 0; j < NODES; j++) { last_rx[i][j][BA] = last_rx[i][j][BB] = -1; declared_fail[i][j] = 0; }
    }
    int ev = 0, relayed_by[NODES] = {0};
    int restored = 0, n2_row_end = -1;
    int64_t t_back_normal = -1;

    for (int64_t t = 0; t < T_END; t += 50) {
        /* jumper events */
        while (ev < s->nev && s->ev[ev].t <= t) { att[s->ev[ev].node][s->ev[ev].bus] = s->ev[ev].attached; ev++; }
        if (!restored && t >= T_RESTORE) {
            for (int i = 0; i < NODES; i++) att[i][BA] = att[i][BB] = 1;
            restored = 1;
        }

        /* heartbeats */
        for (int i = 0; i < NODES; i++)
            if (t >= next_hb[i]) {
                transmit(t, i, BA, i, RX_DLY_MIN, RX_DLY_MAX);
                transmit(t, i, BB, i, RX_DLY_MIN, RX_DLY_MAX);
                next_hb[i] = t + uni(US(18), US(22));
            }

        /* deliveries due */
        for (int k = 0; k < nd; ) {
            if (dq[k].t <= t) { struct delivery d = dq[k]; dq[k] = dq[--nd]; receive(&d); }
            else k++;
        }

        /* relay checks (STM32s) on the 1 ms tick grid, every 5 ms */
        for (int me = N2; me <= N3; me++)
            if ((t - relay_phase[me]) >= 0 && ((t - relay_phase[me]) % US(5)) == 0) {
                enum conn_row fast = conn_classify(bits_of(me, t, CONN_RELAY_TRIGGER_MS));
                bool acked_both = last_ack[me][BA] > probe_t[me] && last_ack[me][BB] > probe_t[me];
                bool probe = false;
                enum conn_row gated = GATE ? relay_gate(&fsm[me], fast, acked_both, &probe) : fast;
                if (probe) {                         /* repeat own HB on both buses */
                    probe_t[me] = t;
                    transmit(t, me, BA, me, RX_DLY_MIN, RX_DLY_MAX);
                    transmit(t, me, BB, me, RX_DLY_MIN, RX_DLY_MAX);
                }
                bool act = relay_fsm_update(&fsm[me], gated);
                if (fsm[me].active != CONN_ROW_NONE) relayed_by[me] = 1;
                if (act && PRIME) prime(me, t);
            }

        /* failure monitors, every 20 ms */
        for (int me = 0; me < NODES; me++)
            if ((t - mon_phase[me]) >= 0 && ((t - mon_phase[me]) % US(20)) == 0) {
                for (int p = 0; p < NODES; p++) {
                    if (p == me) continue;
                    uint32_t aa = ms_age(t, last_rx[me][p][BA]), ab = ms_age(t, last_rx[me][p][BB]);
                    uint32_t fresh = aa < ab ? aa : ab;
                    bool fail = (aa > (uint32_t)FAIL_MS) && (ab > (uint32_t)FAIL_MS);
                    if (t > US(200) && fresh != CONN_AGE_NEVER) {
                        double m = (double)FAIL_MS - (double)fresh;
                        bool expected_fail = (p == s->isolated || me == s->isolated) && t < T_RESTORE + US(100) && t > s->ev[0].t;
                        if (!expected_fail && m < *worst_margin_ms) *worst_margin_ms = m;
                    }
                    if (fail && t > US(200)) {
                        bool expected = (p == s->isolated || me == s->isolated) && t <= T_RESTORE + US(150);
                        if (!expected) {
                            if (!declared_fail[me][p] && verbose)
                                printf("    node %d declared node %d failed at t=%.1f ms\n", me + 1, p + 1, t / 1000.0);
                            if (!declared_fail[me][p]) errors++;
                            declared_fail[me][p] = 1;
                        }
                    }
                }
            }

        if (t == T_RESTORE - US(10)) n2_row_end = conn_classify(bits_of(N2, t, CONN_LOST_MS));
        if (restored && t_back_normal < 0 &&
            conn_classify(bits_of(N2, t, CONN_LOST_MS)) == CONN_ROW_NORMAL &&
            fsm[N2].active == CONN_ROW_NONE && fsm[N3].active == CONN_ROW_NONE)
            t_back_normal = t;
    }

    if (n2_row_end != s->expect_n2_row) {
        if (verbose) printf("    N2 row at end of fault = %d, expected %d\n", n2_row_end, s->expect_n2_row);
        errors++;
    }
    for (int me = N2; me <= N3; me++) {
        int should = (me == s->expect_relayer);
        if (relayed_by[me] != should) {
            if (verbose) printf("    node %d relay=%d, expected %d\n", me + 1, relayed_by[me], should);
            errors++;
        }
    }
    if (t_back_normal < 0 || t_back_normal - T_RESTORE > US(500)) {
        if (verbose) printf("    not back to normal within 500 ms of restore (%.1f ms)\n",
                            t_back_normal < 0 ? -1.0 : (t_back_normal - T_RESTORE) / 1000.0);
        errors++;
    }
    if (forwarded > 400) { if (verbose) printf("    %ld frames forwarded (loop?)\n", forwarded); errors++; }
    return errors;
}

#define J(t, n, b, a) { US(t), n, b, a }
static const struct scenario S[] = {
    /* (a)/(b): single node off one bus. N2's row reflects what N2 sees. */
    { "(a) N1 off A",            { J(500,N1,BA,0) }, 1,  2, -1, -1 },
    { "(b) N1 off B",            { J(500,N1,BB,0) }, 1,  3, -1, -1 },
    { "(a) N3 off A",            { J(500,N3,BA,0) }, 1,  4, -1, -1 },
    { "(b) N3 off B",            { J(500,N3,BB,0) }, 1,  5, -1, -1 },
    { "(a) N2 off A",            { J(500,N2,BA,0) }, 1, 11, -1, -1 },
    { "(b) N2 off B",            { J(500,N2,BB,0) }, 1, 12, -1, -1 },
    /* (e): whole bus lost */
    { "(e) all off A",           { J(500,N1,BA,0), J(500,N2,BA,0), J(500,N3,BA,0) }, 3, 11, -1, -1 },
    { "(e) all off B",           { J(500,N1,BB,0), J(500,N2,BB,0), J(500,N3,BB,0) }, 3, 12, -1, -1 },
    /* (c): the four checkoff relay cases, sequential (both orders) and simultaneous */
    { "(c1) N1-B then N3-A",     { J(400,N1,BB,0), J(800,N3,BA,0) }, 2,  9, N2, -1 },
    { "(c1) N3-A then N1-B",     { J(400,N3,BA,0), J(800,N1,BB,0) }, 2,  9, N2, -1 },
    { "(c1) simultaneous",       { J(800,N1,BB,0), J(800,N3,BA,0) }, 2,  9, N2, -1 },
    { "(c2) N1-A then N3-B",     { J(400,N1,BA,0), J(800,N3,BB,0) }, 2,  8, N2, -1 },
    { "(c2) N3-B then N1-A",     { J(400,N3,BB,0), J(800,N1,BA,0) }, 2,  8, N2, -1 },
    { "(c2) simultaneous",       { J(800,N1,BA,0), J(800,N3,BB,0) }, 2,  8, N2, -1 },
    { "(c3) N1-A then N2-B",     { J(400,N1,BA,0), J(800,N2,BB,0) }, 2, 12, N3, -1 },
    { "(c3) simultaneous",       { J(800,N1,BA,0), J(800,N2,BB,0) }, 2, 12, N3, -1 },
    { "(c4) N1-B then N2-A",     { J(400,N1,BB,0), J(800,N2,BA,0) }, 2, 11, N3, -1 },
    { "(c4) simultaneous",       { J(800,N1,BB,0), J(800,N2,BA,0) }, 2, 11, N3, -1 },
    /* (d): one node off both buses -> failure is the EXPECTED outcome */
    { "(d) N1 isolated",         { J(800,N1,BA,0), J(800,N1,BB,0) }, 2,  6, -1, N1 },
    { "(d) N3 isolated",         { J(800,N3,BA,0), J(800,N3,BB,0) }, 2,  7, -1, N3 },
    { "(d) N2 isolated",         { J(800,N2,BA,0), J(800,N2,BB,0) }, 2, 10, -1, N2 },
    /* not tested at checkoff (N2 and N3 split, only the Pi on both): no
     * STM32 can relay, so N2 and N3 lose each other. Listed for completeness. */
};

int main(int argc, char **argv)
{
    int trials = (argc > 1) ? atoi(argv[1]) : 2000;
    struct { const char *label; int fail_ms; int prime; int gate; } cfg[] = {
        { "AS IMPLEMENTED: fail 66 ms, prime ON, ACK gate ON", 66, 1, 1 },
        { "fail 70 ms (N3 template value), prime ON, gate ON", 70, 1, 1 },
        { "ablation: ACK gate OFF", 66, 1, 0 },
        { "ablation: priming OFF", 66, 0, 1 },
    };
    int total_bad_impl = 0;

    for (unsigned c = 0; c < sizeof cfg / sizeof cfg[0]; c++) {
        FAIL_MS = cfg[c].fail_ms; PRIME = cfg[c].prime; GATE = cfg[c].gate;
        printf("\n=== %s, %d trials per scenario ===\n", cfg[c].label, trials);
        printf("  %-24s %8s   %s\n", "scenario", "failed", "worst margin to a false zone-fail");
        for (unsigned i = 0; i < sizeof S / sizeof S[0]; i++) {
            int bad = 0; double worst = 1e9;
            for (int k = 0; k < trials; k++) {
                uint64_t seed = rng;
                int e = run_trial(&S[i], 0, &worst);
                if (e) {
                    if (bad == 0 && c < 2) {    /* replay the same trial verbosely */
                        uint64_t after = rng; double dummy = 0;
                        printf("  first failing trial of %s:\n", S[i].name);
                        rng = seed; run_trial(&S[i], 1, &dummy); rng = after;
                    }
                    bad++;
                }
            }
            printf("  %-24s %5d/%-5d %s%.1f ms\n", S[i].name, bad, trials, worst < 0 ? "" : "+", worst);
            if (c == 0) total_bad_impl += bad;
        }
    }
    printf("\nIMPLEMENTED CONFIG: %s\n", total_bad_impl ? "FAILURES" : "all scenarios pass");
    return total_bad_impl ? 1 : 0;
}

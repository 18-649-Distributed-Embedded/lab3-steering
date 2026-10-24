#!/usr/bin/env python3
"""
Lab 3 real-time analysis calculator (N2 steering zone report).

Edit the MEASURED block below after re-measuring on the bench, then run:
    python docs/rta.py
Every table and chain total in the report is printed from these inputs.

Formulas (Student clarification, Oct 22 2026):
  task:    R = C + B + sum_{j in hp} ceil(R / T_j) * C_j        (iterate)
  message: w = B + sum_{j in hp} ceil((w + J_j + tau) / T_j) * C_j
           R = J + w + C
All times in microseconds.
"""
from math import ceil

# ── MEASURED (scope, max over >= 1 min of traffic) ─────────────────────────
M1_PI      = 271.875   # #1 Pi stage, UDP_RX -> CMD_TX                (N1)
M2_RX_N3   = 188.28    # #2 CAN_RX C on N3, INT fall -> CAN_RX_A       (N3)
M3_DRIVE   = 353.12    # #3 DRIVE C on N3, CAN_RX_A -> DIR_A           (N3)
M4_SPI     = 51.56     # #4 SPI frame load, CS low window (= B)
M5_RX_N2   = 188.28    # #5 CAN_RX C on N2      PROVISIONAL (= #2 until measured)
M6_RELAY   = 80.0      # #6 RELAY C on N2       PROVISIONAL (estimate until measured)

# ── FIXED by the handout ───────────────────────────────────────────────────
FRAME   = 270.0        # DLC-8 frame, worst-case stuffing, 500 kbit/s
TAU     = 2.0          # bit time
P20     = 20000.0      # common period
SERVO   = 20000.0      # one servo PWM period (50 Hz)
HB_LOSS = 66000.0      # 3 missed heartbeats at 20 ms + 10 %
HB_MON_T = 20000.0
RX_PER_BUS = 7         # handout: 7 messages per bus per 20 ms
BASE_UTIL = 8.6        # % per bus, Appendix A

def rta(tasks):
    """tasks: list of dict(name,C,T,D,B,n) in priority order (highest first).
    n = jobs per period T (CAN_RX: frames per 20 ms). Returns rows with R and work."""
    out = []
    for i, t in enumerate(tasks):
        hp = tasks[:i]
        R = t["C"] + t["B"]
        while True:
            Rn = t["C"] + t["B"] + sum(ceil(R / j["T"]) * j["n"] * j["C"] for j in hp)
            if abs(Rn - R) < 1e-9:
                break
            R = Rn
            if R > 10 * t["D"] + 1e6:
                break
        terms = " + ".join(
            f"{ceil(R / j['T']) * j['n']}×{j['C']:g}" for j in hp)
        work = f"{t['C']:g} + {t['B']:g}" + (f" + {terms}" if terms else "")
        out.append(dict(t, R=R, ok=R <= t["D"], work=work))
    return out

def table(rows, title):
    print(f"\n### {title}")
    print("| Task | Prio | C (µs) | T | D (µs) | B (µs) | Work | R (µs) | R ≤ D? |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['name']} | {r['prio']} | {r['C']:g} | {r['Tlabel']} | {r['D']:g} | "
              f"{r['B']:g} | {r['work']} | {r['R']:.2f} | {'yes' if r['ok'] else 'NO'} |")
    return {r["name"]: r["R"] for r in rows}

def task(name, prio, C, T, D, B, n=1, Tlabel=None):
    return dict(name=name, prio=prio, C=C, T=T, D=D, B=B, n=n,
                Tlabel=Tlabel or f"{T/1000:g} ms")

# ── Part 3: N2, single bus (placeholder C, CAN_RX = 80 per handout) ────────
n2 = table(rta([
    task("CAN_RX", 1, 80,  P20, 300,   M4_SPI, RX_PER_BUS, "20 ms ×7"),
    task("BLINK_F",3, 20,  P20, 500,   M4_SPI),
    task("STEER",  4, 50,  P20, 1000,  M4_SPI),
    task("TEST_BTN",5,20,  50000, 10000, M4_SPI),
    task("HB_TX",  6, 120, P20, 20000, M4_SPI),
    task("HB_MON", 7, 30,  P20, 20000, M4_SPI),
    task("FORCE_TX",8,120, 50000, 50000, 0),
]), "N2 single bus")

# ── Part 3: N3, single bus (priorities assumed deadline-monotonic) ─────────
n3 = table(rta([
    task("CAN_RX", 1, M2_RX_N3, P20, 300, M4_SPI, RX_PER_BUS, "20 ms ×7"),
    task("DRIVE",  2, M3_DRIVE, P20, 500, M4_SPI),
    task("BLINK_R",3, 20,  P20, 500,   M4_SPI),
    task("PID",    4, 200, 10000, 10000, M4_SPI),
    task("TEST_BTN",5,20,  50000, 10000, M4_SPI),
    task("HB_TX",  6, 120, P20, 20000, M4_SPI),
    task("HB_MON", 7, 30,  P20, 20000, 0),
]), "N3 single bus")

# ── Messages ───────────────────────────────────────────────────────────────
def msg_R(J, hp, B=FRAME):
    """hp: list of (J_j, T_j). Returns (w, R)."""
    w = B
    while True:
        wn = B + sum(ceil((w + Jj + TAU) / Tj) * FRAME for Jj, Tj in hp)
        if wn == w:
            break
        w = wn
    return w, J + w + FRAME

ctrl_w, ctrl_R = msg_R(M1_PI, [])
# BLINK 0x200: hp = CTRL, FAULT, HB_N1, HB_N2, HB_N3 (all 20 ms)
hp_blink = [(M1_PI, P20), (0, P20), (0, P20), (n2["HB_TX"], P20), (n3["HB_TX"], P20)]
blink_w, blink_R = msg_R(M1_PI, hp_blink)
print(f"\nCTRL: J={M1_PI} B=270 w={ctrl_w:g} R={ctrl_R:.3f}")
print(f"BLINK: J={M1_PI} w={blink_w:g} R={blink_R:.3f}")

# ── Part 3 chains ──────────────────────────────────────────────────────────
chains1 = [
    ("Brake / throttle", ctrl_R + n3["CAN_RX"] + n3["DRIVE"], 2000,
     f"CTRL R {ctrl_R:.3f} + CAN_RX(N3) {n3['CAN_RX']:.2f} + DRIVE {n3['DRIVE']:.2f}"),
    ("Steering", ctrl_R + n2["CAN_RX"] + n2["STEER"] + SERVO, 50000,
     f"CTRL R {ctrl_R:.3f} + CAN_RX(N2) {n2['CAN_RX']:.2f} + STEER {n2['STEER']:.2f} + servo 20000"),
    ("Blinker start", blink_R + n3["CAN_RX"] + n3["BLINK_R"], 100000,
     f"BLINK R {blink_R:.3f} + CAN_RX(N3) {n3['CAN_RX']:.2f} + BLINK_R {n3['BLINK_R']:.2f} (N3 is the slower side)"),
    ("Blinker sync", (n3["CAN_RX"] + n3["BLINK_R"]) - (80 + 20), 1000,
     f"N3 max ({n3['CAN_RX']:.2f} + {n3['BLINK_R']:.2f}) − N2 min (80 + 20)"),
    ("Error propagation", n2["TEST_BTN"] + HB_LOSS + HB_MON_T + n3["HB_MON"], 100000,
     f"TEST_BTN(N2) {n2['TEST_BTN']:.2f} + 66000 + 20000 + HB_MON(N3) {n3['HB_MON']:.2f}"),
]
def chain_table(chains, title):
    print(f"\n### {title}")
    print("| Chain | Stages | Total (µs) | Deadline (µs) | Met? |")
    print("|---|---|---|---|---|")
    for n, tot, d, s in chains:
        print(f"| {n} | {s} | {tot:.3f} | {d:g} | {'yes' if tot <= d else 'NO'} |")
chain_table(chains1, "Chains, single bus")

# ── Part 4: N2 with RELAY, scenario (c): N1 off B, N3 off A ────────────────
# Frames N2 receives per 20 ms: A = CTRL, FAULT(N1), HB_N1, BLINK; B = HB_N3, FAULT(N3)
RX_RELAY = 6
# Frames N2 forwards per 20 ms: to B = CTRL, BLINK, HB_N1, FAULT(N1); to A = HB_N3, FAULT(N3)
FWD_TO_B, FWD_TO_A = 4, 2
FWD = FWD_TO_A + FWD_TO_B
n2r = table(rta([
    task("CAN_RX", 1, M5_RX_N2, P20, 300, M4_SPI, RX_RELAY, "20 ms ×6 (4 on A, 2 on B)"),
    task("RELAY",  2, M6_RELAY, P20, 2000, M4_SPI, FWD, "per forwarded frame, 6 / 20 ms"),
    task("BLINK_F",3, 20,  P20, 500,   M4_SPI),
    task("STEER",  4, 50,  P20, 1000,  M4_SPI),
    task("TEST_BTN",5,20,  50000, 10000, M4_SPI),
    task("HB_TX",  6, 120, P20, 20000, M4_SPI),
    task("HB_MON", 7, 30,  P20, 20000, M4_SPI),
    task("FORCE_TX",8,120, 50000, 50000, 0),
]), "N2 with RELAY, scenario (c)")
canrx_eq = M5_RX_N2 + M4_SPI + M5_RX_N2
print(f"CAN_RX if the other bus's equal-priority driver thread is mid-frame: {canrx_eq:.2f}")

util_A = BASE_UTIL + FWD_TO_A * FRAME / P20 * 100
util_B = BASE_UTIL + FWD_TO_B * FRAME / P20 * 100
print(f"\nUtilization A = 8.6 + {FWD_TO_A}×1.35 = {util_A:.2f} %   B = 8.6 + {FWD_TO_B}×1.35 = {util_B:.2f} %")

J_rel = M1_PI + 2 * FRAME + n2r["CAN_RX"] + n2r["RELAY"]
ctrlB_w, ctrlB_R = msg_R(J_rel, [])
print(f"Relayed CTRL on B: J = {M1_PI} + 540 + {n2r['CAN_RX']:.2f} + {n2r['RELAY']:.2f} = {J_rel:.3f}; "
      f"w = {ctrlB_w:g}; R = {ctrlB_R:.3f}")

# Relayed BLINK: first hop on A as before, then N2, then B with the same 5 hp IDs
J_blinkB = blink_R + n2r["CAN_RX"] + n2r["RELAY"]
hp_blinkB = [(J_rel, P20), (0, P20), (J_rel, P20), (n2r["HB_TX"], P20), (n3["HB_TX"], P20)]
blinkB_w, blinkB_R = msg_R(J_blinkB, hp_blinkB)
n3_path = n2r["CAN_RX"] + n2r["RELAY"] + blinkB_w + FRAME + n3["CAN_RX"] + n3["BLINK_R"]

chains4 = [
    ("Brake / throttle", ctrlB_R + n3["CAN_RX"] + n3["DRIVE"], 2000,
     f"relayed CTRL R {ctrlB_R:.3f} + CAN_RX(N3) {n3['CAN_RX']:.2f} + DRIVE {n3['DRIVE']:.2f}"),
    ("Steering", ctrl_R + n2r["CAN_RX"] + n2r["STEER"] + SERVO, 50000,
     f"CTRL on A {ctrl_R:.3f} + CAN_RX(N2) {n2r['CAN_RX']:.2f} + STEER {n2r['STEER']:.2f} + servo 20000"),
    ("Blinker start", blinkB_R + n3["CAN_RX"] + n3["BLINK_R"], 100000,
     f"relayed BLINK R {blinkB_R:.3f} + CAN_RX(N3) {n3['CAN_RX']:.2f} + BLINK_R {n3['BLINK_R']:.2f}"),
    ("Blinker sync", n3_path - (M5_RX_N2 + 20), 1000,
     f"N3 path via relay {n3_path:.2f} − N2 min ({M5_RX_N2:g} + 20)"),
    ("Error propagation", n2r["TEST_BTN"] + HB_LOSS + HB_MON_T + n3["HB_MON"], 100000,
     f"TEST_BTN(N2) {n2r['TEST_BTN']:.2f} + 66000 + 20000 + HB_MON(N3) {n3['HB_MON']:.2f}"),
]
chain_table(chains4, "Chains, scenario (c)")

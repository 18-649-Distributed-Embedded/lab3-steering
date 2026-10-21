#ifndef CORE_CAN_CATALOG_H
#define CORE_CAN_CATALOG_H

#include <stdint.h>
#include <zephyr/sys/util.h>

/*
 * Unified CAN Message Catalog — Lab 3 template task set
 * Shared between N1 (Pi), N2 (Steering), N3 (Drivetrain).
 *
 * All frames: standard 11-bit ID, DLC=8, sent on BOTH buses.
 * Byte 0 of every frame: [flags:4][seq:4]
 *   - seq (bits 3:0): rolling 4-bit counter, wraps 0-15
 *   - flags (bits 7:4): message-specific state nibble
 * Dedup rule: drop frame if (ID, seq) matches the last accepted frame for that ID.
 */

/* ── Message IDs ─────────────────────────────────────────── */
#define CAN_ID_CTRL    0x010U   /* N1 → N2, N3   | period 20ms  | deadline 2ms   */
#define CAN_ID_FAULT   0x020U   /* any → all     | event/20ms   | deadline 100ms */
#define CAN_ID_HB_N1   0x100U   /* N1 → N2, N3   | period 20ms  | deadline 20ms  */
#define CAN_ID_HB_N2   0x101U   /* N2 → N1, N3   | period 20ms  | deadline 20ms  */
#define CAN_ID_HB_N3   0x102U   /* N3 → N1, N2   | period 20ms  | deadline 20ms  */
#define CAN_ID_BLINK   0x200U   /* N1 → N2, N3   | event/20ms   | deadline 100ms */
#define CAN_ID_FORCE   0x300U   /* N2 → N1       | period 50ms  | deadline 50ms  */

/* ── Zone IDs (byte 1 of FAULT) ──────────────────────────── */
#define CAN_ZONE_N1    0x01U
#define CAN_ZONE_N2    0x02U
#define CAN_ZONE_N3    0x03U

/* ── FAULT reason codes (byte 2 of FAULT) ────────────────── */
#define CAN_FAULT_TEST_BTN   0x01U   /* test button single press      */
#define CAN_FAULT_WATCHDOG   0x02U   /* no CTRL for > 150ms           */
#define CAN_FAULT_ESTOP      0x03U   /* brake byte asserted in CTRL   */
#define CAN_FAULT_OUT_RANGE  0x04U   /* out-of-range input value      */
#define CAN_FAULT_ZONE_FAIL  0x05U   /* peer zone failure detected    */

/* ── Zone state nibble (upper nibble of byte 0 in HB frames) */
#define CAN_ZONE_STATE_OK        0x0U
#define CAN_ZONE_STATE_DEGRADED  0x1U
#define CAN_ZONE_STATE_FAULT     0x2U

/* ── Bus health byte (byte 2 of HB frames) ───────────────── */
#define CAN_BUS_HEALTH_A   BIT(0)   /* CAN A is healthy */
#define CAN_BUS_HEALTH_B   BIT(1)   /* CAN B is healthy */

/* ── Blink state (byte 1 of BLINK) ──────────────────────── */
#define CAN_BLINK_OFF     0x00U
#define CAN_BLINK_LEFT    0x01U
#define CAN_BLINK_RIGHT   0x02U
#define CAN_BLINK_HAZARD  0x03U

/* Helper: pack byte 0 */
static inline uint8_t can_byte0(uint8_t flags_nibble, uint8_t seq)
{
    return (uint8_t)(((flags_nibble & 0x0F) << 4) | (seq & 0x0F));
}

/* Helper: extract seq from byte 0 */
static inline uint8_t can_seq(uint8_t byte0)
{
    return byte0 & 0x0F;
}

/* Helper: extract flags nibble from byte 0 */
static inline uint8_t can_flags(uint8_t byte0)
{
    return (byte0 >> 4) & 0x0F;
}

#endif /* CORE_CAN_CATALOG_H */


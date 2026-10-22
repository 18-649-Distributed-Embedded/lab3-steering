#ifndef CORE_CMD_H
#define CORE_CMD_H

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/sys/atomic.h>

// Safety Interlock Bitmask Flags (bit numbers for atomic_*_bit)
// Any bit set in system_safety_flags = this node is in the error state.
#define SAFETY_FLAG_WATCHDOG_TIMEOUT 0   // no valid CTRL for > 150ms (also set at power-up)
#define SAFETY_FLAG_ESTOP            1   // reserved (unused on N2)
#define SAFETY_FLAG_ZONE_FAIL_SELF   2   // this zone failed (test button)
#define SAFETY_FLAG_PEER_FAULT       3   // a peer reported FAULT (FAULT msg or HB state)
#define SAFETY_FLAG_ZONE_FAIL_N1     4   // N1 heartbeat missing on BOTH buses
#define SAFETY_FLAG_ZONE_FAIL_N3     5   // N3 heartbeat missing on BOTH buses

extern atomic_t system_safety_flags;

// Bus health flags (bit numbers) - drive DEG_A / DEG_B LEDs (R4.7)
#define BUS_FLAG_DEG_A 0
#define BUS_FLAG_DEG_B 1

extern atomic_t bus_health_flags;

// Blinker state flags
#define BLINKER_OFF    0x00
#define BLINKER_LEFT   0x01
#define BLINKER_RIGHT  0x02
#define BLINKER_HAZARD 0x03



#endif // CORE_CMD_H

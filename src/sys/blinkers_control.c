#include "sys/blinkers_control.h"
#include "hw/blinkers.h"

void blinkers_control_init(void)
{
    /* Safe-state: all LEDs off */
    blinkers_set_left(false);
    blinkers_set_right(false);
    /* Thread removed for Lab 3 Part 1 — will be re-added in Part 2 */
}

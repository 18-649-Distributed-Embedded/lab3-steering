#ifndef SYS_HEALTH_H
#define SYS_HEALTH_H

/**
 * @brief Initialize the health monitor:
 *        - TEST_BTN (PC13, sw0 alias): single press = zone fail, double = restore
 *        - Watchdog: sets WATCHDOG_TIMEOUT if no CTRL for > 150ms
 */
void health_init(void);

#endif /* SYS_HEALTH_H */

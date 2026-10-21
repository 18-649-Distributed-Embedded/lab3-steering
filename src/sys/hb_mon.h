#ifndef SYS_HB_MON_H
#define SYS_HB_MON_H

/**
 * @brief Start the HB_MON task: monitors N1 + N2 heartbeats on both buses,
 *        updates ZONE_FAIL flags and drives DEG_A / DEG_B LEDs (R4.7).
 */
void hb_mon_init(void);

#endif /* SYS_HB_MON_H */


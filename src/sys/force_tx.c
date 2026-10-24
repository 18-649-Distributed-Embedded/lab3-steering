#include "sys/force_tx.h"
#include <zephyr/kernel.h>
#include "hw/can.h"
#include "core/can_catalog.h"
#include "core/cmd.h"

#define FORCE_TX_THREAD_STACK  512
#define FORCE_TX_THREAD_PRIO   8      /* periodic, 50 ms deadline */
#define FORCE_TX_PERIOD_MS     50

K_SEM_DEFINE(force_tx_sem, 0, 1);

static void force_tx_timer_isr(struct k_timer *t)
{
    k_sem_give(&force_tx_sem);
}

K_TIMER_DEFINE(force_tx_timer, force_tx_timer_isr, NULL);

static void force_tx_thread_func(void *p1, void *p2, void *p3)
{
    uint8_t seq = 0;
    int16_t dummy = -100;   /* Lab 3: dummy counter sweeping -100..100 */

    while (1) {
        k_sem_take(&force_tx_sem, K_FOREVER);

        uint8_t payload[8] = {
            can_byte0(0, seq),
            (uint8_t)(int8_t)dummy,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        };
        /* Self-failed zone is silent on the bus (R4.5 / S5) */
        if (!atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF)) {
            can_send_both(CAN_ID_FORCE, 8, payload);
        }

        seq = (seq + 1) & 0x0F;
        dummy = (dummy >= 100) ? -100 : dummy + 1;
    }
}

K_THREAD_STACK_DEFINE(force_tx_stack, FORCE_TX_THREAD_STACK);
static struct k_thread force_tx_thread_data;

void force_tx_init(void)
{
    k_timer_start(&force_tx_timer, K_MSEC(FORCE_TX_PERIOD_MS), K_MSEC(FORCE_TX_PERIOD_MS));

    k_thread_create(&force_tx_thread_data, force_tx_stack,
                    K_THREAD_STACK_SIZEOF(force_tx_stack),
                    force_tx_thread_func,
                    NULL, NULL, NULL,
                    FORCE_TX_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&force_tx_thread_data, "FORCE_TX");
}

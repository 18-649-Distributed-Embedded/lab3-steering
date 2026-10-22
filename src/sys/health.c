#include "sys/health.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include "core/cmd.h"
#include "core/can_catalog.h"
#include "core/log.h"
#include "hw/can.h"
#include "sys/can_rx.h"

/* ── Globals ────────────────────────────────────────────── */
atomic_t system_safety_flags = ATOMIC_INIT(BIT(SAFETY_FLAG_WATCHDOG_TIMEOUT));
atomic_t bus_health_flags    = ATOMIC_INIT(0);

/* ── TEST_BTN (PC13 via sw0 alias) ─────────────────────── */
static const struct gpio_dt_spec test_btn =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static struct gpio_callback btn_cb_data;
K_SEM_DEFINE(btn_sem, 0, 5);   /* max 5 to capture rapid presses */

static void btn_isr(const struct device *dev,
                    struct gpio_callback *cb,
                    uint32_t pins)
{
    static uint32_t last_press_ms = 0;
    uint32_t now = k_uptime_get_32();

    if ((now - last_press_ms) < 80U) {
        return;    /* debounce: ignore pulses < 80ms */
    }
    last_press_ms = now;
    k_sem_give(&btn_sem);
}

#define BTN_THREAD_STACK  512
#define BTN_THREAD_PRIO   3    /* on the fault chain */

static uint8_t fault_seq = 0;

static void send_fault(uint8_t reason)
{
    uint8_t payload[8] = {
        can_byte0(0, fault_seq & 0x0F),
        CAN_ZONE_N2,
        reason,
        0x00, 0x00, 0x00, 0x00, 0x00,
    };
    fault_seq++;
    can_send_both(CAN_ID_FAULT, 8, payload);
}

static void btn_thread_func(void *p1, void *p2, void *p3)
{
    while (1) {
        /* Wait for first button press */
        k_sem_take(&btn_sem, K_FOREVER);

        bool already_failed = atomic_test_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF);

        if (!already_failed) {
            /* Single press: immediately trip zone failure (<10ms deadline) */
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF);
            send_fault(CAN_FAULT_TEST_BTN);
            DEBUG_PRINT("TEST_BTN: single press -> zone failed\n");

            /* Wait to see if this was actually the first of a double press */
            int ret = k_sem_take(&btn_sem, K_MSEC(500));
            if (ret == 0) {
                /* Second press within 500ms -> double press! Restore */
                atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF);
                send_fault(0);
                DEBUG_PRINT("TEST_BTN: double press -> zone restored\n");
                while (k_sem_take(&btn_sem, K_NO_WAIT) == 0) {}
            }
        } else {
            /* We are currently failed: wait for second press to complete double-press */
            int ret = k_sem_take(&btn_sem, K_MSEC(500));
            if (ret == 0) {
                /* Second press within 500ms -> double press! Restore */
                atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_ZONE_FAIL_SELF);
                send_fault(0);
                DEBUG_PRINT("TEST_BTN: double press -> zone restored\n");
                while (k_sem_take(&btn_sem, K_NO_WAIT) == 0) {}
            } else {
                DEBUG_PRINT("TEST_BTN: single press while failed (double press needed to restore)\n");
            }
        }
    }
}

/* ── Watchdog ────────────────────────────────────────────── */
#define WATCHDOG_TIMEOUT_MS  150U
#define WATCHDOG_STACK       512
#define WATCHDOG_PRIO        7    /* background safety check */

static void watchdog_thread_func(void *p1, void *p2, void *p3)
{
    while (1) {
        k_msleep(20);

        uint32_t now = k_uptime_get_32();
        uint32_t last = last_ctrl_time;

        if (last == 0 || (now - last) > WATCHDOG_TIMEOUT_MS) {
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_WATCHDOG_TIMEOUT);
        } else {
            atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_WATCHDOG_TIMEOUT);
        }
    }
}

/* ── Stacks / threads ────────────────────────────────────── */
K_THREAD_STACK_DEFINE(btn_stack,       BTN_THREAD_STACK);
K_THREAD_STACK_DEFINE(watchdog_stack,  WATCHDOG_STACK);
static struct k_thread btn_thread_data;
static struct k_thread watchdog_thread_data;

/* ── Init ────────────────────────────────────────────────── */
void health_init(void)
{
    if (!device_is_ready(test_btn.port)) {
        DEBUG_PRINT("Error: test button GPIO not ready\n");
        return;
    }
    gpio_pin_configure_dt(&test_btn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&test_btn, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&btn_cb_data, btn_isr, BIT(test_btn.pin));
    gpio_add_callback(test_btn.port, &btn_cb_data);

    k_thread_create(&btn_thread_data, btn_stack,
                    K_THREAD_STACK_SIZEOF(btn_stack),
                    btn_thread_func,
                    NULL, NULL, NULL,
                    BTN_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&btn_thread_data, "TEST_BTN");

    k_thread_create(&watchdog_thread_data, watchdog_stack,
                    K_THREAD_STACK_SIZEOF(watchdog_stack),
                    watchdog_thread_func,
                    NULL, NULL, NULL,
                    WATCHDOG_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&watchdog_thread_data, "WATCHDOG");

    DEBUG_PRINT("Health monitor initialized (TEST_BTN=PC13, watchdog=150ms)\n");
}

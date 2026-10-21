#include "hw/blinkers.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "core/log.h"

static const struct gpio_dt_spec led_left = GPIO_DT_SPEC_GET(DT_ALIAS(led_left), gpios);
static const struct gpio_dt_spec led_right = GPIO_DT_SPEC_GET(DT_ALIAS(led_right), gpios);

int blinkers_init(void)
{
    if (gpio_pin_configure_dt(&led_left, GPIO_OUTPUT_INACTIVE) < 0 ||
        gpio_pin_configure_dt(&led_right, GPIO_OUTPUT_INACTIVE) < 0) {
        DEBUG_PRINT("Error: Failed to configure turn signal LEDs\n");
        return -1;
    }
    return 0;
}

void blinkers_set_left(bool state) { gpio_pin_set_dt(&led_left, state ? 1 : 0); }
void blinkers_set_right(bool state) { gpio_pin_set_dt(&led_right, state ? 1 : 0); }

void blinkers_toggle_all(void)
{
    gpio_pin_toggle_dt(&led_left);
    gpio_pin_toggle_dt(&led_right);
}

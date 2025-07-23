#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#define BUTTON_NODE DT_NODELABEL(user_button)
#define LED_NODE DT_NODELABEL(led0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static struct gpio_callback button_cb_data;

void button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (pins & BIT(button.pin)) {
        printk("Button interrupt triggered\n");
        // Toggle the LED state
        gpio_pin_toggle_dt(&led);
    }
}

void main(void)
{
    gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&button_cb_data, button_handler, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    printk("GPIO interrupt configured.\n");

    while (1) {
        k_msleep(1000);
    }
}

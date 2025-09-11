#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#define UART_NODE DT_NODELABEL(usart1)
const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

void uart_poll_send(const struct device *dev, const char *str)
{
    while (*str) {
        uart_poll_out(dev, *str++);
    }
}

int main(void)
{
    while (1) {
        const char *send = "Hello, I'm MCU!\n";
        uart_poll_send(uart_dev, send);
        k_msleep(1000); 
    }
}
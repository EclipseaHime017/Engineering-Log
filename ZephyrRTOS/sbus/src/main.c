#include <math.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sbus.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/debug/thread_analyzer.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define SBUS_NODE DT_NODELABEL(sbus0)

int main()
{
    const struct device *sbus = DEVICE_DT_GET(SBUS_NODE);

    if (!device_is_ready(sbus)) {
        LOG_ERR("SBUS device not ready");
        return 0;
    }

    uint8_t channel = 5; 
    float percent = 0; 
    int digital = 0; 

    LOG_INF("Channel %d: percent = %f, digital = %d", channel, percent, digital);

    while (1) {
        percent = sbus_get_percent(sbus, channel);
        digital = sbus_get_digit(sbus, channel);

        LOG_INF("Channel %d: percent = %f, digital = %d", channel, percent, digital);
        k_msleep(500);
    }

    return 0;
}
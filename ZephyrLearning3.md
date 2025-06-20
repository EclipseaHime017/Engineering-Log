# ZephyrRTOS Learning III - GPIO基础和中断

**作者：EclipseaHime017**  
**环境：Ubuntu22.04**  
**设备：DJI Robomaster Development Board A STM32F427IIH6**  

## 一、ZephyrRTOS中的GPIO

GPIO（General Purpose Input/Output）用于控制芯片上的引脚的电平状态，高/低电平对应 1/0。
Zephyr把GPIO抽象成设备对象，用统一的API进行管理，并通过设备树进行硬件配置绑定。

### 1. 设备树配置  
开始GPIO的部分前一个需要澄清的概念是设备树（devicetree)，Zephyr 中的 设备树（Device Tree） 是整个系统硬件抽象和驱动自动化配置的核心机制。
它允许你在不写死驱动代码的情况下，声明并管理硬件资源（如 GPIO、UART、I2C、SPI、LED、按键等），使应用层代码更加通用、可移植。

| 功能      | 说明                                     |
| ------- | -------------------------------------- |
| 📌 抽象硬件 | 将芯片资源（GPIO 引脚、串口编号、I2C 控制器等）从驱动逻辑中分离出来 |
| 📌 自动配置 | 编译时由设备树生成头文件和配置结构，驱动用统一 API 访问         |
| 📌 移植方便 | 相同代码可以适配多个芯片，只需修改设备树配置                 |
| 📌 简化代码 | 通过宏自动生成 `struct device` 绑定，避免硬编码设备名    |

为了能够在代码中控制GPIO的状态，需要先从设备树中声明你要控制的GPIO引脚。例如：
```c
/ {
    leds {
        compatible = "gpio-leds";

        led0: led_0 {
            gpios = <&gpioa 5 GPIO_ACTIVE_HIGH>;
            label = "User LED";
        };
    };

    aliases {
        led0 = &led0;
    };
};
```
说明：
>gpios = <&gpioa 5 GPIO_ACTIVE_HIGH>; 表示使用 GPIOA 控制器，第 5 引脚，高电平有效  
&gpioa 是 GPIO 控制器的句柄  
aliases 中的 led0 = &led0; 给这个 LED 节点设置了一个别名  

这样写完后，用户代码中就可以通过 DT_ALIAS(led0) 快速引用这个节点。在代码里包括
```c
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0) //这句宏的作用是：将 LED0_NODE 定义为设备树中别名为 led0 的节点句柄
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
//它展开后大致等价于：
//const struct gpio_dt_spec led = {
//    .port = DEVICE_DT_GET(DT_PHANDLE(LED0_NODE, gpios)),
//    .pin = DT_GPIO_PIN(LED0_NODE, gpios),
//    .dt_flags = DT_GPIO_FLAGS(LED0_NODE, gpios)
//};
```
从设备树里面导入的GPIO口是一个结构体
```c
struct gpio_dt_spec {
    const struct device *port;  // GPIO 控制器设备
    gpio_pin_t pin;             // 引脚编号（如 13）
    gpio_flags_t dt_flags;      // 从 DTS 中获取的标志，例如 GPIO_ACTIVE_LOW
};
```

### 2. GPIO输出
在Learning II里面有提到，ZephyrRTOS操作GPIO和HAL是类似的，首先需要配置一个GPIO引脚为输出、输入、上拉、下拉、中断等模式。
```c
int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, gpio_flags_t flags);
```
spec：通过设备树获取的 gpio_dt_spec 指针  
flags：
| 宏名                     | 说明            |
| ---------------------- | ------------- |
| `GPIO_OUTPUT`          | 输出模式（默认推挽）     |
| `GPIO_OUTPUT_ACTIVE`   | 输出并拉高（初始化为 1） |
| `GPIO_OUTPUT_INACTIVE` | 输出并拉低（初始化为 0） |
| `GPIO_INPUT`           | 输入模式（用于按钮等）   |
| `GPIO_PULL_UP`         | 上拉            |
| `GPIO_PULL_DOWN`       | 下拉            |
| `GPIO_OPEN_DRAIN`      | 开漏输出          |
| `GPIO_OPEN_SOURCE`     | 开源输出（我没见过，GPT说的）|

完成gpio的配置以后，我们通常使用GPIO无异于输入输出，比如点灯的时候，如果希望GPIO输出，那么通过
```c
int gpio_pin_set_dt(const struct gpio_dt_spec *spec, int value); // spec：设备树封装的引脚信息 value：0或非0
gpio_pin_set_dt(&led, 1);  // 输出高电平
gpio_pin_set_dt(&led, 0);  // 输出低电平
```
此外
```c
gpio_pin_toggle_dt(&led);
```
可以反转当前 GPIO 引脚的输出状态（高变低，低变高）。最后，如果你需要知道GPIO的状态
```c
int gpio_pin_get_dt(const struct gpio_dt_spec *spec); //读取 GPIO 引脚的电平（用于输入模式）
bool gpio_is_ready_dt(const struct gpio_dt_spec *spec); //用于判断设备是否就绪（通常在 main() 中提前检查）
```

### 3. GPIO输入
GPIO 输入模式用于读取引脚上的电平状态，常见应用包括：按键检测、读取外部设备的 READY 状态、接收数字传感器输出、响应外部中断信号。
在设备树定义中，GPIO_ACTIVE_LOW表示逻辑“按下”为低电平（即物理低电平被解释为逻辑1），基本配置和输出一样，
随后初始化引脚为输入
```c
gpio_pin_configure_dt(&button, GPIO_INPUT);
//或者 gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
```
然后可以通过gpio_pin_get_dt读取电平状态（记得消除抖动）。

一个按键控制LED的例子是，
```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#define LED_NODE   DT_ALIAS(led0)
#define BUTTON_NODE DT_ALIAS(sw0)

const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

void main(void)
{
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);  // 初始亮
    gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);

    while (1) {
        int val = gpio_pin_get_dt(&button);
        gpio_pin_set_dt(&led, !val); // 按下时 val=0，点亮LED
        k_msleep(20);
    }
}
```

## 二、 GPIO中断
中断（Interrupt）是CPU 在运行主程序时被外部或内部事件打断，转去处理这些事件的一种机制。
```txt
正常运行 → 事件发生 → 触发中断请求（IRQ）
     ↓
中断控制器（NVIC/EXTI）识别请求
     ↓
CPU 保存当前执行状态（上下文）
     ↓
跳转到 ISR（中断服务程序）执行
     ↓
ISR 执行完成 → 恢复原状态 → 返回主程序
```
Zephyr提供对GPIO引脚的中断支持，允许你配置某个引脚在电平或边沿变化时触发回调函数，这是处理按键、传感器、外部设备信号的常见手段。
在硬件层面，GPIO控制器（如STM32的EXTI、nRF的GPIOTE）将引脚状态变化映射为中断信号，由CPU响应。

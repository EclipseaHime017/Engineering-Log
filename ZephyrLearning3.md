# ZephyrRTOS Learning III - GPIO基础和中断

**作者：EclipseaHime017**  
**环境：Ubuntu22.04**  
**设备：DJI Robomaster Development Board A STM32F427IIH6** or **DJI Robomaster Development Board C STM32F407IGT6**


## 一、ZephyrRTOS中的GPIO

&emsp;&emsp; GPIO（General Purpose Input/Output）用于控制芯片上的引脚的电平状态，高/低电平对应 1/0。
Zephyr把GPIO抽象成设备对象，用统一的API进行管理，并通过设备树进行硬件配置绑定。

### 1. 设备树配置  
&emsp;&emsp; 开始GPIO的部分前一个需要澄清的概念是设备树（devicetree)，Zephyr 中的 设备树（Device Tree） 是整个系统硬件抽象和驱动自动化配置的核心机制。
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
&emsp;&emsp; 在Learning II里面有提到，ZephyrRTOS操作GPIO和HAL是类似的，首先需要配置一个GPIO引脚为输出、输入、上拉、下拉、中断等模式。
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
&emsp;&emsp; GPIO 输入模式用于读取引脚上的电平状态，常见应用包括：按键检测、读取外部设备的 READY 状态、接收数字传感器输出、响应外部中断信号。
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
&emsp;&emsp; 中断（Interrupt）是CPU 在运行主程序时被外部或内部事件打断，转去处理这些事件的一种机制。
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
中断类型基本分为
| 类型       | 示例             | 描述     |
| -------- | -------------- | ------ |
| **外部中断** | GPIO、按键、传感器    | 来自外设引脚 |
| **内部中断** | 定时器溢出、DMA 完成   | 芯片内部事件 |
| **软件中断** | 异常、系统调用（如 SVC） | 由软件触发  |

&emsp;&emsp; Zephyr提供对GPIO引脚的中断支持，允许配置某个引脚在电平或边沿变化时触发回调函数，这是处理按键、传感器、外部设备信号的常见手段。
在硬件层面，GPIO控制器（如STM32的EXTI、nRF的GPIOTE）将引脚状态变化映射为中断信号，由CPU响应。
GPIO的中断实现可以基本分为以下五步。
>配置设备树定义的 GPIO 引脚（包含中断极性）
使用```gpio_pin_configure_dt() ```配置引脚
设置```gpio_callback ```结构体与回调函数
调用```gpio_add_callback() ```注册中断处理函数
使用``` gpio_pin_interrupt_configure_dt() ```配置触发方式

&emsp;&emsp; 首先注册GPIO引脚后，在源文件里面包括
```c
const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON, gpios);
```
然后注册中断回调函数的结构体
```c
static struct gpio_callback button_cb_data
```
这个回调结构体包括了
```c
struct gpio_callback {
    sys_snode_t node; //sys_snode_t	回调链表节点，Zephyr 内部用链表管理所有注册的中断回调。
    gpio_callback_handler_t handler; //gpio_callback_handler_t	回调函数指针，在中断发生时被调用。
    gpio_port_pins_t pin_mask; //gpio_port_pins_t，表示该回调关注哪些 GPIO 引脚（每位代表一个pin，一个32位无符号整形，对应引脚序号为1其余为0）
};
```
此外还需要注册回调函数
```c
void (*gpio_callback_handler_t)(const struct device *port,struct gpio_callback *cb, gpio_port_pins_t pins);
```
其中三个参数分别代表设备地址，回调触发的结构体，和相应GPIO的编号；注册（声明定义）回调函数以后，需要对结构体初始化，然后注册回调，这两个函数的定义分别是:  
```c
/**
* @brief 初始化回调结构体
* @param callback 指向回调结构体的指针
* @param handler 回调函数指针
* @param pin_mask GPIO 引脚
*/
static inline void gpio_init_callback(struct gpio_callback *callback, gpio_callback_handler_t handler, gpio_port_pins_t pin_mask)
{
    callback->handler = handler;
    callback->pin_mask = pin_mask;
}
/**
* @brief 注册回调进驱动链表
* @param port GPIO 控制器设备
* @param callback 指向回调结构体的指针
* @note 第一部是获取GPIO驱动实现的```gpio_driver_api```接口结构体,第二步时返回结构体的成员（见后文）
*/
int gpio_add_callback(const struct device *port, struct gpio_callback *callback)
{
    const struct gpio_driver_api *api = (const struct gpio_driver_api *)port->api;
    return api->manage_callback(port, callback, true);  // true = add
}
```
(Optional)在STM32中，
```c
static int gpio_stm32_manage_callback(const struct device *port,
                                      struct gpio_callback *callback,
                                      bool set)
{
    struct gpio_stm32_data *data = port->data;
    return gpio_manage_callback(&data->cb, callback, set);
}

int gpio_manage_callback(sys_slist_t *callbacks,
                         struct gpio_callback *callback,
                         bool set)
{
    if (set) {
        sys_slist_append(callbacks, &callback->node);
    } else {
        sys_slist_find_and_remove(callbacks, &callback->node);
    }
    return 0;
}
```
从而实现注册。(Optional end)

&emsp;&emsp; 最后如果要启用GPIO中断，那么需要配置中断触发方式
```c
gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
```
中断的触发方式有
| 宏                           | 含义                |
| --------------------------- | ----------------- |
| `GPIO_INT_EDGE_TO_ACTIVE`   | 上升沿触发             |
| `GPIO_INT_EDGE_TO_INACTIVE` | 下降沿               |
| `GPIO_INT_EDGE_BOTH`        | 双边沿               |
| `GPIO_INT_LEVEL_ACTIVE`     | 高电平触发             |
| `GPIO_INT_LEVEL_INACTIVE`   | 低电平触发             |

用一个按键控制GPIO的例子来实际操作一下。
>按下按键，LED点亮；松开按键，LED熄灭。
~~显然这样的任务用中断来实现又有了点脱裤子排矢气的感觉了，不过无所谓了哈哈哈哈~~

&emsp;&emsp; 首先梳理一下思路，为了实现这样的功能，需要注册两个GPIO分别表示LED和按键，请参考开发板引脚原理图。
随后，需要在代码中创建回调结构体和回调函数并进行初始化和注册。最后主函数while循环内进行循环延时即可（为了保证while不占用过多资源）。
```c
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
```
其次稍微对先前的内容进行一个复习，创建CMakeLists.txt和prj.conf
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(EXTI)

target_sources(app PRIVATE src/main.c)
```
```Kconfig
CONFIG_GPIO=y

# 启用 UART 控制台
CONFIG_UART_CONSOLE=y

# 启用控制台支持
CONFIG_CONSOLE=y
```
最后编译烧录即可，详细代码可以参考仓库里适用于robomaster C板的案例（暂未更新）

## Appendix for Learning 3

### 注册回调驱动链表
&emsp;&emsp; 本质上它是 Zephyr 中为了支持多个 PIO引脚共享一个中断源、多个中断用户共享同一控制器而设计的一种事件调度机制，
是Zephyr实现驱动层中断分发的标准方式。  
&emsp;&emsp; Zephyr驱动中，维护一个callback链表每当中断触发时，遍历链表，每个回调检查是否命中当前中断pin命中就调用你注册的回调函数。
>把GPIO控制器比作一个“报警器”，每个gpio_callback是一个“监听器”挂在上面，  
报警器响了，就去遍历所有监听器，看看谁订阅了当前的报警来源。

&emsp;&emsp; Zephyr的GPIO驱动支持同一个GPIO控制器或引脚绑定多个gpio_callback回调结构体，每个GPIO控制器驱动内部维护了一个链表：
```c
struct gpio_driver_data {
    ...
    sys_slist_t callbacks;  // 🔗 链表结构，存放多个gpio_callback
};
```
以 Zephyr 的 GPIO 驱动为例，中断触发后，执行类似如下流程：
```c
void gpio_isr(const struct device *dev)
{
    uint32_t triggered_pins = read_pending_pins(dev); //读取当前触发的的pin掩码

    SYS_SLIST_FOR_EACH_NODE(&data->callbacks, node) {
        struct gpio_callback *cb = CONTAINER_OF(node, struct gpio_callback, node);

        if (triggered_pins & cb->pin_mask) {
            cb->handler(dev, cb, triggered_pins);  //调用匹配的回调函数
        }
    }
}
//注册回调，就是往这个链表里append()一个节点。
```
Zephyr使用的是单向链表sys_slist_t，调用sys_slist_append()来添加新的回调节点。所以第一个注册的回调最先执行后注册的回调会在链表尾部，排在后面执行。

&emsp;&emsp; 对比STM32 HAL库开发，HAL库实现中断回调函数：
```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
```
这种方式灵活性差,用户需手动管理每个引脚的响应逻辑且无法动态注册/注销回调。
| 比较项    | Zephyr RTOS                | STM32 HAL                      |
| ------ | -------------------------- | ------------------------------ |
| 中断分发   | 驱动中维护 callback 链表          | 编译时写死在 `EXTIxx_IRQHandler`     |
| 用户逻辑注册 | `gpio_add_callback()` 动态添加 | 手动写 `HAL_GPIO_EXTI_Callback()` |
| 支持多个回调 | ✅ 支持多个 callback            | ❌ 通常只能写一个函数                    |
| 多用户支持  | ✅ 每个 pin 可单独绑定             | ❌ 需要用户手动区分 pin                 |
| 动态绑定   | ✅ 运行时绑定 ISR                | ❌ 靠编译时固定分配                     |


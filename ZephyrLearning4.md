# ZephyrRTOS Learning IV -  UART通信

**作者：EclipseaHime017**  
**环境：Ubuntu22.04**  
**设备：DJI Robomaster Development Board A STM32F427IIH6** or **DJI Robomaster Development Board C STM32F407IGT6**

## 一、 UART通信

### 1. UART通信原理

UART，全称 Universal Asynchronous Receiver/Transmitter，是一种在嵌入式系统中非常常见的串行通信协议。它的主要功能是将并行数据转换为串行格式发送（TX），同时将接收到的串行数据转换为并行格式供处理器读取（RX）。 UART是一种点对点的通信方式，也就是说它只在两个设备之间进行数据交换。例如，开发板（MCU）通过串口与上位机（PC）通信，或MCU与某个GPS模块通信。在这种连接中，一个设备的TX（Transmit，发送）引脚连接到另一个设备的RX（Receive，接收）引脚，反之亦然。
这种一对一连接方式使得UART通信非常直接，但也意味着你无法像I²C那样用一条总线连接多个从设备。
与一些需要共享时钟信号的通信协议（如 SPI、I²C）不同，UART是一种异步通信协议，这意味着通信双方不需要共用一个时钟线来同步数据传输。

####  异步传输：为什么不需要时钟线？
UART 被称为“异步”，是因为发送方和接收方并不共享一个时钟信号。那么它们如何知道什么时候开始或停止读取数据？这就靠一种称为“帧格式”的机制来解决。
在 UART 传输过程中，数据会被包裹在一个固定结构的帧内，该帧包含了起始位、数据位、可能的奇偶校验位、以及停止位。接收方通过起始位检测到数据即将到来，并利用事先约定好的波特率（即数据传输速率）来计算每一位的持续时间，从而按时采样数据。

UART 的数据帧结构是通信中的核心，它定义了每次传输一组数据时的规则。一个典型的 UART 帧可能如下所示：
```txt
| 起始位 (1 bit) | 数据位 (8 bits) | 校验位 (0 or 1 bit) | 停止位 (1 or 2 bits) |
```
1. 起始位通常为低电平（逻辑 0），用于通知接收端“即将开始一组数据传输”。
2. 数据位是你真正想要传输的信息，通常为 8 位（也可为 5~9 位）。
3. **校验位（可选）**用于错误检测，分为奇校验和偶校验。
4. 停止位为高电平（逻辑 1），表示一帧数据的结束。

这种结构允许接收方在没有时钟线的情况下，通过采样来正确还原出原始数据。

#### Intuition：发送端发送的数据帧，为什么接收端可以精确识别接收？--波特率：通信的节奏

为了确保接收方能正确地识别每一位数据，通信双方必须使用相同的波特率（Baud Rate），即单位时间内传输的比特数。常见的波特率有：
1. 9600 bps：稳定可靠，适合低速通信
2. 115200 bps：嵌入式开发常用，兼顾速度和可靠性
3. 更高波特率：如 921600 bps，适合高速模块，如 GPS、摄像头等

需要注意的是，波特率越高，对电路质量、干扰抑制等要求越高，失误率也会增加。

综上，UART是一种简洁高效的点对点异步通信方式。它通过 TX/RX 引脚传输串行数据帧，省去了时钟线，只需通信双方严格按照统一的帧结构和波特率，即可可靠地传输数据。起始位标识数据开始，数据位承载有效信息，校验位提升可靠性，停止位则标志帧结束，使得UART成为嵌入式设备中最基础、最常用的通信手段之一。

### 2. Zephyr中的UART驱动结构
Zephyr为串口提供了完整的抽象层，只需要在源文件里```#include <zephyr/drivers/uart.h>```
就可以使用一致的API操作不同平台上的UART控制器。
```sh
应用层（调用 UART API）
     ↓
Zephyr HAL 层（`drivers/serial/` 中的统一接口）
     ↓
SoC Vendor 提供的 UART 控制器驱动
     ↓
硬件寄存器
```
Zephyr中所有UART设备信息都由设备树描述并静态生成，这要求开发板中的设备树中有UART设备有正确的定义，并且已经启用。
随后可以通过```devicetree.h```获取对应设备。
```c
#define UART_NODE DT_NODELABEL(uart0)
const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);
```
DEVICE_DT_GET()通过Devicetree自动生成的对象查找UART实例，这个设备实例就包含了对应的driver_api函数表指针，
也就是说，这样一个操作可以使得新定义的```uart_dev```成为实现MCU串口功能的一个途径，允许程序通过操作这个标识符使用UART功能。

顺带讲一下，device结构体的定义是
```c
struct device {
    const char *name;
    const void *config;
    void *data;
    const void *api;
    ...
};
```
其中
1. ```const char *name```这个字段保存的是设备的名字字符串，比如 "UART_0"；
2. ```const void *config;```这是一个只读指针，指向与这个设备相关的硬件配置数据，例如：寄存器地址，中断号，波特率等属于设备的静态描述信息；
3. ```void *data```这是一个可读写的指针，指向设备驱动运行过程中用到的状态数据，比如当前发送缓冲区、接收缓冲区指针。中断状态标志，DMA 传输状态，某些寄存器的备份值；
4. ```const void *api;```这是这个设备的最关键成员之一，指向一个函数表结构体（api），描述了该设备支持的所有操作函数。函数表的结构因设备类型而异，例如：```struct uart_driver_api```，```struct i2c_driver_api```；

这就是Zephyr中设备驱动模型的核心基础。通过这个结构体，Zephyr 实现了平台无关、接口统一、驱动动态注册和自动管理的特性。当定义的UART变量获取的实例设备及其api以后，就可以使用```struct uart_driver_api```里面定义的函数对串口进行控制。

### 3. 通过Zephyr使用UART

如果有使用过HAL控制串口的话，一定会记得UART在HAL库里面的基本使用方式主要有轮循和中断两种。当然你也可能没有用过HAL库而是用ESP32或者Arduino等单片机进行开发，但其内核思想都是一致的。

#### 轮询模式

轮询（Polling）方式指的是：程序主动不断地检查串口硬件寄存器的状态，判断是否有数据可以接收，或者发送缓冲区是否空闲，从而进行收发操作。
它不依赖中断或 DMA，而是CPU 主动地、周期性地“轮询”硬件状态，这是一种同步、阻塞的通信方式。以接收为例，UART接收模块会把串行数据转换为字节，并将其放入一个接收寄存器。程序会进入一个循环，不断查询某个状态标志位，如果标志位为“有数据可读”，程序就从数据寄存器读取数据；
如果没有数据，就继续循环，直到等到数据或超时。
而发送逻辑类似：查询发送缓冲区是否空（如 TXE），为空则写入数据；否则等待。

在基于HAL库的STM32开发中，串口通信的实现首先需要初始化UART：
```c
HAL_UART_Init(UART_HandleTypeDef *huart);
```
然后将待发送的数据写入UART发送缓冲区，调用轮询发送函数或者调用接受函数从UART接收缓冲区读取数据：
```c
HAL_UART_Transmit(UART_Handle TypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout); //发送
HAL_UART_Receive(UART_Handle TypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);  //接受
```

类似的，Zephyr基于设备树获取设备自动完成初始化，随后通过调用发送向串口发送一个字节或者调用接受函数**非阻塞式的**从串口接收一个字节：
```c
int uart_poll_in(const struct device *dev, unsigned char *c); //发送，若发送缓冲区忙，函数会阻塞，直到能够写入
int uart_poll_in(const struct device *dev, unsigned char *c); //接受，如果有数据可以读，函数将其写入*c并返回0
```
一个简单的示例是通过UART实现MCU向PC发送消息。
```c
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
```
值得注意的是，这里串口发送是通过一个新定义的发送函数实现的，这是因为Zephyr倾向于将驱动API保持在最小通用接口层（只实现基本收发），所以个人认为可能略有不便。此外，由于Zephyr发送字符是阻塞发送，就可能导致在发长字符串的时候程序卡住。实际开发中建议用 printk()、LOG_INF()或中断模式替代长串发送到串口控制台。

#### 中断模式

UART 的中断方式（Interrupt-driven UART）指的是当串口硬件检测到事件（如收到数据、发送完成、错误）时，自动触发 CPU 的中断向量，进入 ISR（中断服务程序）处理，而不是由 CPU 主动轮询。这是硬件驱动通信的典型模式，它能让 MCU 在数据尚未到来时处理其他任务，只有在需要时才响应。

中断驱动通信是 Zephyr 推荐的正式方式，具备高效、实时、低耦合等优势,只需写一个回调函数和启用中断，就可以完全掌控串口通信.


| 项目     | 轮询模式         | 中断模式            |
| ------ | ------------ | --------------- |
| 响应机制   | CPU 主动查询     | 硬件自动触发          |
| CPU 使用 | 忙等，占用多       | 只在事件发生时响应       |
| 复杂度    | 简单           | 稍复杂，需回调         |
| 实时性    | 差            | 高，适合高速通信        |
| 应用场景   | 简单串口通信，调试、打印 | 串口控制协议、数据流、外设通信 |

使用UART中断发送或者接收需要配置相关项目
```Kconfig
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
```
两种UART中断配置
| 类型           | 说明                        | 函数                     |
| ------------ | ------------------------- | ---------------------- |
| **接收中断（RX）** | 有新字节进入 UART 接收 FIFO 时触发   | `uart_irq_rx_enable()` |
| **发送中断（TX）** | UART 发送 FIFO 空闲、可以继续发送时触发 | `uart_irq_tx_enable()` |


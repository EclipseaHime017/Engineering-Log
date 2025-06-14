# ZephyrRTOS Learning I

**作者：EclipseaHime017**  
**环境：Ubuntu22.04**  

## 一、Zephyr快速入门
### 1.ZephyrRTOS介绍
#### （1）实时操作系统（RTOS）
&emsp;&emsp; RTOS（Real-Time Operating System）是专为嵌入式系统设计的一种操作系统，强调任务响应的实时性、确定性和可靠性。在简单的嵌入式系统中，**裸机开发**就够了，但当系统变复杂时（多个外设、通信接口、实时响应等），RTOS有明显优势： 

  | 裸机	| RTOS |
  |-----|-----|
  |主循环轮询|多线程并发处理|
  |手动管理任务切换|自动调度任务|
  |同步难度大|提供同步机制|
  |实时性难以保证|具备实时调度能力|
  
&emsp;&emsp; RTOS与裸机开发在软硬件交互上的区别：  
>裸机开发（Bare-metal）：没有操作系统，代码从 main() 进入后靠 while(1) 死循环跑程序。所有任务轮询或者中断触发，任务调度需要手动控制。硬件抽象通常基于寄存器直接访问或 HAL（硬件抽象层）库。
>
>RTOS 开发：有线程/任务调度器，各功能模块运行在不同线程中。RTOS 提供同步机制（如互斥锁、信号量），实现模块之间协作。中断与线程结合更紧密，可用工作队列或消息队列异步处理。

#### （2）RTOS基本原理
&emsp;&emsp; 线程管理：RTOS 支持多个线程（任务），每个线程有自己的优先级和栈。  
&emsp;&emsp; 调度器：  
&emsp;&emsp; --基于优先级：高优先级任务先执行  
&emsp;&emsp; --时间片轮转（Time slicing）：相同优先级任务轮流执行  
&emsp;&emsp; --支持抢占（Preemptive）  
&emsp;&emsp; 中断响应：RTOS 响应硬件中断并唤醒相应线程  
&emsp;&emsp; 同步与通信机制：信号量、互斥锁、事件组、消息队列等  
&emsp;&emsp; 定时器支持：支持定时任务、超时处理

&emsp;&emsp; RTOS 并非只是“能多线程”这么简单，不同 RTOS 在 调度器架构、上下文切换、中断响应、内存分配、资源管理等核心机制上存在显著差异,以几个主流 RTOS 为例进行对比（Zephyr、FreeRTOS、RT-Thread、RTX）   

**调度器（Scheduler）**

| 特性    | Zephyr      | FreeRTOS   | RT-Thread | RTX                |
| ----- | ----------- | ---------- | --------- | ------------------ |
| 抢占式调度 | 支持          | 支持         | 支持        | 支持                 |
| 时间片轮转 | 支持          | 需手动启用      | 支持        | 支持                 |
| 优先级数目 | 0\~31       | 通常 <10（可调） | 0\~255    | 动态分配               |
| 多核支持  | ✅ SMP（对称多核） | ❌（单核）      | ❌         | ❌（CMSIS RTX5 开始支持） |

&emsp;&emsp; Zephyr 支持 SMP 多核调度，FreeRTOS 主体仍为单核（需用 SMP 分支才能支持） * Zephyr 的调度器更现代化，支持 Thread Domain、Tickless idle 等优化机制。  
&emsp;&emsp; SMP：Symmetric Multiprocessing（对称多处理），是指一个系统内有多个 对称 CPU 核心，共享主存与 I/O，操作系统调度器能将任务分配到任意核心上运行。类似桌面操作系统的任务在多个 CPU 核上自由调度。Zephyr 内核支持 SMP 架构，即在多个 CPU 核心上同时调度多个线程。  
&emsp;&emsp; 在设备树中，设置 cpus 节点，Zephyr 就能在多个核上跑多个线程。与之不同的是，FreeRTOS 是单核 RTOS（原生），FreeRTOS 设计初衷是用于资源有限的 单核 MCU（如 STM32、ESP32 单核），其任务调度器只能在 单个 CPU 核心上运行所有任务，多核支持并非主线版本，需使用 ESP-IDF 称为“FreeRTOS SMP”分支 或 Symmetric FreeRTOS 扩展版。  
一个很好的例子是ESP32双核芯片，FreeRTOS 默认：只能将所有任务固定在 core0 或 core1 上运行。Zephyr SMP 支持：能动态分配任务到任意核心，真正并行。  

**线程与上下文切换** 

&emsp;&emsp; Zephyr 提供强大的 线程生命周期管理（init/suspend/resume/abort），支持 Thread Stack Analyzing、线程监控等内核追踪机制
| 特性               | Zephyr               | FreeRTOS       | RT-Thread | RTX |
| ---------------- | -------------------- | -------------- | --------- | --- |
| 静态线程             | ✅（K\_THREAD\_DEFINE） | ✅（静态 TCB）      | ✅         | ✅   |
| 动态线程             | ✅（k\_thread\_create） | ✅（xTaskCreate） | ✅         | ✅   |
| 上下文切换时间          | 较快                   | 快              | 中         | 快   |
| Tickless idle 支持 | ✅                    | ✅              | ✅         | ✅   |

**中断处理架构**

&emsp;&emsp; Zephyr 采用 ARM CMSIS + 自定义中断抽象层，中断优先级精细管理（通过设备树和 Kconfig）
| 特性         | Zephyr           | FreeRTOS | RT-Thread | RTX |
| ---------- | ---------------- | -------- | --------- | --- |
| ISR 分离与优先级 | ✅（NVIC抽象良好）      | ✅        | ✅         | ✅   |
| 中断安全 API   | 丰富               | 少        | 中等        | 中等  |
| 软件中断/事件触发  | ✅（可通过 workqueue） | ❌        | ✅         | ✅   |

**内存管理**

&emsp;&emsp; Zephyr 提供：Slab、Heap、Mempool 多种内存分配器，用户空间与内核空间分离（可选），用户线程在非特权模式下运行，提高安全性
| 特性        | Zephyr            | FreeRTOS           | RT-Thread | RTX           |
| --------- | ----------------- | ------------------ | --------- | ------------- |
| 栈大小配置     | 每线程独立栈            | 每任务分配栈             | 同上        | 同上            |
| 动态内存分配    | 支持 heap/slab/pool | 通常靠 `pvPortMalloc` | 支持        | 支持            |
| 内存保护（MPU） | ✅（基于 ARM MPU）     | 需要额外扩展             | ❌         | ✅（CMSIS RTX5） |

**内核模块结构**
&emsp;&emsp; Zephyr 的架构更偏向完整的“类 Linux”嵌入式 OS，而 FreeRTOS 更像轻量任务调度内核。
| 模块        | Zephyr                 | FreeRTOS  | RT-Thread |
| --------- | ---------------------- | --------- | --------- |
| 线程/调度器    | ✅                      | ✅         | ✅         |
| 信号量/互斥锁   | ✅                      | ✅         | ✅         |
| 时间管理      | ✅（sys\_clock）          | ✅         | ✅         |
| 消息机制      | ✅（fifo/pipe/msgq/mbox） | 仅 queue   | ✅         |
| 网络栈       | ✅（独立模块）                | ❌（需 LwIP） | ✅         |
| 文件系统      | ✅（LittleFS, FATFS）     | ❌         | ✅         |
| Shell/命令行 | ✅（支持 RTT, UART）        | ❌         | ✅         |
| 设备树支持     | ✅                      | ❌         | ❌         |

Zephyr架构：    
--内核架构  
----线程模型：支持协程、抢占式多线程、优先级  
----调度器：抢占式优先级调度器，支持 time slicing  
----内存管理：堆/栈管理，slab 分配器  
----同步机制：信号量（k_sem）、互斥锁（k_mutex）、消息队列（k_msgq）、FIFO（k_fifo）、邮箱（k_mbox） 
--驱动模型  
----基于设备树（DeviceTree）配置硬件，驱动采用设备模型初始化流程，自动注册   

#### (3)特点：轻量、安全、可移植，支持多种芯片（如 STM32、nRF、ESP32）

#### (4)用途：物联网（IoT）、传感器网络、可穿戴设备、工业控制  


### 2.环境搭建  
（1）```west```工具（Zephyr官方提供的工作流管理工具，基于Python实现）
```west```的核心功能：
--管理Zephyr项目结构（基于 manifest 文件）  
--自动clone多个仓库（如 zephyr、hal_stm32 等）  
--构建Zephyr工程（封装 cmake & ninja）  
--调用烧录工具（如 nrfjprog, openocd, pyocd）  

安装顺序：Python & pip，Zephyr SDK，west 工具

（2）初始化项目：
首先看官方文档安装必要项目(https://docs.zephyrproject.org/latest/develop/getting_started/index.html)  
方案一(官方推荐使用 Python 虚拟环境 venv)：
```sh
python3 -m venv ~/zephyrproject/.venv
source ~/zephyrproject/.venv/bin/activate
pip install west
```
方案二（直接pip install west）  
&emsp;&emsp; 上述两种方案区别在于：前者不会干扰其他项目Python包，每个项目可以独立配置所需Python模块版本；后者无需额外理解Python虚拟环境，但是后续工作可能存在依赖冲突，不适合多人协作。  
注：此处zephyrproject是工作空间（文件夹）的名称，具体可以自定  
```sh
west init zephyrproject
cd zephyrproject
west update
west zephyr-export
```
&emsp;&emsp; 该命令是为了：  
>west init 用于初始化一个 Zephyr 项目工作区，同时创建 .west 配置目录并拉取并读取 west.yml(manifest)文件，准备项目结构(-m是用于指定一个 manifest 文件（west.yml）的 Git 仓库地址， --mr是指定你想要 checkout 的 manifest 分支或 commit hash
用于锁定 Zephyr 的版本)
>
>更新zephyr
>
>导出 Zephyr 环境变量到 shell 中，主要作用是，设置 CMake 的路径，使你可以脱离 west build，使用 cmake 单独构建 Zephyr 项目。  

## 二、Zephyr项目入门

### 1.Zephyr工程示例

Zephyr工作空间的结构通常如下图所示：  
```
zephyrproject/        # Zephyr工作空间
├── zephyr/           # Zephyr主体源码
├── modules/          # 第三方模块
├── bootloader/       # 引导程序
├── boards/           # 开发板配置（dts/kconfig）
├── samples/          # 示例工程
├── applications/     # 你自己写的工程或者应用
├── .west             # west工具的内部管理目录
└── .venv             # 工作空间专用python包
```
在```zephyr/samples/basic/blinky```中可以找到官方提供的点灯程序；
执行：
```sh
west build -b <your_board> .     # 示例: -b robomaster_board_c
west flash                       # 烧录
```
&emsp;&emsp; 可以编译并烧录程序，需要注意的是，zephyr官方只提供了部分开发板设备树，这里robomaster_board_c是没有对应的设备树和配置信息的，需要自己写，这一部分后续教程应当体现。
利用blinky程序来分析Zephyr工程的结构和必要文件。  

(1)src文件夹  
&emsp;&emsp; 这个文件里面包含的主要是项目的源代码（.c文件），对于大型项目有头文件应该包括一个include文件夹与src并列存放相应宏声明和函数定义。  
```c
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
```
&emsp;&emsp; 代码思路很简单，主函数while循环里面变换led设备状态实现亮灭，然后延时一秒，当前led状态会通过led_state变量输出。
里面有一个```gpio_dt_spec```结构体包括一个端口指针（LED0_NODE），一个gpio引脚类型变量（gpios），一个本质是uint16_t的标志；```GPIO_DT_SPEC_GET(node_id, prop)```本质是```GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, 0)```
```c
#define GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx)			       \
	{								       \
		.port = DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(node_id, prop, idx)),\
		.pin = DT_GPIO_PIN_BY_IDX(node_id, prop, idx),		       \
		.dt_flags = DT_GPIO_FLAGS_BY_IDX(node_id, prop, idx),	       \
	}
```
&emsp;&emsp; 这个宏会通过DEVICE_DT_GET()在设备树里面找到端口地址，用DT_GPIO_PIN_BY_IDX(node_id, gpio_pha, idx)->DT_PHA_BY_IDX(node_id, gpio_pha, idx, pin)找到引脚序号，
用DT_GPIO_FLAGS_BY_IDX(node_id, gpio_pha, idx)->DT_PHA_BY_IDX_OR(node_id, gpio_pha, idx, flags, 0)获取引脚电平标志。  
&emsp;&emsp; gpio_is_ready_dt() 是 Zephyr 提供的一个实用函数，用于检查由 DeviceTree 描述的 GPIO 设备是否“就绪”（ready）以供使用。具体来说，它接收一个指向 struct gpio_dt_spec 的指针（例如在示例中定义的 led），并根据该 GPIO 设备的状态判断设备是否已正确初始化和配置。   
&emsp;&emsp; gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) 是 Zephyr 中用于配置 GPIO 引脚的一个便捷接口，它基于设备树定义的硬件信息来进行引脚配置。  
&emsp;&emsp; gpio_pin_toggle_dt() 是 Zephyr 提供的一个 GPIO 操作 API，它的作用是对由设备树描述的 GPIO 引脚进行状态反转（Toggle）。也就是说，如果当前引脚处于高电平，它就会切换为低电平；反之亦然。  

(2)CMakeLists.txt  

&emsp;&emsp; 虽然Zephyr工程的构建是使用west build的，但是Zephyr的整个构建是基于CMake+west的：west 是命令行构建管理工具；CMake 是真正执行构建逻辑的系统；
而CMakeLists.txt 就是构建指令的脚本。这个CMakeLists.txt文件是 Zephyr应用工程的构建配置文件，它的主要作用是告诉 CMake 如何build这个示例工程。
```cmake
cmake_minimum_required(VERSION 3.20.0)
#指定了构建此工程所需的最低 CMake 版本为 3.20.0

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
#告诉 CMake 查找 Zephyr 项目，并且该工程必须依赖于 Zephyr。通过环境变量 ZEPHYR_BASE 提供 Zephyr 源码的路径，以便正确定位 Zephyr 的 CMake 模块和相关工具链

project(blinky)
#声明当前工程名称为 “blinky”

target_sources(app PRIVATE src/main.c)
#指定将 src/main.c 文件添加到名为 “app” 的目标中，该目标会包含应用程序的代码。这里使用 PRIVATE 关键字，表示这些源文件仅对该 target 可见
```
&emsp;&emsp; 在```target_sources(app PRIVATE src/main.c)```中，app 指的是 Zephyr 构建系统为当前应用程序自动创建的目标（target）。Zephyr 的 CMake 配置会在内部定义一个名为 app 的目标，用于链接和构建应用。在 CMakeLists.txt 中使用 target_sources(app PRIVATE src/main.c) 就是将应用程序的源文件添加到这个目标中，从而参与最终固件的编译和链接。  

&emsp;&emsp; 值得注意的一点是在find_package中有一个$ENV{ZEPHYR_BASE}代表zephyr库地址的环境变量，事实上可能很多人发现在.bashrc下面并没有相关配置仍然可以正常编译，这大概是因为在先前工程示例中提到的.west/config里面有配置。

(3)prj.conf  
&emsp;&emsp; prj.conf 是 Zephyr 配置内核和模块功能的地方，类似 Kconfig。blinky当中仅有CONFIG_GPIO=y，表示启用 GPIO 子系统。此外，如果写
```Kcofig
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_MAIN_STACK_SIZE=1024
```
就相当于你告诉 Zephyr：请启用 GPIO 驱动，启用日志子系统，主线程栈大小设为 1024 字节，最终构建系统会把这些配置项送入 Kconfig 系统中，与默认值合并，然后生成 .config 文件（供编译系统使用）。
&emsp;&emsp; Kconfig 是 Zephyr（和 Linux 内核）用于配置内核和驱动功能的“菜单系统”。可以把它当成一个“配置选项数据库”，定义了每个功能、模块的开关、依赖和提示信息。每个模块（如 GPIO、I2C、shell、LVGL）通常都会有一个 Kconfig 文件。
&emsp;&emsp; Konfig和prj.config的关系如同
```ruby
你写的 prj.conf
        ↓
合并所有模块的 Kconfig（Zephyr + 驱动 + 你自己）
        ↓
生成最终配置文件 `.config`（实际参与编译的宏定义）
        ↓
生成 autoconf.h（被 C 代码 include 使用）

```
例如，如果我想用串口，那么第一步检查使用的board的设备树里是否定义了UART0、UART1等节点，并且启用了 alias uart0,第二步在 prj.conf 开启串口功能：
```Kconfig
CONFIG_SERIAL=y
CONFIG_UART_CONSOLE=y  # 把串口设置为 console
```
总的来说，
| 项目   | `Kconfig`        | `prj.conf` |
| ---- | ---------------- | ---------- |
| 本质   | 配置项定义文件          | 配置项“值”配置   |
| 维护者  | Zephyr 各模块作者     | 用户      |
| 功能   | 定义模块开关、依赖、说明     | 启用模块、修改默认值 |
| 位置   | 各模块文件夹下          | 工程根目录      |
| 作用   | 建立配置体系           | 提交实际的配置请求  |
| 操作方式 | 手写 or menuconfig | 手写为主       |

(4)sample.yaml  
&emsp;&emsp; sample.yaml 是 Zephyr 项目中的元信息文件，定义了这个 sample 的：名称、标签分类、依赖（如 GPIO）、自动测试（CI）的规则、可运行的平台（board）。  
以 blinky 为例
```yaml
sample:
  name: Blinky Sample             # 示例工程的名称

tests:                            # 测试用例配置
  sample.basic.blinky:           # 测试用例名（用于 CI）
    tags:                        # 分类标签，可用于筛选运行特定样例
      - LED
      - gpio
    filter: dt_enabled_alias_with_parent_compat("led0", "gpio-leds")
                                  # 使用设备树条件过滤，仅在存在 led0 且其父为 gpio-leds 的板子上运行
    depends_on: gpio              # 指定该样例依赖 GPIO 驱动（硬件依赖）
    harness: led                  # 指定测试工具，led harness 会检测 LED 是否被控制
    integration_platforms:       # 自动测试时优先在这些平台运行
      - frdm_k64f
```

(5)README.rst  
类似于README.md，提供了指导性的内容。  oajfiksdjk

### APPENDIX for Learning I

#### CMSIS
CMSIS：Cortex Microcontroller Software Interface Standard，它是 ARM 官方发布的一个标准化软件接口，为基于 ARM Cortex-M 的 MCU 提供统一的编程接口。  
| 模块               | 作用                                    |
| ---------------- | ------------------------------------- |
| **CMSIS-Core**   | 提供访问 Cortex-M 寄存器、异常向量、中断管理的头文件和函数    |
| **CMSIS-DSP**    | 提供常用数字信号处理库（FFT、滤波等）                  |
| **CMSIS-RTOS**   | RTOS 抽象层，统一 RTOS 接口（如支持 FreeRTOS、RTX） |
| **CMSIS-Driver** | 抽象通用外设驱动（UART、SPI、I2C等）               |
| **CMSIS-Pack**   | 用于管理 MCU 支持包（芯片包、驱动等）                 |

CMSIS-RTOS 是 RTOS 接口的统一抽象标准，即便换 RTOS（如 FreeRTOS 或 RTX），只要实现 CMSIS-RTOS API 层，应用层代码无需修改。  
CMSIS 的意义在于：抽象硬件平台差异（不同厂商 MCU）、统一 API 接口，利于跨平台 RTOS 开发、促进代码重用和移植。






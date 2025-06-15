# ZephyrRTOS Learning II

**作者：EclipseaHime017**  
**环境：Ubuntu22.04**

## 一、Zephyr线程创建与同步机制

&emsp;&emsp;RTOS为了能够同时执行不同的任务，或者方便调度不同任务的进行，需要通过创建线程分配任务来进行。ZephyrRTOS内部使用```struct k_thread```来描述一个线程，它记录了：
>当前栈地址  
优先级  
线程状态（ready, pending, suspended, etc）
等待的资源（如信号量、互斥锁）
线程入口函数和参数
CPU 寄存器保存区（上下文）
...

&emsp;&emsp;为了便于传递，k_thread通常是用指针的形式，Zephyr官方提供了一种对应的索引```k_tid_t```，他和```struct k_thread *```完全等价。
| 类型                | 用途            | 说明                                 |
| ----------------- | ------------- | ---------------------------------- |
| `struct k_thread` | **线程控制块本体**   | 包含线程上下文、栈指针、调度状态、优先级等              |
| `k_tid_t`         | **线程标识符（ID）** | 是 `struct k_thread*`，用于内核 API 识别线程 |


此外，在讲解Zephyr创建线程之前，需要讲清楚Zephyr中线程栈（stack）的分配、管理机制。  
&emsp;&emsp;如果有c/c++的基础的话，一定会记得静态内存分配和动态内存分配和堆栈的关系。首先，Zephyr每个线程都需要一块“私有内存”来保存：函数调用的局部变量、返回地址（PC）、寄存器上下文、中断嵌套或系统调用数据，这块私有内存区域就是线程的栈（Stack），其大小和安全性直接影响线程的运行稳定性。Zephyr 会根据使用的架构（ARM, x86 等）自动设置栈对齐、保护、溢出检测。Zephyr创建栈空间的方式主要是
```c
K_THREAD_STACK_DEFINE(name, size);
```
使用 K_THREAD_STACK_DEFINE() 宏时，Zephyr会根据CPU有不同的对齐要求（ARM Cortex-M：通常 8 字节对齐，x86：通常 4 字节，RISC-V：通常 16 字节）做栈自动对齐，确保兼容性，所以不可以直接用 malloc和uint8_t stack[1024]为线程栈分配内存，会破坏 Zephyr 的堆栈保护机制。
当然可以尝试，但是可能会出现一些问题比如无法确保架构要求的栈对齐（如 ARM 要求 8/16 字节），不会自动注册到内核栈监控机制（如栈溢出检测、初始化填充），不被 Zephyr 的调度器认可为“合法线程栈”，调度时会出错甚至 HardFault，若放在 main() 中，栈位置可能和主线程栈冲突或被优化掉等等，
所以既然都已经提供了一个很好的分配方式了，为什么不用呢？  
&emsp;&emsp;与栈相关有很多配置可以在对应prj.conf里面打开，例如
```Kconfig
CONFIG_HW_STACK_PROTECTION=y #栈保护
CONFIG_INIT_STACKS=y #栈初始化
CONFIG_THREAD_STACK_INFO=y #栈使用分析
CONFIG_THREAD_NAME=y     # 便于查看线程名字
```

其中栈保护防止线程栈越界访问其他内存区域（如堆、其他线程栈、全局区）造成严重错误，当线程栈溢出触碰这块内存时，系统将触发硬件异常。栈初始化会初始化线程栈区域，用一个固定值填充，以便后续通过stack_unused_space_get()分析“实际用掉多少栈”。
栈使用分析运行时或开发后期判断：栈是否溢出？栈空间是否浪费？实际使用了多少？是否该缩小/扩大某个线程的栈？同样可以使用thread_analyzer模块，添加

```Kconfig
CONFIG_THREAD_ANALYZER=y
CONFIG_THREAD_ANALYZER_RUN_UNLOCKED=y
CONFIG_THREAD_ANALYZER_AUTO=y
```

可以立刻看到哪些线程栈配置过大（浪费内存），哪些快要溢出。既然有可能溢出，那么栈大小要设置多大？作者目前的一个参考设置大小是
| 功能                       | 建议栈大小（ARM Cortex-M） |
| ------------------------ | ------------------- |
| 简单 blinky 或 LED 控制       | 512 \~ 1024 B       |
| GPIO + I2C 传感器轮询         | 1024 \~ 2048 B      |
| 网络处理 / 串口解析 / JSON       | 2K \~ 4K            |

&emsp;&emsp;回到正题，Zephyr线程创建的方式主要有两种，运行时创建和静态创建，还有一种基于动态分配（k_malloc)的线程创建（本质和运行时类似）作者尚未探索，可能后续会加入。

### 1. 运行时线程创建

>核心函数 ```k_thread_create()```

&emsp;&emsp;完整函数是

```c
struct k_thread *k_thread_create(struct k_thread *new_thread,
                                 k_thread_stack_t *stack,
                                 size_t stack_size,
                                 k_thread_entry_t entry,
                                 void *p1, void *p2, void *p3,
                                 int priority, uint32_t options,
                                 k_timeout_t delay);
``````

&emsp;&emsp;首先```new_thread```是一个k_thread\*类型的结构体指针，需要在运行时手动定义；```stack```是栈空间指针，必须使用 K_THREAD_STACK_DEFINE() 宏分配，通常在main前面分配。```stack_size```是size_t类型的，表示栈大小，单位是字节。
```entry```表示的是函数的入口，本质是一个任意类型指针（函数指针）有三个参数，最后三个参数分别是优先级，创建选项（0）和延迟启动时间，常用K_NO_WAIT 或 K_MSEC(n)。

&emsp;&emsp;在运行时创建线程，要手动定义k_thread这个结构体，并且需要提前分配栈空间，这与静态方式（K_THREAD_DEFINE）不同。
MCU启动后，main() 中根据设定调用 k_thread_create()，Zephyr 内部将 thread 控制块加入调度队列，设置好入口函数、优先级、栈指针等，等待调度器调度进入运行态，随后运行入口函数，退出后线程自动结束。注意，尽可能不要在线程里面分配动态内存。k_thread_create()会返回一个```struct k_thread *```也就是```k_tid_t```用于后续对线程进行维护。

有一些相关的API，如
```c
k_thread_abort(struct k_thread *thread); //立即终止线程，并释放资源

k_thread_suspend(struct k_thread *thread); //将线程暂停，不再调度

k_thread_resume(struct k_thread *thread); //恢复暂停的线程

k_thread_name_set() ;
k_thread_name_get() ; //设置和获取线程的名字（调试用）
```
此外，对于main函数存在其他工作的情况下，可以在prj.conf添加
```Kconfig
CONFIG_MAIN_STACK_SIZE=1024
CONFIG_PRINTK=y
CONFIG_THREAD_NAME=y
```
### 2.静态线程创建

&emsp;&emsp;除了运行时创建线程，Zephyr 提供了一种宏 K_THREAD_DEFINE() 用于在编译时静态创建线程。其定义形式为
```c
K_THREAD_DEFINE(name, stack_size, entry_function,
                p1, p2, p3, priority, options, delay);
```
&emsp;&emsp;参数和k_thread_create()几乎一致，除了前两项stack_size这里与k_thread_create需要提供对应的线程栈地址不一样，K_THREAD_DEFINE只需提供一个大小即可，栈会随后被自动创建，而且其他的相关变量也会基于提供的标识符name一样被自动创建，而运行时创建需要手动创建线程栈和线程结构体。例如：
```c
K_THREAD_DEFINE(led_tid, 1024, led_task, NULL, NULL, NULL, 3, 0, 0);
```
那么，下面这些变量会自动被创建，同时可以根据需要被调用，
| 变量名             | 类型                     | 说明                            |
| --------------- | ---------------------- | ----------------------------- |
| `led_tid`       | `k_tid_t`              | 线程 ID，可传入如 `k_thread_abort()` |
| `led_tid_data`  | `struct k_thread`      | 线程控制块（TCB）                    |
| `led_tid_stack` | `K_THREAD_STACK_ARRAY` | 线程栈空间                         |

与运行时线程创建的对比总结
| 比较项   | `K_THREAD_DEFINE()`（静态） | `k_thread_create()`（动态）                     |
| ----- | ----------------------- | ------------------------------------------- |
| 创建时间  | 编译时 / 链接时               | 运行时                                         |
| 内存分配  | 静态（在 .noinit 段）         | 静态或动态，需手动申请栈空间                              |
| 栈管理   | 自动定义                    | 使用 `K_THREAD_STACK_DEFINE`                  |
| 使用场景  | 常驻线程、初始化线程              | 临时线程、动态扩展线程池                                |
| 启动控制  | 自动启动                    | 可配置延迟或条件启动                                  |
| 接口复杂度 | 简洁                      | 灵活但复杂                                       |
| 生命周期  | 程序运行期间常驻                | 可动态创建、销毁                                    |
| 回收机制  | 无需手动释放                  | 建议调用 `k_thread_abort()` 或 `k_thread_join()` |
| 性能开销  | 更少                      | 略高                                          |

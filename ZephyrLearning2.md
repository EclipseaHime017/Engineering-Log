# ZephyrRTOS Learning II - ZephyrRTOS线程创建和同步机制

**作者：EclipseaHime017**  
**环境：Ubuntu22.04**

注：使用线程和同步机制需要在源代码中添加
```c
#include <zephyr/kernel.h>
```

## 一、Zephyr线程创建

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

## 二、线程优先级与调度

&emsp;&emsp;线程同步机制是ZephyrRTOS多线程的核心内容，目的在于安全、高效地组织多个线程之间的数据共享与协作。Zephyr三大同步机制：
| 同步机制                     | 用于解决的问题     | 特点与用途                |
| ------------------------ | ----------- | -------------------- |
| **信号量** (Semaphore)      | 通知 / 控制线程数量 | 控制访问次数，线程间通知、计数型资源控制 |
| **互斥锁** (Mutex)          | 共享资源互斥访问    | 临界区保护，支持优先级继承        |
| **消息队列** (Message Queue) | 线程间传递结构化数据  | 队列模型，线程安全通信          |

### 1.信号量```k_sem```
&emsp;&emsp;在嵌入式多线程系统中，多个任务往往需要协调地工作，例如：等待另一个任务的结果，等待硬件事件的发生（如传感器中断），受限资源的共享使用（如串口、I2C总线），控制任务的执行节奏（如间隔启动）。
信号量的目的就是为了解决这种线程间的协调与同步问题，信号量是 RTOS 中实现线程间同步与资源访问控制的核心机制，它使并发系统能够高效、可靠地运行而不会陷入冲突与混乱。  

&emsp;&emsp;信号量的主要目的和用途在于：  
（1） 线程间同步：通知某个事件已经发生  
信号量常用于一个线程“告诉”另一个线程，“某个条件已经满足，你可以继续了”。比如：一个后台线程正在处理数据，主线程在完成采集后，通过释放信号量通知它可以开始处理。
类似“门铃”机制：主线程按铃，子线程被唤醒执行，这种模式下的信号量通常是初值为0的二值信号量（binary semaphore）。  

（2）线程等待硬件事件  
嵌入式系统中，很多线程是等待外部中断或硬件状态变化后才执行某项操作。比如：串口接收中断发生时，ISR 给信号量“打招呼”，等待数据的线程被唤醒。
保证线程不会因忙等而浪费 CPU 资源，提升能效。这种场景中，信号量实现了线程与中断服务程序（ISR）之间的通信桥梁。  

（3） 控制对有限资源的访问（并发管理）  
当多个线程要访问一个数量受限的资源时（比如只有两个传感器可用），信号量可以起到资源计数器的作用：
每个线程想访问时“获取”信号量，使用完后“归还”，如果资源已满（信号量为0），其他线程会阻塞等待，一种“排号取号”的访问制度，避免了资源冲突。这类信号量称为计数信号量（counting semaphore），其值反映资源可用数量。  

（4）限速或节拍控制  
某些任务不应该无限制运行，而是按节奏运行（比如每秒执行一次采样），主线程可以周期性地释放信号量，子线程则在信号量处等待，每收到一个信号运行一次。这种方式比 k_sleep() 等更灵活，能应对外部事件节奏变化。  

&emsp;&emsp;信号量的结构定义是
```c
struct k_sem {
    _wait_q_t wait_q;  // 挂起等待信号量的线程队列
    unsigned int count; // 当前可用的信号量资源数量
    unsigned int limit; // 信号量允许累积的最大值（上限）
};
```
如果要在线程中使用信号量，Zephyr提供了一系列API：  
（1）初始化
```c
struct k_sem my_sem;
k_sem_init(&my_sem, initial_count, limit);
```
参数定义：  
>initial_count: 初始信号量计数（可为 0）  
limit: 最大允许的计数值（防止无限增加）
 
如果，
```c
k_sem_init(&my_sem, 2, 3);
```
那么这意味着：当前有 2个“token” 可供线程 take，最多允许累积到 3个 token，如果再 give 超过就无效。
如果初始化一个信号量为k_sem_init(&my_sem, 0, 1); 这个信号量也被称为二值信号量（binary semaphore）  

静态创建信号量
```c
K_SEM_DEFINE(name, initial_count, limit)
```

（2）等待
```c
int k_sem_take(struct k_sem *sem, k_timeout_t timeout);
```
>超时参数：  
K_FOREVER: 永久阻塞  
K_NO_WAIT: 不阻塞，立即返回  
K_MSEC(n): 最多等待 n 毫秒  

>返回值：  
0: 获取成功  
-EAGAIN: 非阻塞获取失败  
-EWOULDBLOCK: 超时未获取  

&emsp;&emsp;当 k_sem_take() 被调用时，如果 sem->count > 0：立即减 1，线程继续运行；如果 sem->count == 0：当前线程会被放入 sem->wait_q 队列，并挂起；
内核调度器将不再调度此线程，直到：有其它线程/ISR 调用 k_sem_give()，释放一个信号量，唤醒它；
或者超时（如果设置了 timeout），线程被从等待队列移除（内核会将其从 wait_q 队列中移除），该线程会恢复运行，k_sem_take() 返回 -EAGAIN（获取失败）。  

（3）释放信号量
```c
k_sem_give(&my_sem);
```
&emsp;&emsp;**信号量的本质不是“谁用完了谁释放”，而是“释放资源以唤醒等待线程”**，信号量和互斥锁（mutex）不同，它不是线程占有，用完再释放的过程。
信号量控制的是某个“可用资源的数量”，k_sem_give() 的行为是：增加信号量的可用“资源数量”，唤醒等待的线程（如果有的话）。

&emsp;&emsp;```k_sem_give()```通常用于两类情况：  
**某个事件已经发生，通知等待线程继续执行**:一个线程处理完某项任务，然后 k_sem_give() 通知其他线程“你可以接着干了”, 表示“事件发生了”；  
**某个资源被释放回池，允许其他线程获取**:例如：有2个串口，初始化信号量 count=2,每个线程 k_sem_take() 表示“我要用串口”，用完后 k_sem_give()，表示“串口归还了”,所以信号量是资源的“可用数计数器”。  
简洁一句话总结：k_sem_give() 的作用是：“释放一个资源”或“发出一次事件信号”，不代表“谁用谁还”，而是由逻辑流程决定是否归还。

&emsp;&emsp;一个信号量的应用实例是如何周期点亮LED灯，虽然仅仅是为了周期点亮一个led使用信号量有点\*\*\*\*\*\*的意味，但毕竟还是比较简单容易实操，可以基于先前的blinky例子进行修改。
```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static struct k_sem led_sem; //定义信号量

// 控制线程：定时发信号
void controller_thread(void)
{
    while (1) {
        k_sem_give(&led_sem);  //发信号
        k_msleep(1000); //每秒发一次
    }
}

// LED 线程：等待信号然后点亮
void led_thread(void)
{
    while (1) {
        k_sem_take(&led_sem, K_FOREVER); //等待控制线程给信号

        gpio_pin_set_dt(&led, 1);  //点亮
        k_msleep(200);      //点亮200ms
        gpio_pin_set_dt(&led, 0);  //熄灭
    }
}

//静态线程创建
K_THREAD_DEFINE(controller_tid, 512, controller_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(led_tid, 512, led_thread, NULL, NULL, NULL, 6, 0, 0);

//main函数中初始化LED和信号量
void main(void)
{
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    k_sem_init(&led_sem, 0, 1);  // 初始信号量为 0，上限为 1
}
```
这段代码里使用了一些gpio的函数```gpio_pin_set_dt```和先前的```gpio_pin_toggle_dt```，这和STM32的HAL库函数里操作GPIO```HAL_GPIO_WritePin()```的类似。

### 2.互斥锁```k_mutex```

&emsp;&emsp;在RTOS中，多线程之间**并发访问共享资源（如全局变量、外设等）**是常见场景。如果不加控制，多个线程同时读写会导致：
>数据竞争（race condition）（多线程同时访问共享资源（如变量、SPI 总线））  
状态错乱（比如两个线程都在更新某个 LED 状态）
不可预测行为，严重时系统崩溃

&emsp;&emsp;这时候我们就需要一种“线程之间排队访问”的机制 —— 互斥锁（mutex）。一个互斥锁（mutex）是一种同步原语，用于保证在任意时刻，最多只有一个线程可以访问某个共享资源，就像一把钥匙控制一个房间，只有持有钥匙的人可以进入。
其他人必须等钥匙被释放，才能继续。
| 特性      | 互斥锁 (`k_mutex`) | 信号量 (`k_sem`)      |
| ------- | --------------- | ------------------ |
| 控制权限    | 一次只允许一个线程进入     | 可以多个线程并发（只要计数 > 0） |
| 用于      | 保护共享资源          | 线程间事件通信、资源配额       |
| 拥有者限制   | 有，仅持有线程可解锁      | 无，任意线程可 give/take  |
| 支持优先级继承 | 是             | 否                |
| **核心目的**        | **互斥访问共享资源**      | **线程间通信 / 控制访问许可数量**      |
| **计数值**         | 二元锁（通常只允许一个线程持有）  | 整数计数器，表示可用“许可”数量          |
| **是否区分线程**      | 是，谁加锁必须谁解锁        | 否，线程 A give，线程 B take 也可以 |
| **可重入（递归）**     | ❌ Zephyr 中不支持递归锁  | ✅ 可多次 take/give 配合使用      |
| **阻塞行为**        | 没有锁就阻塞等待          | 没有许可就阻塞等待                 |
| **适用于**         | 保护**共享资源访问**      | 控制**资源数量**或**任务协作**       |
| **释放者是否必须是拥有者** | 是（不能由别的线程 unlock） | 否（任何线程都可以 give）           |


&emsp;&emsp;互斥锁的结构体定义是
```c
struct k_mutex {
	struct _k_mutex_data *base;
};

struct _k_mutex_data {
	struct k_spinlock lock;
	struct k_thread *owner;
	uint32_t lock_count;
	sys_dlist_t wait_q;
};
```
| 成员           | 类型                  | 说明                   |
| ------------ | ------------------- | -------------------- |
| `lock`       | `struct k_spinlock` | 用于保证多核/中断安全地访问该结构    |
| `owner`      | `struct k_thread *` | 当前拥有该互斥锁的线程指针        |
| `lock_count` | `uint32_t`          | 当前锁嵌套计数（用于支持递归锁）     |
| `wait_q`     | `sys_dlist_t`       | 等待队列，挂起等待该 mutex 的线程 |

(1)初始化

```c
void k_mutex_init(struct k_mutex *mutex);
```
初始化一个互斥锁变量，在使用前必须调用一次。 

静态创建互斥锁（无需手动初始化）
```c
K_MUTEX_DEFINE(name)
```
动态初始化：适用于运行时决定是否创建、或在很多实例中按需初始化。  
静态创建：适用于全局或固定的共享资源保护场景。更简洁且减少遗漏初始化的风险。  

(2)加锁
```c
int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout);
```
参数说明：
>mutex：要获取的互斥锁对象；  
timeout：  
--K_FOREVER: 一直阻塞等待直到获取成功；  
--K_NO_WAIT: 立即返回；  
--K_MSEC(x)：等待指定时间（毫秒）；  

返回值：
>0：成功获取；  
-EBUSY：立即尝试失败；  
-EAGAIN：超时；  
-EINVAL：非法参数。  

(3)解锁
```c
int k_mutex_unlock(struct k_mutex *mutex);
```
返回值：
>0：成功释放；  
-EPERM：非 owner 线程尝试释放；  
-EINVAL：非法参数；  

(4)注意事项  
&emsp;&emsp;Zephyr 的 k_mutex 是 不可递归加锁（non-recursive） 的。如果一个线程对 mutex 多次 lock，会陷入死锁。死锁是指两个或多个线程在等待对方释放资源，导致彼此都无法继续执行的状态。
死锁的典型场景：
>线程 A 获得了 mutex A，等待 mutex B；
线程 B 获得了 mutex B，等待 mutex A；
相互等待，永远阻塞。

&emsp;&emsp;Zephyr的 k_mutex 是非递归的（non-recursive），且并不自动检测死锁。若使用不当，线程会永久挂起在等待队列中，表现为程序“卡住”。

&emsp;&emsp;我们来详细展开一个使用互斥锁保护LED操作的示例，在一个多线程系统中，如果多个线程要控制同一个外设（如GPIO控制的LED），而这些操作不是原子的（比如要先设定再等待、再设定），
就会出现竞态条件（race condition），导致行为不一致甚至崩溃。比如线程 A 要点亮 LED（高电平），延迟一段时间后熄灭；线程 B 也要点亮 LED，但延迟时间不同。若两个线程并发访问 LED，没有同步机制，
可能导致LED 刚被点亮就被另一个线程熄灭；

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

// 获取 LED 设备节点
#define LED_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

// 定义一个互斥锁
static struct k_mutex led_mutex;

// 线程 A：点亮 LED 1 秒
void thread_a(void)
{
    while (1) {
        k_mutex_lock(&led_mutex, K_FOREVER); // 获取锁

        printk("Thread A 控制 LED\n");
        gpio_pin_set_dt(&led, 1);  // 点亮 LED
        k_msleep(1000);            // 持续 1 秒
        gpio_pin_set_dt(&led, 0);  // 熄灭 LED

        k_mutex_unlock(&led_mutex); // 释放锁
        k_msleep(500);              // 等待下一轮
    }
}

// 线程 B：点亮 LED 200ms
void thread_b(void)
{
    while (1) {
        k_mutex_lock(&led_mutex, K_FOREVER); // 获取锁

        printk("Thread B 控制 LED\n");
        gpio_pin_set_dt(&led, 1);  // 点亮 LED
        k_msleep(200);            // 持续 200ms
        gpio_pin_set_dt(&led, 0);  // 熄灭 LED

        k_mutex_unlock(&led_mutex); // 释放锁
        k_msleep(800);              // 等待下一轮
    }
}

// 静态线程定义
K_THREAD_DEFINE(tid_a, 512, thread_a, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(tid_b, 512, thread_b, NULL, NULL, NULL, 6, 0, 0);

void main(void)
{
    // 初始化 LED 输出
    if (!device_is_ready(led.port)) {
        printk("LED device not ready\n");
        return;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    // 初始化互斥锁
    k_mutex_init(&led_mutex);
}
```

### 3.消息队列```k_msgq```

&emsp;&emsp;RTOS中，线程（或任务）之间通常需要交换数据，而不仅仅是同步“事件”。例如：一个传感器线程采集数据，另一个线程处理数据；一个主控线程下发指令，多个执行线程按指令动作。
这种**需要传递“结构化数据”**的场景，信号量和互斥锁就不够用了。  
>消息队列的核心目的：在任务之间安全、可靠、解耦地传递数据（消息）。  

**消息队列机制原理**: 消息队列是一个线程安全的循环缓冲区，用于存放固定大小的“消息”。  

| 特性     | 消息队列 (`k_msgq`) | 信号量 (`k_sem`) | 互斥锁 (`k_mutex`) |
| ------ | --------------- | ------------- | --------------- |
| 数据传递   | ✅ 支持完整消息        | ❌ 无数据，仅标志     | ❌ 仅保护访问         |
| 异步解耦   | ✅ 典型用途          | ✅ 一定程度        | ❌ 持锁期间同步        |
| 多线程支持  | ✅ 支持多生产者/消费者    | ✅             | ✅               |
| ISR 支持 | 只支持 `put`（非阻塞）  | 仅 `give` 支持   | ❌ ISR 禁止使用      |
| 用途     | 数据交换 + 通信       | 通知、控制节拍       | 保护临界资源          |

消息队列的定义是
```c
struct k_msgq {
	struct k_queue queue;        /* 内部使用的队列结构（用于管理等待线程） */
	uint32_t msg_size;           /* 每个消息的大小（单位：字节） */
	uint32_t max_msgs;           /* 最大消息数量 */
	char *buffer_start;          /* 消息缓冲区的起始地址 */
	char *buffer_end;            /* 消息缓冲区的结束地址（= start + msg_size * max_msgs） */
	char *read_ptr;              /* 当前读取位置指针 */
	char *write_ptr;             /* 当前写入位置指针 */
	size_t used_msgs;            /* 当前消息数量 */
	_wait_q_t wait_q;            /* 等待队列（用于挂起等待取消息的线程） */
};
```

| 成员名                    | 类型   | 含义                     | 举例或备注                                    |
| ---------------------- | ---- | ---------------------- | ---------------------------------------- |
| `struct k_queue queue` | 内部结构 | 实现底层同步机制，如挂起线程、排队唤醒等   | Zephyr 的队列系统封装了 `_wait_q_t`              |
| `uint32_t msg_size`    | 整数   | 每条消息的字节长度              | 比如：`sizeof(struct sensor_data)`          |
| `uint32_t max_msgs`    | 整数   | 最多能容纳多少条消息             | 总容量 = `msg_size * max_msgs`              |
| `char *buffer_start`   | 指针   | 指向消息存储区的首地址            | 内存由 `K_MSGQ_DEFINE()` 或 `msgq_init()` 提供 |
| `char *buffer_end`     | 指针   | 指向存储区尾部地址（非含义上的“最后一条”） | 计算方式是 `start + size * count`             |
| `char *read_ptr`       | 指针   | 下一个消息读取的位置             | 接收线程使用                                   |
| `char *write_ptr`      | 指针   | 下一个消息写入的位置             | 发送线程使用                                   |
| `size_t used_msgs`     | 整数   | 当前队列中有效消息数量            | 判定空满的依据                                  |
| `_wait_q_t wait_q`     | 内核结构 | 等待消息的线程列表（阻塞的消费者）      | 每个等待线程都会排队挂起在这里                          |

(1)消息队列创建  
静态创建:  
```c
K_MSGQ_DEFINE(name, msg_size, max_msgs, align)
```
例如
```c
K_MSGQ_DEFINE(my_msgq, sizeof(int), 10, 4);  // 定义一个最多放10个 int（每个4字节）的队列
```

动态创建：
```c
struct k_msgq my_msgq;
k_msgq_init(&my_msgq, buffer, msg_size, max_msgs);
```
例如
```c
char my_buffer[10 * sizeof(int)];
struct k_msgq my_msgq;

void main(void) {
    k_msgq_init(&my_msgq, my_buffer, sizeof(int), 10);
}
```
&emsp;&emsp;需要注意到，动态创建消息队列不仅仅是提供一个声明一个结构体然后初始化这个结构体，还需要在初始化的时候提供一个对应的缓存空间用来储存数据。  
如果分配到的 buffer 大小与预期不一致（过小或过大），会带来不同的问题。  
>过小：消息队列在写入或读取时可能越界，导致内存覆盖、不可预测错误，甚至系统奔溃。
过大：不会直接导致越界，但会浪费 RAM；且如果 buffer 对齐不当，也可能潜在影响性能或触发对齐相关的运行时错误。

(2)消息发送
```c
int k_msgq_put(struct k_msgq *q, const void *data, k_timeout_t timeout);
```
&emsp;&emsp;运行流程：检查队列是否已满（used_msgs == max_msgs）；  
	若未满：拷贝 data 到 buffer[write_index]write_index++，used_msgs++  
	若有等待线程在 get，直接唤醒一个线程并将数据交给它；  
	若已满：若 timeout != K_NO_WAIT，当前线程被挂起，加入队列；
	否则立即返回 -ENOMSG；
	若在中断中，只允许使用 K_NO_WAIT（不可阻塞）版本。

(3)消息接受
```c
int k_msgq_get(struct k_msgq *q, void *data, k_timeout_t timeout);
```
&emsp;&emsp;运行流程：检查队列是否为空（used_msgs == 0）；  
	若非空：从 buffer[read_index] 拷贝数据到 data；read_index++，used_msgs--  
	若有线程等待 put（因队列满），唤醒一个并允许其发送；  
	若为空：若 timeout != K_NO_WAIT，当前线程被挂起；  
	否则立即返回 -ENOMSG；  

(4)应用实例
&emsp;&emsp;生产者线程模拟周期性采集传感器数据（如温度、湿度），把数据打包成消息，投递到消息队列中。消费者线程等待消息队列有数据后，取出并“处理”该数据（这里用打印模拟处理逻辑）。
如果消息队列满了，生产者可选择阻塞等待或直接丢弃/覆盖旧数据（本例用阻塞等待方式）。
这样，生产者和消费者在时序上解耦：生产者每隔固定时间产生数据，无需关心消费者处理速度；消费者一有数据就处理，无需关心何时产生。

首先定义消息和消息队列
```c
#include <zephyr/kernel.h>

#define MSGQ_MAX_MSGS 5

struct sensor_data {
    uint32_t timestamp;   // 采集时刻的简单标记（可用 k_uptime_get_32()）
    int16_t temperature;  // 温度，单位: 0.01°C（举例）
    int16_t humidity;     // 湿度，单位: 0.01% RH（举例）
};

K_MSGQ_DEFINE(sensor_msgq,
              sizeof(struct sensor_data),
              MSGQ_MAX_MSGS,
              4);
```
生产者线程和消费者线程
```c
#define PRODUCER_STACK_SIZE 512
#define PRODUCER_PRIORITY   5

void producer_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct sensor_data data;
    uint32_t count = 0;

    while (1) {
        // 模拟采集：生成一个简单递增的“温度/湿度”值
        data.timestamp = k_uptime_get_32();
        data.temperature = 2000 + (count % 100);  // 示例值
        data.humidity    = 5000 + (count % 100);  // 示例值

        // 投递消息：若队列满，则阻塞等待（最多等待 500 ms）
        int ret = k_msgq_put(&sensor_msgq, &data, K_MSEC(500));
        if (ret == 0) {
            printk("[Producer] Put data: time=%u, temp=%d, hum=%d\n",
                   data.timestamp, data.temperature, data.humidity);
        } else if (ret == -EAGAIN) {
            // 超时未能 put：队列仍然满。可选择丢弃或其他处理
        }
        count++;
        // 生产周期：每 300 ms 采集一次
        k_sleep(K_MSEC(300));
    }
}

// 静态定义生产者线程
K_THREAD_DEFINE(producer_tid, PRODUCER_STACK_SIZE, producer_thread, NULL, NULL, NULL, PRODUCER_PRIORITY, 0, 0);

#define CONSUMER_STACK_SIZE 512
#define CONSUMER_PRIORITY   6

void consumer_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct sensor_data recv;

    while (1) {
        // 从消息队列取消息：若队列空，则永久等待
        int ret = k_msgq_get(&sensor_msgq, &recv, K_FOREVER);
        if (ret == 0) {
            // 模拟处理
            // 模拟处理耗时
            k_sleep(K_MSEC(150));
        } else {
            // 理论上 K_FOREVER 不会返回错误
        }
    }
}

// 静态定义消费者线程
K_THREAD_DEFINE(consumer_tid,
                CONSUMER_STACK_SIZE,
                consumer_thread,
                NULL, NULL, NULL,
                CONSUMER_PRIORITY, 0, 0);
```
最终主函数
```c
void main(void)
{
   //啥都不需要，主函数也是一个线程
}

```

# C/C++ Tutorial

## Outline

Structure of C: What comprise a C program?
Fundamental Concept of C language: Codes, Compilers, IDE
Variables: integers, floats, characters(ASCII and Unicode)
Complex Variables: Struct, Enum and Union
Array and Pointer
Logic flow(IF, For, While and Switch)
Function and Function Pointers
Class(If possible)

## 1. Structure of C: What comprise a C program?

一个标准的 C 程序通常包含以下几个部分：
预处理指令（如 #include <stdio.h>）：告诉编译器在编译前需要引入的头文件或宏。
全局声明：如全局变量、函数声明。
主函数 main()：程序的入口，执行从这里开始。
其他函数：辅助功能的实现。
```c
#include <stdio.h>   // 预处理指令

int add(int a, int b); // 函数声明

int main() {          // 主函数入口
    int result = add(3, 4);
    printf("Result: %d\n", result);
    return 0;         // 程序结束
}

int add(int a, int b) { // 函数实现
    return a + b;
}
```

## 2. Fundamental Concept of C language: Codes, Compilers, IDE

源代码（Code）：程序员编写的 .c 文件。
编译器（Compiler）：如 GCC、Clang，将 C 代码翻译成可执行的机器指令。
IDE（集成开发环境）：如 VS Code、CLion、Code::Blocks，集成了编辑器、编译器、调试器。

📌 编译流程：
预处理 (cpp)：展开 #include，处理 #define。
编译 (cc1)：把 C 转换为汇编。
汇编 (as)：把汇编转换为目标文件 .o。
链接 (ld)：把多个 .o 和库文件合并成可执行文件。

## 3. Variables: Integers, Floats, Characters (ASCII and Unicode)

在 C/C++ 中，变量是存储数据的命名容器，它们占用内存空间，类型决定了存储的数据和占用的大小。

整型（Integer）存储整数值（正数、负数、零）
包括 int, short, long 等，区别主要在于可表示的范围和内存占用。

浮点型（Float/Double）存储小数或科学计数法表示的数值
float 精度较低，double 精度较高

字符型（char）存储单个字符，一般用 ASCII 编码; C++ 可以使用 wchar_t 或 char16_t/char32_t 支持 Unicode，表示多字节或国际字符

变量声明与初始化
声明：告诉编译器变量的类型和名称
初始化：给变量赋初值
变量是内存的抽象表示，类型决定它能存储什么以及如何解释存储的二进制数据。

4. Complex Variables: Struct, Enum and Union

在 C/C++ 中，复杂变量用于组织或抽象数据：

结构体（struct）将不同类型的数据组合成一个整体用于描述“对象”的属性，但不包含行为（C++ 的类才可以）
类似现实世界的“记录卡片”，每个字段是一个属性
```c
struct Student {
    int id;
    char name[20];
};
```

枚举（enum）定义一组有名字的整型常量, 增强代码可读性和可维护性，实际存储上通常是整数，但语义更清晰
```c
enum Week { MON, TUE, WED, THU, FRI, SAT, SUN };
```
联合体（union）多个变量共享同一块内存，用于节省内存或实现“多态”存储，每次只能使用其中一个成员，访问前需明确哪一成员是有效的
```c
union Data {
    int i;
    float f;
    char c;
};
```
概念核心：这些复杂变量都是为了更高效、更有意义地组织数据，而不仅仅是单个原子值。

5. Array and Pointer

数组和指针是 C/C++ 中最核心的内存管理工具：

数组（Array）同类型元素的连续内存块，可以通过下标访问，便于批量存储和处理数据

指针（Pointer）存储内存地址的变量，可以指向任意类型变量，包括数组、结构体、函数等

数组与指针关系，数组名本质上是首元素的地址，指针运算可以实现遍历数组

指针是 C/C++ 直接操控内存的关键，它可以实现动态内存分配、函数回调、数据共享等高级功能

数组是数据容器，指针是对内存的直接引用。掌握指针是理解 C/C++ 高级特性的基础。
```c
#include <stdio.h>

int main() {
    int arr[3] = {10, 20, 30};
    int *p = arr; // 指针指向数组

    for(int i = 0; i < 3; i++) {
        printf("arr[%d] = %d, *(p+%d) = %d\n", i, arr[i], i, *(p+i));
    }
    return 0;
}

```

## 6. Control Flow (IF, For, While, Switch)

逻辑控制语句决定程序执行的路径，是程序“思考”的基础：

条件分支（if/else）根据布尔条件决定执行哪段代码，支持多级判断

循环（for / while / do-while）用于重复执行操作
for 循环适合已知次数的循环
while 循环适合条件驱动循环
do-while 保证至少执行一次

多分支选择（switch/case）针对单个变量的多个值执行不同分支，可读性和效率高于连续的 if-else

控制流是程序逻辑的核心，决定了程序在不同输入或状态下的行为，逻辑流是让程序“做决策”的机制，本质是对条件和重复执行的管理。

## 7. Function and Function Pointers

函数和函数指针是 C/C++ 实现模块化和高级控制的重要概念：

函数（Function）封装可重复执行的逻辑，提高代码复用性和可维护性，可以有参数和返回值

函数指针（Function Pointer）存储函数的地址，可作为参数传递，实现回调或动态选择函数，支持高级设计模式，如策略模式、事件处理

函数抽象行为，指针抽象“行为的引用”是 C/C++ 高度灵活和高效的基础
函数是行为的封装，函数指针是行为的变量化，让程序在运行时可以灵活选择行为。
```c
#include <stdio.h>

// 普通函数
int add(int a, int b) {
    return a + b;
}

// 函数指针作为参数
void operate(int x, int y, int (*func)(int, int)) {
    printf("Result: %d\n", func(x, y));
}

int main() {
    int (*fp)(int, int) = add; // 定义函数指针
    printf("Call via pointer: %d\n", fp(2, 3));

    operate(5, 6, add); // 传递函数指针
    return 0;
}

```
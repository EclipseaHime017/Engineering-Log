#include <stdio.h>
/**
 * @brief Overflow example in C
 * @note This program is referred from https://github.com/ShiqiYu/CPP/blob/main/week02/examples/overflow.cpp
 */
int main()
{
    int a = 56789;
    int b = 56789;
    int c = a * b; //correct result is 3223990521
    printf("c = %d\n", c);

    // unsigned int c1 = a * b;
    // printf("c1 = %u\n", c1);
    return 0;
}


#include <stdio.h>

int main() {
  int a = 5;
  float b = 3.14f;
  printf("Size of int: %zu bytes\n", sizeof(a));
  printf("Size of float: %zu bytes\n", sizeof(b));
  return 0;
}
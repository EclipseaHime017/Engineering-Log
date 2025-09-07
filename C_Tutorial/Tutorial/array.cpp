#include <iostream>
using namespace std;

int main() {
    int len = 1;
    while (len < 10) {
        int num_array2[len]; //variable-length array
        cout << "len = " << len;
        cout << ", sizeof(num_array2) = "
             << sizeof(num_array2) << endl;
        len++;
    }
    return 0;
}
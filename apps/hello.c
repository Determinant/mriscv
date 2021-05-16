#include "mriscv.h"

int main() {
    char buff[32];
    for (int i = -5; i < 5; i++)
        uprintf(buff, "Hello, world! %d\n", i);
    print(buff);
    return 0;
}

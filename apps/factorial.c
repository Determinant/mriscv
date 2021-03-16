#include "mriscv.h"

int fac(int n) {
    return n <= 1 ? 1 : n * fac(n - 1);
}

int main() {
    char buff[10];
    for (unsigned i = 0; i < 10; i++)
        bprintf(buff, "factorial(%u) = %u\n", i, fac(i));
    return 0;
}

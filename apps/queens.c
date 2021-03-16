#include "mriscv.h"

int queen(int i,
          unsigned atk_l,
          unsigned atk_r,
          unsigned atk_d, int n) {
    if (i == 0) return 1;
    int nsol = 0;
    unsigned forbid = atk_l | atk_r | atk_d;
    for (unsigned p = 0; p < n; p++)
    {
        unsigned _p = 1 << p;
        if (forbid & _p) continue;
        nsol += queen(i - 1,
            (atk_l | _p) << 1,
            (atk_r | _p) >> 1,
            atk_d | _p, n);
    }
    return nsol;
}

int nqueen(int n) {
    return queen(n, 0, 0, 0, n);
}

int main() {
    char buff[10];
    int res = nqueen(8);
    bprintf(buff, "nqueen = %d\n", res);
    return 0;
}

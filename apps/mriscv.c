static unsigned * const UART_TXDATA = (unsigned *)0x00001000;
//static const unsigned UART_RXDATA = 0x00001001;

unsigned strlen(const char *s) {
    unsigned n = 0;
    while (*s) s++, n++;
    return n;
}

void itoa(int n, char *s) {
    unsigned char neg = 0, i = 0;
    char c;
    if (n < 0) n = -n, neg = 1;
    do
    {
        s[i++] = '0' + n % 10;
        n /= 10;
    }
    while (n > 0);
    if (neg) s[i++] = '-';
    for (unsigned char j = 0, k = i - 1; j < i; j++, k--)
    {
        c = s[k];
        s[k] = s[j];
        s[j] = c;
    }
    s[i] = 0;
}

void print(const char *s) {
    unsigned n = strlen(s);
    for (unsigned i = 0; i < n; i++)
    {
        while ((*UART_TXDATA) >> 31) {} // TODO: use amoswap
        (*UART_TXDATA) = ((*UART_TXDATA) & ~0xff) | s[i];
    }
}

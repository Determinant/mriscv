#include <stdarg.h>
#include "mriscv.h"

static uint32_t * volatile const __attribute__ ((used)) UART_TXDATA = (uint32_t *)0x00001000;

uint32_t strlen(const char *s) {
    unsigned n = 0;
    while (*s) s++, n++;
    return n;
}

void int_to_string(int _n, char *s, bool sign, bool uppercase, int base) {
    bool neg = 0;
    uint8_t i = 0;
    uint32_t n;
    char c;
    if (_n < 0 && sign)
        n = -_n, neg = true;
    else
        n = _n, neg = false;
    do
    {
        uint32_t d = n % base;
        s[i++] = (d < 10 ? '0' : (uppercase ? 'A' : 'a')) + d;
        n /= base;
    }
    while (n > 0);
    if (neg) s[i++] = '-';
    for (uint8_t j = 0, k = i - 1; j < k; j++, k--)
    {
        c = s[k];
        s[k] = s[j];
        s[j] = c;
    }
    s[i] = 0;
}

void itoa(int n, char *s) {
    return int_to_string(n, s, true, false, 10);
}

int putchar(int c) {
    while ((*UART_TXDATA) >> 31) {} // TODO: use amoswap
    (*UART_TXDATA) = ((*UART_TXDATA) & ~0xff) | c;
    return c;
}

size_t print(const char *s) {
    size_t n = strlen(s);
    for (unsigned i = 0; i < n; i++)
        putchar(s[i]);
    return n;
}

int bprintf(char *buff, const char *fmt, ...) {
    size_t nwrote = 0;
    va_list ap;
    va_start(ap, fmt);
    for (const char *p = fmt; *p != '\0'; p++) {
        if (*p == '%') {
            p++;
            switch (*p) {
                case 's':
                    nwrote += print(va_arg(ap, const char *));
                    break;
                case 'd':
                    itoa(va_arg(ap, int), buff);
                    nwrote += print(buff);
                    break;
                case 'u':
                    int_to_string(va_arg(ap, unsigned), buff, false, false, 10);
                    nwrote += print(buff);
                    break;
                case 'x':
                    int_to_string(va_arg(ap, unsigned), buff, false, false, 16);
                    nwrote += print(buff);
                    break;
                case 'X':
                    int_to_string(va_arg(ap, unsigned), buff, false, true, 16);
                    nwrote += print(buff);
                    break;
                case '%':
                    putchar('%');
                    nwrote++;
                    break;
                default:
                    va_end(ap);
                    return -1;
            }
        } else {
            putchar(*p);
            nwrote++;
        }
    }
    va_end(ap);
    return nwrote;
}

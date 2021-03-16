#ifndef _MRISCV_H
#define _MRISCV_H

typedef unsigned uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

typedef int int32_t;
typedef short int16_t;
typedef char int8_t;
#ifndef __cplusplus
typedef uint8_t bool;
#define true 1
#define false 0
#endif

typedef unsigned size_t;

size_t strlen(const char *s);
void itoa(int n, char *s);
size_t print(const char *s);
int putchar(int c);
int bprintf(char *buff, const char *fmt, ...);

#endif

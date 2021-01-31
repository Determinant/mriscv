.global _start
.text

_start:
    lui sp, 0x200
    call main
halt:
    beq x0, x0, halt

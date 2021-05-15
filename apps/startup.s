.global _start
.text

_start:
    lui sp, 0x220
    call main
	nop
	nop
	nop
	nop
	nop
halt:
    beq x0, x0, halt

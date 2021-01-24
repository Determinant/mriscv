.global _start
.text

_start:
    lui sp, 0x200
	call main

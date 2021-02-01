.PHONY: all

all: \
	add.txt add.bin add.elf \
	queens.txt queens.bin queens.elf \
	forward.txt forward.bin forward.elf

%.elf: %.c startup.s
	riscv64-linux-gnu-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Wl,-Tlink.x -g -o $@ startup.s $<

%.elf: %.s startup.s
	riscv64-linux-gnu-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Wl,-Tlink.x -g -o $@ startup.s $<

%.txt: %.elf
	riscv64-linux-gnu-objdump -Mno-aliases -Mnumeric -dS $< > $@

%.bin: %.elf
	riscv64-linux-gnu-objcopy -O binary -j .text $< $@


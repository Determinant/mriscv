.PHONY: all

all: add add.txt add.bin

add: add.c
	riscv64-linux-gnu-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Wl,-Tlink.x -o $@ startup.s $<

add.txt: add
	riscv64-linux-gnu-objdump -d $< > $@

add.bin: add
	riscv64-linux-gnu-objcopy -O binary -j .text $< $@


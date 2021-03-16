.PHONY: all clean

GCC_TOOLCHAIN = ./rv32i/
CC = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-gcc
OBJDUMP = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objdump
OBJCOPY = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objcopy

OBJS :=	factorial.txt factorial.bin factorial.elf \
		hello.txt hello.bin hello.elf \
		queens.txt queens.bin queens.elf \
		forward.txt forward.bin forward.elf \
		exception.txt exception.bin exception.elf

all: $(OBJS)

clean:
	rm -f $(OBJS)

$(CC):
	[ -d riscv-gnu-toolchain ] || git clone https://github.com/riscv/riscv-gnu-toolchain.git --depth=1
	cd riscv-gnu-toolchain/; ./configure --prefix "$$(pwd)/../rv32i/" --with-arch=rv32i; make -j4; cd -; rm -rf riscv-gnu-toolchain/

%.elf: %.c $(CC) startup.s link.x memory.x mriscv.o
	$(CC) -march=rv32i -mabi=ilp32 -O2 -nostartfiles -Wl,-Tlink.x -g -o $@ startup.s $< mriscv.o

%.elf: %.s $(CC) startup.s link.x memory.x
	$(CC) -march=rv32i -mabi=ilp32 -O2 -nostdlib -Wl,-Tlink.x -g -o $@ startup.s $<

%.txt: %.elf
	$(OBJDUMP) -Mno-aliases -Mnumeric -dS $< > $@

%.bin: %.elf
	$(OBJCOPY) -O binary -j .text -j .rodata -j .data $< $@

mriscv.o: mriscv.c $(CC)
	$(CC) -march=rv32i -mabi=ilp32 -O2 -nostdlib -g -o $@ -c $<

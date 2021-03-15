.PHONY: all clean cargo

OBJS :=	hello.txt hello.bin hello.elf \

all: $(OBJS)

clean:
	rm -f $(OBJS)

cargo:
	cargo build --release --examples

%.elf: cargo
	cp target/riscv32i-unknown-none-elf/release/examples/$(basename $@) $@

%.elf: %.s startup.s link.x memory.x
	riscv64-linux-gnu-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Wl,-Tlink.x -g -o $@ startup.s $<

%.txt: %.elf
	riscv64-linux-gnu-objdump -Mno-aliases -Mnumeric -dS $< > $@

%.bin: %.elf
	riscv64-linux-gnu-objcopy -O binary -j .text -j .rodata $< $@

mriscv.o: mriscv.c
	riscv64-linux-gnu-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -g -o $@ -c $<

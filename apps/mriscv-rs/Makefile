.PHONY: all clean cargo

GCC_TOOLCHAIN = ../rv32i/
OBJDUMP = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objdump
OBJCOPY = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objcopy

OBJS :=	hello.txt hello.bin hello.elf \

all: $(OBJS)

clean:
	rm -f $(OBJS)

cargo:
	cargo build --release --examples

%.elf: cargo
	cp target/riscv32i-unknown-none-elf/release/examples/$(basename $@) $@

%.txt: %.elf
	$(OBJDUMP) -Mno-aliases -Mnumeric -dS $< > $@

%.bin: %.elf
	$(OBJCOPY) -O binary -j .text -j .rodata $< $@

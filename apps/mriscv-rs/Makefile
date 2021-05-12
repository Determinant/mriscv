.PHONY: all clean cargo

#GCC_TOOLCHAIN = ../rv32i/
#OBJDUMP = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objdump
#OBJCOPY = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objcopy

OBJS :=	\
	hello.txt hello.bin hello.elf \
	video.txt video.bin video.elf \
	snake.txt snake.bin snake.elf \

all: $(OBJS)

clean:
	rm -f $(OBJS)

cargo:
	cargo build --release --examples

%.elf: cargo
	cp target/riscv32i-unknown-none-elf/release/examples/$(basename $@) $@

%.txt: %.elf
	cargo-objdump --release --example $(basename $@) -- -Mno-aliases -Mnumeric -dS > $@
	#$(OBJDUMP) -Mno-aliases -Mnumeric -dS $< > $@

%.bin: %.elf
	cargo-objcopy --release --example $(basename $@) -- -O binary -j .text -j .rodata $@
	#$(OBJCOPY) -O binary -j .text -j .rodata $< $@

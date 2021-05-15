Emulate a Realistic SoC environment
+++++++++++++++++++++++++++++++++++

Ok, now we have a RISC-V processor core implemented in synthesizable
SystemVerilog. But it is still just an RTL design that takes the following pins
to operate normally:

.. code-block:: verilog

  module core (
      input clock,
      input reset,
      input irq,
  
      // i-cache communication
      output [31:0] icache_addr,
      output icache_req,
      input [31:0] icache_data,
      input icache_rdy,
  
      // d-cache communication
      output [31:0] dcache_addr,
      output [31:0] dcache_wdata,
      output [1:0] dcache_ws,
      output dcache_req,
      output dcache_wr,
      input [31:0] dcache_rdata,
      input dcache_rdy
  );

There are many possible ways to use the implementation. You can interface with
some existing cache implementation for i-cache/d-cache and then design
some wrapping peripheral circuits to signal ``clock``, ``reset`` and ``irq``
pin (could be grounded if no external interrupts are needed) properly.  It is
also possible to make it "semi-hosted" by some commercial tool so that
cache/memory can be emulated on our host computer, while the core runs entirely on an
FPGA.  Here we use ``sim.cpp`` as an example, which emulates both the core
logic and peripherals on the computer, with the help of Verilator.  Verilator
can compile the SystemVerilog code into standalone C++ code with optimized,
parallelized emulation and thus gives a usable RISC-V computer emulated in
real-time. In this part, we'll quickly walk through the important ingredients
that make the processor run in a mini setup, ending up with a demo program
that implements a "Snake" (or "Blockade") video game that is playable with
inputs from the keyboard of the host computer.


No Longer a "Toy": Starting up a compiled program
-------------------------------------------------

From the pins required by ``core`` module, it is obvious that we need to
emulate the following in our C++ code:

- ``clock``: the clock signal for the processor;
- ``reset``: the reset pulse required upon startup of the processor (to
  initialize some of its internal registers);

- ``irq``: the Interrupt Request (IRQ) line to notify the processor of some
  external I/O; the actual type of interrupt events is usually adhoc to the
  system; a System-on-Chip (SoC) has its own definitions for inputs from its
  GPIOs, I2C/SPI or Serial/USB ports, whereas a desktop computer has this part implemented
  outside the CPU, on the motherboard, managed by its Basic Input/Output System
  (BIOS) chip (and BIOS also has the duty to initialize the processor when
  powered on);

- ``icache_*``: instruction cache, we can just emulate this together with
  ``dcache_*``;

- ``dcache_*``: data cache, we can simply emulate the entire memory that allows some
  configurable delay in response.

In ``sim.cpp``, we implement our entire SoC-like system in a ``SoC`` struct:

.. code-block:: cpp

    struct SoC {
        std::shared_ptr<Vcore> core;
        SimulatedRAM ram;
        // ...
    };

, where ``Vcore`` is the class generated from our ``core`` module by Verilator.

Upon an emulation time tick, we emulate the memory clock cycle if the cpu clock is on its
rising edge and also evaluate the core processor given its current pin levels:

.. code-block:: cpp

    void tick() {
        if (!core->clock)
        {
            main_time++;
            ram.eval_posedge();
        }
        core->eval();
    }

Each clock cycle of the processor is done by two emulation ticks, so we
alternate the ``core->clock`` pin between 0 and 1 when we advance the emulation
in the main emulation loop:

.. code-block:: cpp

    // ... (in struct Soc)
        void next_tick() {
            core->clock = !core->clock;
            tick();
        }
    // ... (in struct Soc)
    // emulation main loop
    while (!Verilated::gotFinish()) {
        soc.next_tick();
    // ...
    }

A hardware reset is needed before we start the main loop:

.. code-block:: cpp

    // ... (in struct Soc)
        void reset() {
            core->clock = 0;
            core->reset = 1;
            tick();

            core->clock = 1;
            tick();

            core->reset = 0;
        }
    // ... (in struct Soc)
    // reset before the main loop
    soc.reset();

This gives us a runnable processor. But it is not usable until we load a
program and let it start running from the entry point of the program.
Real-world processors (micro-controllers) define which address the program counter
should be at upon a hardware reset (aka. *reset vector*), and it is usually in
the specification of the chip product. For example, Intel uses ``0xfffffff0``
for all its x86 processors when they reset in the "real-mode" (the real-mode is
introduced for backward compatibility to the earliest
8086 chip, of later products). In ``core.sv``, our reset vector is at ``0x100000``:

.. code-block:: verilog

  `define PC_RESET        32'h00100000
  // ...
          if (ctrl_reset) begin
              `ifdef PIPELINE_DEBUG
                  $display("[%0t] reset pc", $time);
              `endif
              pc <= `PC_RESET; // initialize PC
              ctrl_nop_reg <= 1;
          end else if (!ctrl_stall) begin
  // ...

Therefore, we need to load our program into memory at ``0x100000``. The memory
is emulated by ``SimulatedRAM`` class and we provide the emulator a way to load
the content of a binary file to a specific memory location:

.. code-block:: verilog

    void load_image_from_file(FILE *img_file, size_t mem_off, size_t len = 0) {
        if (len == 0)
        {
            auto old = ftell(img_file);
            fseek(img_file, 0L, SEEK_END);
            len = ftell(img_file);
            fseek(img_file, old, SEEK_SET);
        }
        fread(&memory[0] + mem_off, 1, len, img_file);
    }

so that it is easy to load and run a program using the emulator:

::

    // run the hello world program
    ./sim -l apps/hello.bin=0x100000

So far we've figured out (and you should read the code in ``sim.cpp``) how to let a processor load and run a
program in its "raw" (machine code) form. Although one can write a program with
pure machine code and it works in theory, as human beings, we would appreciate
writing in a high-level language such as C/Rust, and compile the program down to
the binary code (called "firmware", or "image") to be more efficient both in
terms of our coding and the program's execution. Let's figure out how to get a
standard gcc compiler (the Rust example is under ``apps/mriscv-rs``, which shares the
linker script with the C apps) to do the job for you, which may also
help you develop more understanding in hardware/software interface.

There're mainly three keywords here: *toolchain*, *linkage* and *startup*. Assuming we're working
on a *host* computer with x86-64 CPU for this project, we need a compiler that
runs on 64-bit x86 ISA, but outputs programs that run on 32-bit RISC-V processor.
Compiling a program to its binary may need multiple tools (compiler,
linker, binutils) in such a *cross-compilation* procedure, and the set of necessary
tools is called a toolchain. In our ``apps/`` directory, you'll find the use of a
``RV32I`` toolchain in ``Makefile``.

::

    GCC_TOOLCHAIN = ./rv32i/
    CC = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-gcc
    OBJDUMP = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objdump
    OBJCOPY = $(GCC_TOOLCHAIN)/bin/riscv32-unknown-elf-objcopy

To compiler a C program into our targeted platform, we run the cross-compiler:

::

    // ... in apps/Makefile
	$(CC) -march=rv32i -mabi=ilp32 -O2 -nostartfiles -Wl,-Tlink.x -g -o $@ startup.s $< mriscv.o

You may have already noticed that there are some options/flags added to the command
line. This is because gcc by default compiles programs that run on an operating
system such as Linux, whose execution environment is largely different from
those bare-metal programs that run directly on the processor (yes, the
operating system can also be viewed as a bare-metal program). Typically, we
need to do the following for an embedded (bare-metal) environment:

- specify the correct architecture feature sets: ``rv32i``, because we don't support
  many other extensions defined by the RISC-V specification. For example,
  multiplications can be emulated by other arithmetic operations generated by
  the compiler, so we can still use multiplication in our C code.

- specify the correct ABI: ``ilp32``, which defines the integer model (such as
  the chosen integer for ``int``, ``unsigned``);

- remove start files and use custom startup code: use ``-nostartfiles`` to
  remove the "prologue" code the compiler generates (because it is for running
  on an OS), where the ``main()`` function is invoked, and add in
  our customized implementation to guide the execution flow to the main
  function correctly: (in ``startup.s``)

  .. code-block:: asm

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

  The global ``_start`` symbol is the entry point that the linker will
  use to determine where the program should really start from.
  We also add some NOPs to make it easier for debugging the processor as they
  clear the pipeline, and let the processor enter an infinite loop (``halt``) at
  the end of the program to guard the execution (so it won't continue into
  an invalid portion of memory). The startup code also sets up the stack pointer
  (``sp``) to the beginning of the stack (we use ``0x220000``, as our RAM
  is ``0x200000``-``0x21ffff``, inclusive).

- configure linkage correctly: because our program runs directly on the
  processor and accesses physical memory (rather than virtual memory) without
  an OS, we need to let the linker know how to finally arrange our binary code
  and how to utilize memory space. We first define the memory layout
  on our platform (``memory.x``):

  ::

    MEMORY
    {
        FLASH : ORIGIN = 0x00100000, LENGTH = 1M
        RAM : ORIGIN = 0x00200000, LENGTH = 32M
    }
    // ...
  
  and then the sections in the binary (``link.x``):

  ::

    INCLUDE memory.x
    SECTIONS
    {
        .  = ORIGIN(FLASH); /* load from the beginning of the flash */
        .text : {
            KEEP(*(.text))
            *(.text.startup)
        } > FLASH
        .rodata : {
            *(.rodata)
            *(.rodata.*)
        } > FLASH
        .data : {
            *(.data)
            *(.sdata)
            *(.sbss)
        } > RAM
        .note.gnu.build-id : {
            *(.note.gnu.build-id)
        } > FLASH
        /DISCARD/ : {*(*)}
    }

  Here we use ``FLASH`` to represent the space for storing the program code
  (which is usually in a flash storage, for real-world SoC chips). ``.text`` is
  the section for the code, followed by ``.rodata`` (read-only data) and ``.data``
  (read-write initialized data). With the linker script, the compiled program
  will start from our reset vector and utilize the available memory space
  correctly.

- (optional) link with some helper libraries: ``mriscv.c``
  implements some helper functions to make development easier (such as ``uprintf``).

The output file from the compiler/linker, though, is in ELF-format, which is
the common format for Linux binaries. The format should not be part of our
final image to be loaded on the processor. Thus, we need to extract its "pure"
content to have the image which only contains code and necessary data:

::

    // ... in apps/Makefile
    $(OBJCOPY) -O binary -j .text -j .rodata -j .data $< $@



Priting to the Console
----------------------

Timer, Event and a Basic Library
--------------------------------

A Snake Game
------------

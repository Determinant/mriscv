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
FPGA.  Here we use `sim.cpp`_ as an example, which emulates both the core
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

In `sim.cpp`_, we implement our entire SoC-like system in a ``SoC`` struct:

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

    // ... (in struct SoC)
        void next_tick() {
            core->clock = !core->clock;
            tick();
        }
    // ... (in struct SoC)
    // emulation main loop
    while (!Verilated::gotFinish()) {
        soc.next_tick();
    // ...
    }

A hardware reset is needed before we start the main loop:

.. code-block:: cpp

    // ... (in struct SoC)
        void reset() {
            core->clock = 0;
            core->reset = 1;
            tick();

            core->clock = 1;
            tick();

            core->reset = 0;
        }
    // ... (in struct SoC)
    // reset before the main loop
    soc.reset();

This gives us a runnable processor. But it is not usable until we load a
program and let it start running from the entry point of the program.
Real-world processors (micro-controllers) define which address the program counter
should be at upon a hardware reset (aka. *reset vector*), and it is usually in
the specification of the chip product. For example, Intel uses ``0xfffffff0``
for all its x86 processors when they reset in the "real-mode" (the real-mode is
introduced for backward compatibility to the earliest
8086 chip, of later products). In `core.sv`__, our reset vector is at ``0x100000``:

.. __: https://github.com/Determinant/mriscv/blob/main/core.sv

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

So far we've figured out (and you should read the code in `sim.cpp`_) how to let a processor load and run a
program in its "raw" (machine code) form. Although one can write a program with
pure machine code and it works in theory, as human beings, we would appreciate
writing in a high-level language such as C/Rust, and compile the program down to
the binary code (called "firmware", or "image") to be more efficient both in
terms of our coding and the program's execution. Let's figure out how to get a
standard gcc compiler (the Rust example is under `apps/mriscv-rs`__, which shares the
linker script with the C apps) to do the job for you, which may also
help you develop more understanding in hardware/software interface.

There're mainly three keywords here: *toolchain*, *linkage* and *startup*. Assuming we're working
on a *host* computer with x86-64 CPU for this project, we need a compiler that
runs on 64-bit x86 ISA, but outputs programs that run on 32-bit RISC-V processor.
Compiling a program to its binary may need multiple tools (compiler,
linker, binutils) in such a *cross-compilation* procedure, and the set of necessary
tools is called a toolchain. In our `apps/`__ directory, you'll find the use of a
``RV32I`` toolchain in ``Makefile``.

.. __: https://github.com/Determinant/mriscv/blob/main/apps/mriscv-rs/
.. __: https://github.com/Determinant/mriscv/blob/main/apps/

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

- (optional) link with some helper libraries: `mriscv.c`_
  implements some helper functions to make development easier (such as ``uprintf``).

The output file from the compiler/linker, though, is in ELF-format, which is
the common format for Linux binaries. The format should not be part of our
final image to be loaded on the processor. Thus, we need to extract its "pure"
content to have the image which only contains code and necessary data:

::

    // ... in apps/Makefile
    $(OBJCOPY) -O binary -j .text -j .rodata -j .data $< $@



Printing to the Console
-----------------------

So far, although our processor can run programs, it is somewhat "useless" because we cannot
even get the output from a program. Also for the purpose of debugging, we would
like something similar to ``printf`` in our C code (``print!`` in Rust) that can directly output
formatted string to the console. How does an SoC achieve this
realistically, as it doesn't have a built-in display/keyboard? The short answer is
*serial* communication (which is tied to the terms like "tty" and "console" for a PC, or an old mainframe server).

Modern micro-controllers support ways to both flash ("program") the program
into the chip's ROM, and establish console I/O from the ROM firmware while it is
running. A very pervasive interface is Universal Asynchronous
Receiver-Transmitter (UART). The basic hardware logic is to first serialize
data (a byte) using a shift register and then transmit bits via asynchronous
serial signals. The data received will be reassembled from the shift
register at the recipient back into a byte. On a micro-controller (such as those
common Arm Cortex chips manufactured by STMicroelectronics and RISC-V chips produced by
SiFive, according to the author's limited knowledge), the UART communication is
done by writing to or reading from some *memory-mapped* special "registers".
Unlike the general or CSR registers we use directly as part of the ISA instructions, these
registers can be more viewed as special (word-aligned) memory locations with which one can communicate
to the processor. Because these memory locations are special in their
semantics, one should use the ``volatile`` keyword when access them in a C program, to
avoid incorrect optimization by the compiler.

We use a custom, but very typical register interface for emulated UART (only
the transmitter, i.e., output) as below:

- the UART transmitter register address: ``0x1000``;
- when the processor writes to the register: the low 8-bit content of the
  written word is transmitted from the processor, and will be printed to the
  emulator's console;

- when the processor reads from the register: the value will be 0 only when the
  previous transmission is finished.

Therefore, we just need the following code to implement the emulator side:

.. code-block:: cpp

    // ...
    const uint32_t uart_txdata_addr = 0x00001000;
    // ... (in `eval_posedge()` of `SimulatedRAM`
    if (addr == uart_txdata_addr)
    {
        if (core->dcache_wr)
        {
            debug("dcache: write uart = %02x\n", addr, data);
            putchar((uint8_t)data);
        }
        else
        {
            core->dcache_rdata = 0;
            debug("dcache: read uart = %02x\n", core->dcache_rdata);
        }
    }
    // ...

, and then we can implement ``putchar`` and ``print`` functions as part of the
library (`mriscv.c`_) for our programs:

.. code-block:: cpp

    // ...
    static uint32_t * volatile const __attribute__ ((used)) UART_TXDATA = (uint32_t *)0x00001000;
    // ...
    int putchar(int c) {
        while ((*UART_TXDATA) >> 31) {}
        (*UART_TXDATA) = ((*UART_TXDATA) & ~0xff) | c;
        return c;
    }

    size_t print(const char *s) {
        size_t n = strlen(s);
        for (unsigned i = 0; i < n; i++)
            putchar(s[i]);
        return n;
    }


Without heap allocation, we can still implement a simplified version of
``sprintf`` --- ``uprintf`` in our library using a (stack-based) preallocated
array buffer:

.. code-block:: cpp

    // ... (see mriscv.c for the implementation of `int_to_string` and `itoa`)
    int uprintf(char *buff, const char *fmt, ...) {
        size_t nwrote = 0;
        va_list ap;
        va_start(ap, fmt);
        for (const char *p = fmt; *p != '\0'; p++) {
            if (*p == '%') {
                p++;
                switch (*p) {
                    case 's':
                        nwrote += print(va_arg(ap, const char *));
                        break;
                    case 'd':
                        itoa(va_arg(ap, int), buff);
                        nwrote += print(buff);
                        break;
                    case 'u':
                        int_to_string(va_arg(ap, unsigned), buff, false, false, 10);
                        nwrote += print(buff);
                        break;
                    case 'x':
                        int_to_string(va_arg(ap, unsigned), buff, false, false, 16);
                        nwrote += print(buff);
                        break;
                    case 'X':
                        int_to_string(va_arg(ap, unsigned), buff, false, true, 16);
                        nwrote += print(buff);
                        break;
                    case '%':
                        putchar('%');
                        nwrote++;
                        break;
                    default:
                        va_end(ap);
                        return -1;
                }
            } else {
                putchar(*p);
                nwrote++;
            }
        }
        va_end(ap);
        return nwrote;
    }

Then we can write our "hello world" program in C (`apps/hello.c`__):

.. __: https://github.com/Determinant/mriscv/blob/main/apps/hello.c

.. code-block:: cpp

    #include "mriscv.h"
    
    int main() {
        char buff[32];
        for (int i = -5; i < 5; i++)
            uprintf(buff, "Hello, world! %d\n", i);
        print(buff);
        return 0;
    }

which outputs formatted strings correctly:

::

    $ ./sim -l apps/hello.bin=0x100000
    Hello, world! -5
    Hello, world! -4
    Hello, world! -3
    Hello, world! -2
    Hello, world! -1
    Hello, world! 0
    Hello, world! 1
    Hello, world! 2
    Hello, world! 3
    Hello, world! 4

We next implement similar functions in Rust. Thanks to Rust's powerful macro
system, one can print formatted strings more ergonomically (a shrinked
version of `apps/mriscv-rs/examples/hello.rs`__):

.. __: https://github.com/Determinant/mriscv/blob/main/apps/mriscv-rs/examples/hello.rs

.. code-block:: rust

    #![no_std]
    #![no_main]
    
    extern crate mriscv;
    extern crate panic_halt;
    use core::fmt::Write;
    use mriscv::{uprint, uprintln};
    use riscv_rt::entry;
    
    #[entry]
    fn main() -> ! {
        uprintln!("hello, world! Count from 10:");
        for i in { 0..10 }.rev() {
            uprintln!("now it is {}...", i);
        }
        mriscv::wfi_loop();
    }

Timer, Keyboard, Video and a Useful Library
-------------------------------------------

We would like to introduce more useful library functions before we write
something more complex in Rust. In `apps/mriscv-rs/src/`__, there are
implementations for:

.. __: https://github.com/Determinant/mriscv/blob/main/apps/mriscv-rs/src

- a way to set the timer by writing to the memory-mapped CSR registers;
- ``uprint!`` and ``uprintln!`` macros;
- a heap-less, circular queue, ``Queue256``, based on the
  heapless ``Vec`` provided by ``heapless`` package;

- a safe, event-driven abstraction for handling timer, software and external
  interrupts, so an application does not need to define the global interrupt
  handlers and thus adopts a cleaner way (without ``unsafe``) to write
  interrupt-driven logic, implemented with the help of ``Queue256``;

- ``get_framebuffer()``  for video output.

We use the external interrupt (``irq``) to signal keyboard inputs are
available, while the actual key code is stored in a memory-mapped register at
``0x3000``. In the emulator, keyboard events are captured by SDL2 (you need to
build ``sim`` with ``env ENABLE_SDL=1 make`` to enable this and the video
support).

Finally, we use a VESA-framebuffer-like interface to memory-map the video
framebuffer into ``0x10000000``--``0x1004ffff`` (320K, RGB222, 640x480, 1 byte per pixel), which
is rendered to the GUI window on the emulator host.

The program can update the framebuffer to render some graphics.  An example is
to randomly draw colored pixels to the entire frame:

.. code-block:: rust

    #![no_std]
    #![no_main]
    
    extern crate panic_halt;
    use rand::Rng;
    use rand::SeedableRng;
    use riscv_rt::entry;
    
    const INTERVAL: u32 = 0x1000000;
    static mut COUNTER: usize = 0;
    static mut RNG: Option<rand::rngs::SmallRng> = None;
    
    #[export_name = "MachineTimer"]
    fn timer_handler(_trap_frame: &riscv_rt::TrapFrame) {
        let fb = mriscv::get_framebuffer();
        unsafe {
            mriscv::set_timer(INTERVAL);
            for i in 0..fb.len() {
                COUNTER += 1;
                fb[i] = RNG.as_mut().unwrap().gen();
            }
        }
    }
    
    #[entry]
    fn main() -> ! {
        unsafe {
            RNG = Some(rand::rngs::SmallRng::from_seed([0; 16]));
            riscv::register::mstatus::set_mie();
            mriscv::set_timer(INTERVAL);
        }
        mriscv::wfi_loop();
    }

The frame will be updated every ~16 million cycles (`apps/mriscv-rs/examples/video.rs`__):

.. __: https://github.com/Determinant/mriscv/blob/main/apps/mriscv-rs/examples/video.rs

.. image:: video.gif
   :align: center
   :width: 80%


A Snake Game
------------

With all these ingredients, to end our journey in this tutorial, we finally put
together a Rust game that is similar to "Snake" in
`apps/mriscv-rs/examples/snake.rs`__ with ~250 lines of code that is nice and clean.
It takes up/down/left/right inputs from the keyboard and renders the game play
every "second":

.. image:: https://raw.githubusercontent.com/Determinant/mriscv/main/apps/mriscv-rs/snake.gif
   :align: center
   :width: 90%

.. __: https://github.com/Determinant/mriscv/blob/main/apps/mriscv-rs/examples/snake.rs

.. _sim.cpp: https://github.com/Determinant/mriscv/blob/main/sim.cpp
.. _mriscv.c: https://github.com/Determinant/mriscv/blob/main/apps/mriscv.c

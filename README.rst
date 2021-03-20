mriscv: Minimalist 32-bit RISC-V core with C & Rust apps.
=========================================================

Features
--------

- Minimalist code: it uses a classic five-stage, in-order pipeline architecture to
  implement RV32I ISA and major part of CSR Mode-M, while keeping the entire
  core code base tiny. Written cleanly from scratch and good for pedagogical
  purpose.

- Practical approach: compared to other prevailing hobby/course projects, this
  project, despite its simplicity, still strives for a more "end-to-end" kind
  of approach.  The core RTL is implemented with synthesizable SystemVerilog
  and is able to run realistic C program directly compiled by gcc and Rust
  program built by cargo/rustc.

- It currently supports:

  - Modularized five-stage pipeline: one can clearly see five stages and their
    data paths/control signals
  - Realistic cache/memory model: the cache interface allows arbitrary cycles
    spent in instruction/data I/O by stalling the pipeline properly
  - RV32I instructions
  - Major CSR (machine mode only) registers
  - Exceptions (illegal instructions, address misalignment, etc.)
  - Interrupts (timer interrupt controlled by ``mtime``/``mtimecmp``, software
    interrupt triggered by writing to the memory-mapped register and external
    interrupt with an ``irq`` pin, which could be interfaced further or triggered
    directly in Verilator-based simulator UI by keyboard input.
  - Console output to the simulator through the memory-mapped register
  - Video framebuffer to the simulator
  - C/Rust application examples that work out of the box (``make -C ./apps``
    will automatically build the gcc toolchain once, and a standard RV32I Rust
    toolchain is required to build the Rust apps)


- TODO: a semi-hosted example on Spartan-6/Artix-7


It Currently Does NOT Have...
=============================

- Cache implementation or DDR RAM interfacing: this is more or less decoupled
  from the core implementation of the processor and it is also an interesting
  and deep topic by itself. I may want to work on a modularized cache
  implementation to integrate with the core in the future, but lacking this
  does not undermine the main purpose (and fun!) of this project.

- "M" extension: I left out multiplication/division for now as it is a very
  isolated extra feature and a good multiplier/divider may take substantial
  space in this tiny project. Both gcc and llvm are able to generate emulated
  code for multiplication/division, so this has lower priority.

- "A" extension: I plan to do some of the atomic operations (like ``amoswap``)
  as they may be useful in memory-mapped register manipulation.

- Supervisor/User mode: I don't have any plan for this because it could
  significantly complicate the existing CSR implementation, which goes against
  the main purpose of this project.


Requirements
------------

- Verilator (required)
- SDL2 (optional, for video and keyboard input as interrupts, enabled by ``ENABLE_SDL=1``)

Build
-----

- ``cd`` to repo root.
- Build RTL simulator: ``make`` (console-only) or ``env ENABLE_SDL=1 make sim`` (with video and keyboard)
- Build C apps: ``make -C ./apps``
- Build Rust apps:

  - install rustup: https://rustup.rs/
  - add rv32i toolchain: ``rustup target add riscv32i-unknown-none-elf``
  - build examples: ``make -C ./apps/mriscv-rs``

Examples
--------

- C Hello World: ``./sim -l apps/hello.bin=0x100000``
- C 8 Queens: ``./sim -l apps/queens.bin=0x100000``
- Rust Hello World: ``./sim -l apps/mriscv-rs/hello.bin=0x100000`` (build with
  ``ENABLE_SDL=1`` and use ``-v`` if want to test keyboard interrupts, press
  Esc to quit the UI)
- Rust video test: ``make clean; env ENABLE_SDL=1 make sim; ./sim -vl apps/mriscv-rs/video.bin=0x100000``
- Rust Snake game: ``./sim -vl apps/mriscv-rs/snake.bin=0x100000``

.. image:: https://raw.githubusercontent.com/Determinant/mriscv/main/apps/mriscv-rs/snake.gif
   :scale: 100%

Memory Map
----------

- ``0x00000000`` -- ``0x00000fff``: (reserved)
- ``0x00001000`` -- ``0x00001000``: UART_TXDATA
- ``0x00001001`` -- ``0x00001fff``: (reserved)
- ``0x00002000`` -- ``0x00002007``: MTIME
- ``0x00002008`` -- ``0x0000200f``: MTIMECMP
- ``0x00002010`` -- ``0x00002013``: MSIP (software interrupt, the lowest bit is used for msip, other 31 bits are hard-wired to 0)
- ``0x00002014`` -- ``0x00002017``: external interrupt control (write any non-zero value will clear last interrupt)
- ``0x00002018`` -- ``0x00002fff``: (reserved)
- ``0x00003000`` -- ``0x00003003``: input key states (the lowest 8 bits correspond
  to a USB scan code, the second 8 bits describe state: 1 for pressed/0 for
  released, other 16 bits are left for customized use)
- ``0x00003004`` -- ``0x000fffff``: (reserved)
- ``0x00100000`` -- ``0x001fffff``: program ("FLASH", 1MB)
- ``0x00200000`` -- ``0x021fffff``: RAM (32MB)
- ``0x02200000`` -- ``0x0fffffff``: (reserved)
- ``0x10000000`` -- ``0x1004ffff``: video framebuffer (320K, RGB222, 1bpp)

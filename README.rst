mriscv
======

Yet Another RISC-V toy processor?
---------------------------------

(WIP)

Requirements
------------

- Verilator

Build
-----

- ``cd`` to repo root.
- Build RTL simulator: ``make``
- Build C apps: ``make -C ./apps``
- Build Rust apps: ``make -C ./apps/mriscv-rs``

Example
-------

- C Hello World: ``./sim -l apps/hello.bin=0x100000``
- C 8 Queens: ``./sim -l apps/queens.bin=0x100000``
- Rust Hello World: ``./sim -l apps/mriscv-rs/hello.bin=0x100000``
- Rust video test: ``make clean; env ENABLE_SDL=1 make sim; ./sim -l apps/mriscv-rs/video.bin=0x100000 -v``

Memory Map
----------

- ``0x00000000`` -- ``0x00000fff``: (reserved)
- ``0x00001000`` -- ``0x00001000``: UART_TXDATA
- ``0x00001001`` -- ``0x00001fff``: (reserved)
- ``0x00002000`` -- ``0x00002007``: MTIME
- ``0x00002008`` -- ``0x0000200f``: MTIMECMP
- ``0x00002010`` -- ``0x000fffff``: (reserved)
- ``0x00100000`` -- ``0x001fffff``: PROGRAM ("FLASH", 1MB)
- ``0x00200000`` -- ``0x021fffff``: RAM (32MB)
- ``0x02200000`` -- ``0x0fffffff``: (reserved)
- ``0x10000000`` -- ``0x1004ffff``: video framebuffer (320K, RGB222, 1bpp)

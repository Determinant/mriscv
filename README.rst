Requirements
------------

- Verilator

Example
-------

- C Hello World: ``make && ./sim -l apps/hello.bin=0x100000``
- C 8 Queens: ``make && ./sim -l apps/queens.bin=0x100000``
- Rust Hello World: ``make && make -C ./apps/mriscv-rs && ./sim -l apps/mriscv-rs/hello.bin=0x100000``

Memory Map
----------

- ``0x00000000`` -- ``0x00000fff``: (reserved)
- ``0x00001000`` -- ``0x00001000``: UART_TXDATA
- ``0x00001001`` -- ``0x00001fff``: (reserved)
- ``0x000a0000`` -- ``0x000bffff``: VGA graphics
- ``0x000c0000`` -- ``0x000fffff``: (reserved)
- ``0x00100000`` -- ``0x001fffff``: PROGRAM ("FLASH")
- ``0x00200000`` -- ``0x021fffff``: RAM

Requirements
------------

- Verilator
- Cross-compilation toolchain that supports RV32/64: gcc-riscv64-linux-gnu (Ubuntu) / riscv64-linux-gnu-gcc (Arch Linux)

Example
-------

- ``make && ./sim -l apps/queens.bin=0 | grep '1fffec'``

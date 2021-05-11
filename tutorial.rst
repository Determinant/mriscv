Minimum RISC-V System From Scratch
==================================

So, we'll make a RISC-V system! It may sound "mission impossible" to build this
kind of thing from scratch in a relatively short time, without prior background
in hardware designs.  But it really isn't as magical as it seems. I hope this
tutorial could serve as a tiny but inspiring guide to those programmers who
know little about hardware like I did to have a better understanding of how a
computer system works from the ground up, and for those who know a bit about
different pieces of the story but can't put them together.

Nothing is better than having a playable example that is both small and
functional.  This repo already contains a RISC-V processor core implementation
that is synthesizable by itself, but also directly works with a minimal
emulator (Verilator-based) code with a realistic system setup. The processor
implements RV32I instruction set with major part of CSR Mode-M, while the
emulator emulates the cache/memory, serial console output, video output and
keyboard input. The applications are built with standard gcc/Rust RISC-V
toolchains and directly run on the processor.

The whole repo is simply divided into three parts:

- Processor core implementation in Verilog: ``core.sv`` (1.2K loc) and
  ``csr.sv`` (182 loc).

- System emulator: ``sim.cpp`` (423 loc).

- Example applications that directly runs on ``sim``: ``apps/*.c`` and
  ``apps/mriscv-rs/examples``.

This tutorial is organized in two parts: the implementation of the processor
core with Verilog and the building of the final system/applications.


Make a RISC-V processor
+++++++++++++++++++++++

What Makes a Processor?
-----------------------


Register Transfer Level Abstraction
-----------------------------------


Instruction Pipelining
----------------------


Decoder: Parsing an Instruction
-------------------------------


Executor: All About "Computing"
-------------------------------


Fetcher: Automation and Loop
----------------------------


Memory Access & Writeback
-------------------------


Resolving Inter-Stage Dependency
--------------------------------


When the Execution Gets Interrupted...
--------------------------------------

Emulate a Realistic SoC environment
+++++++++++++++++++++++++++++++++++

Starting up a program
---------------------

Memory-Mapped Registers
-----------------------

Priting to the Console
----------------------

Timer, Event and a Basic Library
--------------------------------

A Snake Game
------------

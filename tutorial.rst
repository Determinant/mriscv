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
  ``apps/mriscv-rs/examples/*.rs``.

This tutorial is organized in two parts: the implementation of the processor
core with Verilog and the building of the final system/applications.


Make a RISC-V processor
+++++++++++++++++++++++

What Makes a Processor?
-----------------------
A processor, like the name suggests, *processes* data via computation. More
specifically, the prevailing modern computer processors adopt a computation
model that is largely inspired by Turing Machine (as opposed to Lambda
Calculus, which gave birth to functional programming languages). A typical
processor mainly has three main functions for its operation:

1. The ability of taking input data from the containing system and sending
   results back to it. (I/O)
2. The ability of processing the data, such as doing arithmetic or logical
   operations. (Computation)
3. The ability of maintaining an internal state, which could affect its behavior
   in its processing. (State and Control)

Without any one of them, the construct sounds less "exciting": without
I/O, a processor makes no difference from a dummy blackbox that does nothing
inside; without computation, the processor can only output data as-is; without being
stateful, a processor is simply a mathematical expression whose outputs are
immediately determined by its inputs (which could be implemented solely by
*combinational logic* circuits, which will be discussed later) and the system
won't be stateful to control other things. Many mechanical systems are
stateful, such as your wrist watch, which keeps track of the current time as
the state and keep changing it as time goes. In fact, a mechanical watch/clock *is*
a computer/processor as it takes inputs (you can adjust the time and set the alarm),
generates outputs (you can always check the time from it), process the data (basic
arithmetic to maintain the current time) and has state and control (triggers
second/minute/hour hands according to its internal state) -- it is just not a
general-purpose computer by itself. However, interesting mechanical computers
do exist in history [link here].

Although the industry has been evolving its technology in the past decades, the
basic logic for a processor hasn't changed that much compared to its
theoretical model: like a Turning machine takes commands from a tape that
instructs its next operation, a CPU fetches the next *instruction* from a
(logically or physically) continuous portion of memory (or from the cache). It
also changes its internal state by the instruction like Turning machine can
modify its state register. However, when it comes to details, the processor
architectures may differ in how they manage and layout their internal/external
states. Here we choose RISC-V as our target *Instruction Set Architecture*
(ISA) for our processor build. RISC-V is *register-based*, meaning all
temporary values are kept on registers in a register file, like lockers in a
locker room, indexed by names (``x0``-``x31`` in RV32). They're directly
addressable, unlike *stack-based* alternatives which usually have to push to/pop from
a stack of values for their operations.
It is also a *register-to-registers* (*load/store*) architecture, where all
operations are done on the basis of registers, so that values have to be loaded
from the memory to registers before a computation and stored back to the
memory explicitly from registers after the computation, unlike other CISC architectures like x86
which supports mixed use of values from memory and registers (a *register-memory*
architecture).

With these basics in mind, obviously, we need to have different parts of the
processor to take care of the three major functions. There should be a way to
*decode* the instruction into some form of internal, temporary representation of
its functionality, so as to *control* how the rest part of computation should be
carried out. There should be some logic for doing the actual calculation, some
organized internal "lockers" to hold the register values and some logic to write
the results to the memory. Finally, there should be a loop-like logic to drive
the entire composition of different parts, so the processor can move onto the next
instruction and keep its execution, like a machine gun.

Register Transfer Level Abstraction
-----------------------------------
It is not very hard to notice such a power construct could be implemented by
repeatedly applying two kinds of "logic":

- the logic that is like a math expression, which calculates an "immediate"
  output (response) from the given input (signal) by pure, stateless logic
  operation,

- the logic that "remembers" something, controlled by some external signal, which
  could be later altered or read out. (Like a "sequencer/synthesizer", if you're familiar
  with electronic music.)

One gives us some math calculation, while the other one introduces the notion
of states.


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

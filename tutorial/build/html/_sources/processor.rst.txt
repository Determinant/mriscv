Make a RISC-V processor
+++++++++++++++++++++++

What Makes a Processor?
-----------------------
A processor, like the name suggests, *processes* data via computation. More
specifically, the prevailing modern computer processors adopt a computation
model that is largely inspired by Turing Machine (as opposed to Lambda
Calculus, which gave birth to functional programming languages). A typical
processor has three main functions for its operation:

1. The ability of taking input data from the containing system and putting
   results back to it. (I/O)
2. The ability of processing the data, such as doing arithmetic or logical
   operations. (Computation)
3. The ability of maintaining an internal state, which could affect its behavior
   during processing. (State and Control)

Without any one of them, the construct sounds less "exciting": without
I/O, a processor makes no difference from a dummy blackbox that does nothing
inside; without computation, the processor can only output data as-is; without being
stateful, a processor is a simple calculator whose outputs are
immediately determined by its inputs (which could be implemented solely by
*combinational logic* circuits, which will be discussed soon) and the system
won't be stateful to control other things. Many mechanical systems are
stateful, such as your wrist watch, which keeps track of the current time as
the state and changes it as time ticks. In fact, a mechanical watch/clock *is*
a computer/processor as it takes inputs (you can adjust the time and set the alarm),
generates outputs (you can always check the time from the display), process the data (basic
arithmetic to maintain the current time) and has state and control (triggers
second/minute/hour hands according to its internal state) -- it is just not a
*general-purpose* computer. However, interesting mechanical computers
do exist in history [link here].

Although the industry has been evolving its technology in the past decades, the
basic logic for a processor hasn't been much different from its
theoretical model: like a Turning machine takes commands from a tape that
instructs its next operation, a CPU fetches the next *instruction* from a
(logically or physically) continuous portion of memory (or from the cache). It
also changes its internal state by the instruction like the Turning machine can
modify its state register. However, when it comes to details, the processor
architectures may differ in how they manage/layout their internal/external
states or how they interpret the instructions. Here we choose RISC-V as our target *Instruction Set Architecture*
(ISA) for our processor build. RISC-V is *register-based*, meaning all
temporary values are kept by registers in a *register file*, like lockers in a
locker room, individually indexed by names (``x0``-``x31`` in RV32). They're directly
accessible, unlike *stack-based* alternatives which usually have to push to/pop from
a stack of values by their operations.
It is also a *register-to-register* (aka. *load/store*) architecture, where all
operations are done on the basis of registers, so that values have to be loaded
from the memory to registers before a computation and stored back to the
memory explicitly from registers afterwards, unlike many CISC architectures such as x86
which supports mixed use of values both from memory and registers (a *register-memory*
architecture).

With these basics in mind, obviously, we need to have different parts of the
processor to take care of the three major functions. There should be a way to
*decode* the instruction into some form of internal, temporary representation of
its functionality, so as to *control* how the rest part of computation should be
carried out. There should be some logic for doing the actual calculation, some
organized internal "lockers" to hold the values and some logic to read/write
the results from/to the main memory. Finally, there should be a loop-like logic to drive
the entire composition of different parts, so the processor can move onto the next
instruction and keep executing instructions one after another with full
automation, like a machine gun.

Register Transfer Level Abstraction
-----------------------------------
It is not very hard to notice such a powerful construct could be implemented by
repeatedly applying two kinds of "logic":

- the logic that is like a math expression, which calculates an "immediate"
  output (response) from the given input (signal) by pure, stateless logic
  operation,

- the logic that "remembers" something, controlled by some external signal, which
  could be later altered or read out. (Like a "sequencer/synthesizer", if you're familiar
  with electronic music.)

In short, one gives us some math calculation, called *combinational logic*, while the
other one introduces the notion of states, called *sequential logic*.

Indeed, in digital circuit design, there is a widely used abstraction that
is based on this observation. Register Transfer Level (RTL) is used by languages
like Verilog/SystemVerilog and VHDL to create a high-level schematics of logic circuits. It
describes the logical behavior with these two kinds of logic as primitives, without having
to dive too much into their low-level (gate-level) implementation.
Each kind of logic usually has patterns and disciplines for its implementation
and can be either automatically synthesized by specialized tools or hand-crafted if
necessary (or both), while the RTL language can abstract this away so the
design task can be divided into high-level logic design and low-level
implementation (e.g. the use of basic gates and wiring/routing).

Combinational logic is usually implemented directly with
the wiring of a cascade of basic logic gates (e.g. NAND/NOR gates). For a concrete
example, a 2-input *multiplexer* (or simply "mux") can be implemented as in the diagram:

.. image:: mux.svg
   :align: center
   :width: 60%

Each component is a *NAND* gate where the output is the negation of a logical
"and" of two inputs.  With the shown wiring of four gates, the mux choses between inputs (I\ :sub:`0` vs. I\
:sub:`1`) for the output switched by the control signal A. The value of I\
:sub:`0` will be chosen (Q = I\ :sub:`0`) iff. A = 1.

As a comparison, in SystemVerilog (a popular RTL language), the mux can be implemented as:

.. code-block:: verilog

  // 2-input mux
  module mux2(input i0, input i1, input a, output q);
      // Combinational logic is a direct assignment to
      // the wire from an expression.
      // Since it is time-independent, the order of
      // assignments does not matter (unlike many
      // programming languages).
      assign q = a ? i0 : i1;
  endmodule

There is one thing that's worth noting: the "calculation" here happens
almost instantaneously as the underlying logic gates "maintain" their outputs from
their inputs by physics (the use of semi-conductors). There is, however, still some
time delay due to physical properties of the gates and the time for electrons
to propagate on the wire or within the semi-conductors, at the scale of
nano-seconds. Thus, the delay is largely affected by the depth gates wiring and
complexity of the overall construction.

Sequential logic, however, is very different. Here we only discuss about
*synchronous* sequential logic. As the main building block for such a logic, a *flip-flop* not
only takes input as in combinational logic, but also requires a *clock* signal
that drives it. In sequential logic, outputs are only stabilized and deemed as
valid when the clock signal pulses ("rising edge", going to 0 to 1; or "falling edge", 1 to 0). The use of an
additional clock signal effectively introduces the notion of time into the
logic (unlike combinational logic, which is time-independent). The notion of
discrete time also makes the changing state easy to reason about and
manipulate.  Interestingly, such seemingly "magical" building blocks can be
still implemented by pure wiring of gates, to be stateful. The extra clock signal (or
"reset signal", for "latches", its asynchronous counterpart) is the key
ingredient that does the trick. The below diagram shows a wiring scheme for
"D-type" flip-flop with NAND gates, which is a commonly used component in
synthesizing sequential logic. In this flip-flop, the output (Q) will retain
the "memorized" value, when the clock signal (Clk) is 0, and change to the
input (D) when the clock is 1.

.. image:: d-flip-flop.svg
   :align: center
   :width: 60%

Sequential logic in SystemVerilog below may be synthesized/hand-crafted by a D-type
flip flop.

.. code-block:: verilog

  module flip_flop(input d, input clk, output reg q);
      // Sequential logic has notion of time.
      // It can only be specified with in an `always*` block.
      always_ff @ (posedge clk) begin
          // Change the output only at
          // the positive clock edge (clk = 1).
          q <= d;
      end
  endmodule

Finally, consider the scenario where we combine both kinds of logic together: we
wire the input of a combinational logic from the output of a D-type flip-flop,
and then wire the combinational output to the flip-flop input. It creates a
"loop" which takes the output from the current state and puts the new value to
the next state after calculation, implementing an iterator whose iterations are
driven by the clock signal.

.. code-block:: verilog

  module counter(input d, input rst, input clk, output reg q);
      // A counter, whose value could be set to `d` when
      // `rst` = 1 on the positive edge of `clk`, or increased
      // by 1 when `rst` = 0 and clocked.

      // combinational logic, a 2-input mux.
      wire comb_result = rst ? d : (q + 1);
      // sequential logic, to alter the state ("reg" for the register, wired to `q`)
      always_ff @ (posedge clk) begin
          q <= comb_result;
      end
  endmodule

Of course, the period of the clock signal (more
precisely, the minimum gap between two clock cycles) should be conservatively
chosen to be larger than the circuit time of the combinational logic, so the
input to the flip-flop is stabilized before the next clock ticks. This also reveals
why processors nowadays are "pipelined", the topic of the next section.


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


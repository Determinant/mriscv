.PHONY: all

verilator_base=/usr/share/verilator

all: sim

cpu_cc: cpu.v
	verilator cpu.v --Mdir $@ --cc --debug
cpu_cc/Vcpu__ALL.a: cpu_cc
	make -C cpu_cc -f Vcpu.mk
sim: sim.cpp cpu_cc/Vcpu__ALL.a
	$(CXX) -o $@ $< -I cpu_cc -I $(verilator_base)/include/ cpu_cc/Vcpu__ALL.a $(verilator_base)/include/verilated.cpp

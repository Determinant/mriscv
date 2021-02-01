.PHONY: all clean

verilator_base=/usr/share/verilator

all: sim

clean:
	rm -r cpu_cc
	rm sim

cpu_cc: cpu.sv pipeline.sv
	verilator cpu.sv --Mdir $@ --cc -Wall -Wno-style +define+DEBUG
cpu_cc/Vcpu__ALL.a: cpu_cc cpu_cc/Vcpu.cpp
	make -C cpu_cc -f Vcpu.mk
sim: sim.cpp cpu_cc/Vcpu__ALL.a tests/add.bin
	$(CXX) -o $@ $< -I cpu_cc -I $(verilator_base)/include/ cpu_cc/Vcpu__ALL.a $(verilator_base)/include/verilated.cpp -g -O2

tests/add.bin:
	$(MAKE) -C tests

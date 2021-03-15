.PHONY: all clean

verilator_base=/usr/share/verilator

all: sim sim_debug apps

clean:
	rm -r cpu_cc cpu_cc_debug
	rm sim sim_debug

cpu_cc: cpu.sv pipeline.sv csr.sv
	verilator cpu.sv --Mdir $@ --cc -Wall -Wno-style
cpu_cc/Vcpu__ALL.a: cpu_cc cpu_cc/Vcpu.cpp
	make -C cpu_cc -f Vcpu.mk

cpu_cc_debug: cpu.sv pipeline.sv csr.sv
	verilator cpu.sv --Mdir $@ --cc -Wall -Wno-style +define+DEBUG
cpu_cc_debug/Vcpu__ALL.a: cpu_cc_debug cpu_cc/Vcpu.cpp
	make -C cpu_cc_debug -f Vcpu.mk

sim: sim.cpp cpu_cc/Vcpu__ALL.a
	$(CXX) -o $@ $< -I cpu_cc -I $(verilator_base)/include/ cpu_cc/Vcpu__ALL.a $(verilator_base)/include/verilated.cpp -g -O2

sim_debug: sim.cpp cpu_cc_debug/Vcpu__ALL.a
	$(CXX) -o $@ $< -I cpu_cc_debug -I $(verilator_base)/include/ cpu_cc_debug/Vcpu__ALL.a $(verilator_base)/include/verilated.cpp -g -O2 -DDEBUG

apps:
	$(MAKE) -C apps

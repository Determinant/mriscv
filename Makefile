.PHONY: all clean

verilator_base=/usr/share/verilator
LD_FLAGS=

ifeq ($(ENABLE_SDL), 1)
	CXX_FLAGS += -DENABLE_SDL
	LD_FLAGS += -lSDL2
endif

all: sim sim_debug apps

clean:
	rm -rf core_cc core_cc_debug
	rm -f sim sim_debug

core_cc: core.sv core.sv csr.sv
	verilator core.sv --Mdir $@ --cc -Wall -Wno-style
core_cc/Vcore__ALL.a: core_cc core_cc/Vcore.cpp
	make -C core_cc -f Vcore.mk

core_cc_debug: core.sv core.sv csr.sv
	verilator core.sv --Mdir $@ --cc -Wall -Wno-style +define+DEBUG
core_cc_debug/Vcore__ALL.a: core_cc_debug core_cc/Vcore.cpp
	make -C core_cc_debug -f Vcore.mk

sim: sim.cpp core_cc/Vcore__ALL.a
	$(CXX) -o $@ $< -I core_cc -I $(verilator_base)/include/ core_cc/Vcore__ALL.a $(verilator_base)/include/verilated.cpp -g -O2 $(LD_FLAGS) $(CXX_FLAGS)

sim_debug: sim.cpp core_cc_debug/Vcore__ALL.a
	$(CXX) -o $@ $< -I core_cc_debug -I $(verilator_base)/include/ core_cc_debug/Vcore__ALL.a $(verilator_base)/include/verilated.cpp -g -O2 -DDEBUG $(LD_FLAGS) $(CXX_FLAGS)

apps:
	$(MAKE) -C apps

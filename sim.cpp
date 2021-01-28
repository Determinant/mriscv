#include <memory>
#include <vector>
#include "verilated.h"
#include "Vcpu.h"

uint64_t main_time = 0;

double sc_time_stamp() {
    return main_time;
}

void reset(std::shared_ptr<Vcpu> cpu) {
    cpu->reset = 1;
    cpu->clock = 0;
    cpu->eval();
    cpu->clock = 1;
    cpu->eval();
    cpu->clock = 0;
    cpu->reset = 0;
    cpu->eval();
}

class SimulatedRAM {
    std::shared_ptr<Vcpu> cpu;
    std::vector<uint8_t> memory;
    size_t capacity;
    uint64_t icache_next_rdy;
    uint64_t dcache_next_rdy;
    int icache_state; // 0 -> reset; 1 -> reading
    int dcache_state; // 0 -> reset; 1 -> reading/writing

    SimulatedRAM(std::shared_ptr<Vcpu> cpu, size_t capacity): \
            cpu(cpu), capacity(capacity),
            icache_next_rdy(0),
            dcache_next_rdy(0) {
        memory.resize(capacity);
        cpu->icache_rdy = 0;
        cpu->dcache_rdy = 0;
    }

    void tick() {
        if (cpu->reset == 1)
        {
            icache_state = 0;
            dcache_state = 0;
            cpu->icache_rdy = 0;
            cpu->dcache_rdy = 0;
            return;
        }
        if (icache_state == 0)
        {
            cpu->icache_rdy = 0;
            if (cpu->icache_req)
                icache_state = 1;
        }
        if (icache_state == 1)
        {
            if (icache_next_rdy == 0)
            {
                assert(cpu->icache_addr + 4 < capacity);
                cpu->icache_data = *(uint32_t *)(&memory[0] + cpu->icache_addr);
                cpu->icache_rdy = 1;
            } else icache_next_rdy--;
        }

        if (dcache_state == 0)
        {
            cpu->dcache_rdy = 0;
            if (cpu->dcache_req)
                dcache_state = 1;
        }
        if (dcache_state == 1)
        {
            if (dcache_next_rdy == 0)
            {
                assert(cpu->dcache_addr + 4 < capacity);
                auto word = (uint32_t *)(&memory[0] + cpu->dcache_addr);
                if (cpu->dcache_wr)
                    *word = cpu->dcache_wdata;
                else
                    cpu->dcache_rdata = *word;
                cpu->dcache_rdy = 1;
            } else dcache_next_rdy--;
        }
    }

    void schedule_next_icache_rdy(uint64_t nstep) {
        cpu->icache_rdy = nstep;
    }

    void schedule_next_dcache_rdy(uint64_t nstep) {
        cpu->dcache_rdy = nstep;
    }
};

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto cpu = std::make_shared<Vcpu>();
    reset(cpu);
    while (!Verilated::gotFinish()) {
        cpu->clock = !cpu->clock;
        if (cpu->clock)
            main_time++;
        cpu->eval();
        if (main_time > 100)
        {
            break;
        }
    }
    cpu->final();               // Done simulating
}

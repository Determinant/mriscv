#include <cstdio>
#include <memory>
#include <vector>
#include <cstdlib>
#include <cassert>
#include <exception>
#include <getopt.h>
#include "verilated.h"
#include "Vcpu.h"

#ifdef DEBUG
#define debug(f_, ...) printf((f_), ##__VA_ARGS__)
#else
#define debug(...) do {} while(0)
#endif

uint64_t main_time = 0;
uint32_t halt_addr = 0x00000000; //0x0010001c;

const uint32_t uart_txdata = 0x00001000;

double sc_time_stamp() {
    return main_time;
}

class SimulatedRAM {
    std::shared_ptr<Vcpu> cpu;
    std::vector<uint8_t> memory;
    size_t capacity;
    uint64_t icache_next_rdy;
    uint64_t dcache_next_rdy;
    int icache_state; // 0 -> reset; 1 -> reading
    int dcache_state; // 0 -> reset; 1 -> reading/writing

    public:
    SimulatedRAM(std::shared_ptr<Vcpu> cpu, size_t capacity): \
            cpu(cpu), capacity(capacity),
            icache_next_rdy(0),
            dcache_next_rdy(0) {
        memory.resize(capacity);
        cpu->icache_rdy = 0;
        cpu->dcache_rdy = 0;
    }

    void load_image_from_file(FILE *img_file, size_t mem_off, size_t len = 0) {
        if (len == 0)
        {
            auto old = ftell(img_file);
            fseek(img_file, 0L, SEEK_END);
            len = ftell(img_file);
            fseek(img_file, old, SEEK_SET);
        }
        fread(&memory[0] + mem_off, 1, len, img_file);
    }

    void eval_posedge() {
        if (cpu->reset == 1)
        {
            debug("reset ram\n");
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
                cpu->icache_data = \
                    memory[cpu->icache_addr] |
                    (memory[cpu->icache_addr + 1] << 8) |
                    (memory[cpu->icache_addr + 2] << 16) |
                    (memory[cpu->icache_addr + 3] << 24);
                cpu->icache_rdy = 1;
                icache_state = 0;
                debug("icache: read byte @ %08x = %08x\n", cpu->icache_addr, cpu->icache_data);
                //schedule_next_icache_rdy(4);
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
                if (cpu->dcache_wr)
                {
                    auto addr = cpu->dcache_addr;
                    auto data = cpu->dcache_wdata;

                    if (addr == uart_txdata)
                    {
                        debug("dcache: write uart = %02x\n", addr, data);
                        putchar((uint8_t)data);
                    }
                    else
                    {
                        if (cpu->dcache_ws == 0)
                        {
                            debug("dcache: write byte @ %08x = %02x\n", addr, data);
                            memory[addr] = data & 0xff;
                        }
                        else if (cpu->dcache_ws == 1)
                        {
                            debug("dcache: write halfword @ %08x = %04x\n", addr, data);
                            memory[addr] = data & 0xff;
                            memory[addr + 1] = (data >> 8) & 0xff;
                        }
                        else if (cpu->dcache_ws == 2)
                        {
                            debug("dcache: write word @ %08x = %08x\n", addr, data);
                            memory[addr] = data & 0xff;
                            memory[addr + 1] = (data >> 8) & 0xff;
                            memory[addr + 2] = (data >> 16) & 0xff;
                            memory[addr + 3] = (data >> 24) & 0xff;
                        }
                        else assert(0);
                    }
                }
                else
                {
                    if (cpu->dcache_addr == uart_txdata)
                    {
                        cpu->dcache_rdata = 0;
                    }
                    else
                        cpu->dcache_rdata = *(uint32_t *)(&memory[0] + cpu->dcache_addr);
                    debug("dcache: read word @ %08x = %08x\n", cpu->dcache_addr, cpu->dcache_rdata);
                }
                cpu->dcache_rdy = 1;
                dcache_state = 0;
                //schedule_next_dcache_rdy(1);
            } else {
                debug("delayed dcache response: %lu\n", dcache_next_rdy);
                dcache_next_rdy--;
            }
        }
    }

    void schedule_next_icache_rdy(uint64_t nstep) {
        icache_next_rdy = nstep;
    }

    void schedule_next_dcache_rdy(uint64_t nstep) {
        dcache_next_rdy = nstep;
    }
};

struct SoC {
    std::shared_ptr<Vcpu> cpu;
    SimulatedRAM ram;

    SoC(std::shared_ptr<Vcpu> cpu, size_t mem_cap): cpu(cpu), ram(cpu, mem_cap) {}

    void reset() {
        cpu->clock = 0;
        cpu->reset = 1;
        tick();

        cpu->clock = 1;
        tick();

        cpu->reset = 0;
    }

    void tick() {
        if (!cpu->clock)
        {
            main_time++;
            ram.eval_posedge();
        }
        cpu->eval();
    }

    void next_tick() {
        cpu->clock = !cpu->clock;
        tick();
    }

    void halt() {
        cpu->final();               // Done simulating
    }
};

static struct option long_options[] = {
    {"load-image", required_argument, 0, 'l'},
    {"halt-addr", required_argument, 0, 'e'},
};

void die(const char *s) {
    fprintf(stderr, "error: %s\n", s);
    exit(1);
}

int main(int argc, char** argv) {
    int optidx = 0;
    auto soc = SoC(std::make_shared<Vcpu>(), 32 << 20);
    for (;;)
    {
        int c = getopt_long(argc, argv, "l:e:", long_options, &optidx);
        if (c == -1) break;
        switch (c)
        {
            case 'l':
                {
                    std::string arg{optarg};
                    auto pos = arg.find("=");
                    if (pos == std::string::npos)
                        die("invalid image spec, should be in the form of `<filename>=<hex location>`");
                    FILE *img = fopen(arg.substr(0, pos).c_str(), "r");
                    if (img)
                    {
                        size_t t;
                        try {
                            auto loc = std::stoul(arg.substr(pos + 1), &t, 16);
                            soc.ram.load_image_from_file(img, loc);
                        } catch (...) {
                            die("invalid image location");
                        }
                        fclose(img);
                    }
                    break;
                }
            case 'e':
                {
                    size_t t;
                    try {
                        halt_addr = std::stoul(optarg, &t, 16);
                    } catch (...) {
                        die("invalid addr");
                    }
                }
        }
    }
    Verilated::commandArgs(argc, argv);
    soc.reset();
    debug("reset\n");
    while (!Verilated::gotFinish()) {
        soc.next_tick();
        debug("===\n");
        if (soc.cpu->_debug_pc == halt_addr)
        {
            soc.halt();
            printf("halted the processor at 0x%x\n", halt_addr);
            break;
        }
    }
}

#include <iostream>
#include "verilated.h"
#include "Vcpu.h"

uint64_t main_time = 0;

double sc_time_stamp () {
    return main_time;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto top = new Vcpu();
    while (!Verilated::gotFinish()) {
        top->clock = !top->clock;
        if (top->clock)
            main_time++;
        top->eval();
        if (main_time > 100)
        {
            break;
        }
    }
    top->final();               // Done simulating
    delete top;
}

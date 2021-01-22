#include <iostream>
#include "verilated.h"
#include "Vcpu.h"

uint64_t main_time = 0;

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto top = new Vcpu();
    while (!Verilated::gotFinish()) {
        top->clock = !top->clock;
        top->eval();            // Evaluate model
        //std::cout << top->out << endl;       // Read a output
        main_time++;            // Time passes...
        if (main_time > 100)
        {
            break;
        }
    }
    top->final();               // Done simulating
    delete top;
}

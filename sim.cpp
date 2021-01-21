#include <iostream>
#include "verilated.h"
#include "Vcpu.h"

uint64_t main_time = 0;

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto top = new Vcpu();
    top->x = 1;
    top->y = 2;
    while (!Verilated::gotFinish()) {
        top->eval();            // Evaluate model
        //std::cout << top->out << endl;       // Read a output
        main_time++;            // Time passes...
        if (main_time > 10)
        {
            printf("%d\n", top->z);
            break;
        }
    }
    top->final();               // Done simulating
    delete top;
}

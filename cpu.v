`include "pipeline.v"
module cpu(
    input clock,
    input reset,
    input halt,
    input irq,

    output [31:0] inst_addr,
    input [31:0] inst_data,

    output [31:0] data_addr,
    input [31:0] data_in,
    output [31:0] data_out
);
    pipeline pl (
        clock,
        reset,
        halt,
        irq,
        inst_addr,
        inst_data,
        data_addr,
        data_in,
        data_out
    );
endmodule

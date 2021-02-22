`include "pipeline.sv"
module cpu(
    input clock,
    input reset,
    output [31:0] _debug_pc_addr,

    output [31:0] icache_addr,
    output icache_req,
    input [31:0] icache_data,
    input icache_rdy,

    output [31:0] dcache_addr,
    output [31:0] dcache_wdata,
    output [1:0] dcache_ws,
    output dcache_req,
    output dcache_wr,
    input [31:0] dcache_rdata,
    input dcache_rdy
);
    pipeline pl (
        clock,
        reset,
        _debug_pc_addr,
        icache_addr,
        icache_req,
        icache_data,
        icache_rdy,
        dcache_addr,
        dcache_wdata,
        dcache_ws,
        dcache_req,
        dcache_wr,
        dcache_rdata,
        dcache_rdy
    );
endmodule
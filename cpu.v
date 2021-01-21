`include "pipeline.v"
module test(
    input [7:0] x,
    input [7:0] y,
    output [7:0] z);
    initial $display("Hello World!");
    plus plus1 (x, y, z);
endmodule

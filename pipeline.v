// opcode

`define LUI     7'b0110111
`define AUIPC   7'b0010111
`define JAL     7'b1101111
`define JALR    7'b1100111
`define BXX     7'b1100011 // BEQ, BNE, BLT, BGE, BLTU, BGEU
`define LX      7'b0000011 // LB, LH, LW, LBU, LHU
`define SX      7'b0100011 // SB, SH, SW
`define XXXI    7'b0010011 // ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
`define XXX     7'b0110011 // ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND

// the following is not supported yet
`define FEN     7'b0001111 // FENCE
`define EXX     7'b1110011 // ECALL, EBREAK

// funct3 code

`define FADD   3'b000
`define FSLT   3'b010
`define FSLTU  3'b011
`define FXOR   3'b100
`define FOR    3'b110
`define FAND   3'b111
`define FSLL   3'b001
`define FSRX   3'b101 // FSRLI & SRAI

`define FBEQ    3'b000
`define FBNE    3'b001
`define FBLT    3'b100
`define FBGE    3'b101
`define FBLTU   3'b110
`define FBGEU   3'b111

`define FB      3'b000
`define FH      3'b001
`define FW      3'b010
`define FBU     3'b100
`define FHU     3'b101

module register_file(
    input [4:0] reg1_raddr,
    input [4:0] reg2_raddr,
    output [31:0] reg1_rdata,
    output [31:0] reg2_rdata,

    input [4:0] reg_waddr,
    input [31:0] reg_wdata,
    input reg_wen, //< write enable

    input ctrl_clk //< clock
);
    // register file
    reg [31:0] regs [31:0];
    assign reg1_rdata = regs[reg1_raddr];
    assign reg2_rdata = regs[reg2_raddr];
    assign regs[0] = 0;
    always_ff @ (posedge ctrl_clk) begin
        if (reg_wen && reg_waddr != 0)
        begin
            `ifndef SYNTHESIS
                $display(
                    "[%0t] [wb] write %h to register %d",
                    $time, reg_wdata, reg_waddr
                );
            `endif
            regs[reg_waddr] <= reg_wdata;
        end
        else
        begin
            `ifndef SYNTHESIS
                $display("[%0t] [wb] no write to register files", $time);
            `endif
        end
    end
endmodule

module fetcher(
    input [31:0] pc_jump_target,

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input ctrl_is_jump,

    // i-cache communication
    output [31:0] icache_addr,
    output icache_req,
    input [31:0] icache_data,
    input icache_rdy,

    output reg [31:0] inst_reg,
    output reg [31:0] pc_reg,
    output reg ctrl_is_nop_reg,
    output ctrl_fetcher_stall
);
    reg [31:0] pc;
    assign icache_addr = pc;
    assign icache_req = 1;
    assign ctrl_fetcher_stall = !icache_rdy;
    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            pc <= 32'h00000000; // initialize PC to 0x00000000
            ctrl_is_nop_reg <= 1;
        end
        else
            if (!ctrl_stall) begin
                `ifndef SYNTHESIS
                    $display(
                        "[%0t] [fetch] stage works on pc_jump_target=%h pc=%h",
                        $time, pc_jump_target, pc
                    );
                `endif
                inst_reg <= icache_data;
                pc_reg <= pc;
                pc <= ctrl_is_jump ? pc_jump_target : pc + 4;
                ctrl_is_nop_reg <= 0;
            end
            else begin
                `ifndef SYNTHESIS
                    $display("[%0t] [fetch] stage idle rdy=%b", $time, icache_rdy);
                `endif
                if (!ctrl_next_stage_stall)// insert a bubble
                    ctrl_is_nop_reg <= 1;
            end
    end
endmodule;

module decoder(
    input [31:0] inst,
    input [31:0] pc,
    input [31:0] reg1_rdata,
    input [31:0] reg2_rdata,
    output [4:0] reg1_raddr,
    output [4:0] reg2_raddr,

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input ctrl_is_nop,

    output reg [31:0] op1_reg,
    output reg [31:0] op2_reg,
    output reg [4:0] rd_reg,
    output reg [31:0] src_reg,
    output reg [2:0] ctrl_alu_func_reg,
    output reg ctrl_alu_sign_ext_reg,
    output reg ctrl_is_nop_reg,
    output reg ctrl_wb_reg,
    output reg [4:0] ctrl_mem_reg,
    output [31:0] ctrl_pc_jump_target,
    output ctrl_is_jump,
    output ctrl_decoder_stall
);
    assign ctrl_decoder_stall = 0;
    // Layout: [  7 bits          ][5 bits][5 bits][3 bits][ 5 bits         ][ 7 bits ]
    // R-type: [funct7            ][rs2   ][rs1   ][funct3][rd              ][opcode  ]
    //
    // I-type: [    imm[11:0]             ][rs1   ][funct3][rd              ][opcode  ]
    // S-type: [imm[11:5]         ][rs2   ][rs1   ][funct3][imm[4:0]        ][opcode  ]
    // B-type: [imm[12]][imm[10:5]]........................[imm[4:1][imm[11]]..........
    // U-type: [                imm[31:12]                ][rd              ][opcode  ]
    // J-type: [imm[20]][ imm[10:1] ][imm[11]][imm[19:12] ]............................
    //          ^
    //          |____ sign

    reg ctrl_skip_next_reg;
    wire [6:0] opcode = inst[6:0];
    wire [2:0] funct3 = inst[14:12];
    wire [4:0] rs1 = inst[19:15];
    wire [4:0] rs2 = inst[24:20];
    wire [4:0] rd = inst[11:7];
    wire [31:0] ui = {inst[31:12], 12'b0}; //< load upper immediate
    wire [31:0] xxxi = {{20{inst[31]}}, inst[31:20]}; //< sign-extended immediate
    wire [20:0] jal_offset = {inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
    wire [11:0] jalr_offset = inst[31:20];
    wire [12:0] b_offset = {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    wire [11:0] l_offset = inst[31:20];
    wire [11:0] s_offset = {inst[31:25], inst[11:7]};
    wire is_nop = ctrl_is_nop || (opcode == `XXXI && inst[31:7] == 0) || ctrl_skip_next_reg;
    // wire to read from register file
    assign reg1_raddr = rs1;
    assign reg2_raddr = rs2;
    assign ctrl_pc_jump_target =
        opcode == `JAL ? pc + $signed({{11{jal_offset[20]}}, jal_offset}) :
        opcode == `JALR ? reg1_rdata + $signed({{20{jalr_offset[11]}}, jalr_offset}):
        opcode == `BXX ? pc + $signed({{19{b_offset[12]}}, b_offset}) : 'bx;

    assign ctrl_is_jump = opcode == `JAL ? 1 :
                          opcode == `JALR ? 1 :
                          opcode == `BXX ?
                                funct3[2:1] == 0 ? (rs1 == rs2) ^ (funct3[0]) : // BEQ & BNE
                                 (funct3[1] == 0 ?
                                    ($signed(rs1) < $signed(rs2)) ^ (funct3[0]) : // BLT & BGE
                                    (rs1 < rs2) ^ (funct3[0])) : 0; // BLTU & BGEU

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            ctrl_skip_next_reg <= 0;
            ctrl_is_nop_reg <= 1;
        end
        else
            if (!ctrl_stall && !is_nop) begin
                `ifndef SYNTHESIS
                    $display(
                        "[%0t] [decode] stage works on inst=%h pc=%h",
                        $time, inst, pc
                    );
                `endif
                case (opcode)
                    `XXXI: begin
                        op1_reg <= reg1_rdata;
                        op2_reg <= xxxi;
                        ctrl_alu_func_reg <= funct3;
                        ctrl_alu_sign_ext_reg <= inst[30];
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= 5'b00000;
                    end
                    `XXX: begin
                        op1_reg <= reg1_rdata;
                        op2_reg <= reg2_rdata;
                        ctrl_alu_func_reg <= funct3;
                        ctrl_alu_sign_ext_reg <= inst[30];
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= 5'b00000;
                    end
                    `LUI: begin
                        op1_reg <= ui;
                        op2_reg <= 0;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= 5'b00000;
                    end
                    `AUIPC: begin
                        op1_reg <= ui;
                        op2_reg <= pc;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= 5'b00000;
                    end
                    `LX: begin
                        op1_reg <= reg1_rdata;
                        op2_reg <= {{20{l_offset[11]}}, l_offset};
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= {funct3, 2'b10};
                    end
                    `SX: begin
                        op1_reg <= reg1_rdata;
                        op2_reg <= {{20{s_offset[11]}}, s_offset};
                        src_reg <= reg2_rdata;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 0;
                        ctrl_mem_reg <= {funct3, 2'b11};
                    end
                    `JAL, `JALR: begin
                        op1_reg <= pc;
                        op2_reg <= 4;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= {funct3, 2'b00};
                    end
                    default: begin
                        ctrl_wb_reg <= 0;
                        ctrl_mem_reg <= 5'b00000;
                    end
                endcase
                rd_reg <= rd;
                ctrl_skip_next_reg <= ctrl_is_jump;
                ctrl_is_nop_reg <= is_nop;
            end
            else begin
                `ifndef SYNTHESIS
                    $display("[%0t] [decode] stage idle", $time);
                `endif
                if (!ctrl_next_stage_stall)
                    ctrl_is_nop_reg <= 1;
            end
    end
endmodule

module executor(
    input [31:0] op1,
    input [31:0] op2,
    input [31:0] src,
    input [4:0] rd,

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input [2:0] ctrl_alu_func,
    input ctrl_alu_sign_ext,
    input ctrl_is_nop,
    input ctrl_wb,
    input [4:0] ctrl_mem,

    output reg [31:0] res_reg,
    output reg [31:0] src_reg,
    output reg [4:0] rd_reg,
    output reg ctrl_is_nop_reg,
    output reg ctrl_wb_reg,
    output reg [4:0] ctrl_mem_reg,
    output ctrl_executor_stall
);
    assign ctrl_executor_stall = 0;
    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset)
            ctrl_is_nop_reg <= 1;
        else
            if (!ctrl_stall && !ctrl_is_nop) begin
                `ifndef SYNTHESIS
                    $display(
                        "[%0t] [execution] stage works on op1=%h op2=%h src=%h rd=%d",
                        $time, op1, op2, src, rd
                    );
                `endif
                case (ctrl_alu_func)
                    `FADD: res_reg <= ctrl_alu_sign_ext ? op1 - op2 : op1 + op2;
                    `FSLT: res_reg <= $signed(op1) < $signed(op2) ? 1 : 0;
                    `FSLTU: res_reg <= op1 < op2 ? 1 : 0;
                    `FXOR: res_reg <= op1 ^ op2;
                    `FOR: res_reg <= op1 | op2;
                    `FAND: res_reg <= op1 & op2;
                    `FSLL: res_reg <= op1 << op2[4:0];
                    `FSRX: res_reg <= ctrl_alu_sign_ext ? $signed(op1) >> op2[4:0] : op1 >> op2[4:0];
                endcase
                src_reg <= src;
                rd_reg <= rd;
                ctrl_is_nop_reg <= ctrl_is_nop;
                ctrl_wb_reg <= ctrl_wb;
                ctrl_mem_reg <= ctrl_mem;
            end
            else begin
                `ifndef SYNTHESIS
                    $display("[%0t] [execution] stage idle", $time);
                `endif
                if (!ctrl_next_stage_stall)
                    ctrl_is_nop_reg <= 1;
            end
    end
endmodule

module memory(
    input [31:0] res_alu,
    input [31:0] src,
    input [4:0] rd,

    // d-cache communication
    output [31:0] dcache_addr,
    output [31:0] dcache_wdata,
    output [1:0] dcache_ws,
    output dcache_req,
    output dcache_wr,
    input [31:0] dcache_rdata,
    input dcache_rdy,

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input ctrl_wb,
    input ctrl_is_nop,
    input [4:0] ctrl_mem,

    output reg [31:0] res_reg,
    output reg [4:0] rd_reg,
    output reg ctrl_is_nop_reg,
    output reg ctrl_wb_reg,
    output ctrl_mem_stall
);
    assign dcache_req = ctrl_mem[1];
    assign dcache_wr = ctrl_mem[0];
    assign dcache_ws = ctrl_mem[3:2]; // SB/SH/SW
    assign dcache_wdata = src;
    assign dcache_addr = res_alu;
    assign ctrl_mem_stall = ctrl_wb && (!dcache_rdy);
    wire sgn = ctrl_mem[4];
    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset)
            ctrl_is_nop_reg <= 1;
        else
            if (!ctrl_stall && !ctrl_is_nop) begin
                `ifndef SYNTHESIS
                    $display(
                        "[%0t] [memory] stage works on res_alu=%h src=%h rd=%d",
                        $time, res_alu, src, rd
                    );
                `endif
                if (dcache_req == 1) begin
                    if (dcache_wr == 0) begin
                        case (dcache_ws)
                            'b00: res_reg <= {{24{sgn ? dcache_rdata[7] : 1'b0}}, dcache_rdata[7:0]}; // LB/LBU
                            'b01: res_reg <= {{16{sgn ? dcache_rdata[15] : 1'b0}}, dcache_rdata[15:0]}; // LH/LHU
                            'b10: res_reg <= dcache_rdata; // LW
                        endcase
                    end
                    else
                        res_reg <= res_alu;
                end
                rd_reg <= rd;
                ctrl_is_nop_reg <= ctrl_is_nop;
                ctrl_wb_reg <= ctrl_wb;
            end
            else begin
                `ifndef SYNTHESIS
                    $display("[%0t] [memory] stage idle", $time);
                `endif
                if (!ctrl_next_stage_stall)
                    ctrl_is_nop_reg <= 1;
            end
    end
endmodule

module writeback(
    input [31:0] res,
    input [4:0] rd,
    input ctrl_stall,
    input ctrl_wb,
    input ctrl_is_nop,

    output [4:0] reg_waddr,
    output [31:0] reg_wdata,
    output reg_wen,
    output ctrl_writeback_stall
);
    assign reg_waddr = rd;
    assign reg_wdata = res;
    assign reg_wen = !ctrl_stall && !ctrl_is_nop && ctrl_wb;
    assign ctrl_writeback_stall = 0;
endmodule

module pipeline (
    input clock,
    input reset,

    // i-cache communication
    output [31:0] icache_addr,
    output icache_req,
    input [31:0] icache_data,
    input icache_rdy,

    // d-cache communication
    output [31:0] dcache_addr,
    output [31:0] dcache_wdata,
    output [1:0] dcache_ws,
    output dcache_req,
    output dcache_wr,
    input [31:0] dcache_rdata,
    input dcache_rdy
);
    // program counter

    //wire [31:0] pc_reg_next_fetch;
    wire [31:0] pc_jump_target;
    wire ctrl_is_jump;
    wire ctrl_fetcher_stall;
    wire ctrl_decoder_stall;
    wire ctrl_executor_stall;
    wire ctrl_mem_stall;
    wire ctrl_writeback_stall;
    wire [4:0] reg1_raddr;
    wire [4:0] reg2_raddr;
    wire [31:0] reg1_rdata;
    wire [31:0] reg2_rdata;
    wire [4:0] reg_waddr;
    wire [31:0] reg_wdata;
    wire reg_wen;

    register_file main_reg(
        .reg1_raddr(reg1_raddr),
        .reg2_raddr(reg2_raddr),
        .reg1_rdata(reg1_rdata),
        .reg2_rdata(reg2_rdata),
        .reg_waddr(reg_waddr),
        .reg_wdata(reg_wdata),
        .reg_wen(reg_wen),
        .ctrl_clk(clock)
    );

    wire [31:0] pc_reg_fetch;
    wire [31:0] inst_reg_fetch;
    wire ctrl_is_nop_fetch;

    wire ctrl_fetcher_stall_in = ctrl_fetcher_stall ||
                                 ctrl_decoder_stall ||
                                 ctrl_executor_stall ||
                                 ctrl_mem_stall ||
                                 ctrl_writeback_stall;

    wire ctrl_decoder_stall_in = ctrl_decoder_stall ||
                                 ctrl_executor_stall ||
                                 ctrl_mem_stall ||
                                 ctrl_writeback_stall;

    wire ctrl_executor_stall_in = ctrl_executor_stall ||
                                  ctrl_mem_stall ||
                                  ctrl_writeback_stall;

    wire ctrl_mem_stall_in = ctrl_mem_stall ||
                             ctrl_writeback_stall;

    fetcher fetch_stage(
        .pc_jump_target(pc_jump_target),
        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_stall(ctrl_fetcher_stall_in),
        .ctrl_next_stage_stall(ctrl_decoder_stall_in),
        .ctrl_is_jump(ctrl_is_jump),
        .icache_addr(icache_addr),
        .icache_req(icache_req),
        .icache_data(icache_data),
        .icache_rdy(icache_rdy),
        .inst_reg(inst_reg_fetch),
        .pc_reg(pc_reg_fetch),
        .ctrl_is_nop_reg(ctrl_is_nop_fetch),
        .ctrl_fetcher_stall(ctrl_fetcher_stall)
    );

    wire [31:0] op1;
    wire [31:0] op2;
    wire [31:0] src_decode;
    wire [4:0] rd_decode;
    wire [2:0] ctrl_alu_func;
    wire ctrl_alu_sign_ext;
    wire ctrl_is_nop_decode;
    wire ctrl_wb_decode;
    wire [4:0] ctrl_mem_decode;

    decoder decode_stage(
        .inst(inst_reg_fetch),
        .pc(pc_reg_fetch),
        .reg1_raddr(reg1_raddr),
        .reg2_raddr(reg2_raddr),
        .reg1_rdata(reg1_rdata),
        .reg2_rdata(reg2_rdata),
        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_stall(ctrl_decoder_stall_in),
        .ctrl_next_stage_stall(ctrl_executor_stall_in),
        .ctrl_is_nop(ctrl_is_nop_fetch),
        .op1_reg(op1),
        .op2_reg(op2),
        .src_reg(src_decode),
        .rd_reg(rd_decode),
        .ctrl_alu_func_reg(ctrl_alu_func),
        .ctrl_alu_sign_ext_reg(ctrl_alu_sign_ext),
        .ctrl_is_nop_reg(ctrl_is_nop_decode),
        .ctrl_wb_reg(ctrl_wb_decode),
        .ctrl_mem_reg(ctrl_mem_decode),
        .ctrl_pc_jump_target(pc_jump_target),
        .ctrl_is_jump(ctrl_is_jump),
        .ctrl_decoder_stall(ctrl_decoder_stall)
    );

    wire [31:0] res_exec;
    wire [31:0] src_exec;
    wire [4:0] rd_exec;
    wire ctrl_is_nop_exec;
    wire ctrl_wb_exec;
    wire [4:0] ctrl_mem_exec;

    executor exec_stage(
        .op1(op1),
        .op2(op2),
        .src(src_decode),
        .rd(rd_decode),
        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_stall(ctrl_executor_stall_in),
        .ctrl_next_stage_stall(ctrl_mem_stall_in),
        .ctrl_alu_func(ctrl_alu_func),
        .ctrl_alu_sign_ext(ctrl_alu_sign_ext),
        .ctrl_is_nop(ctrl_is_nop_decode),
        .ctrl_wb(ctrl_wb_decode),
        .ctrl_mem(ctrl_mem_decode),
        .res_reg(res_exec),
        .src_reg(src_exec),
        .rd_reg(rd_exec),
        .ctrl_is_nop_reg(ctrl_is_nop_exec),
        .ctrl_wb_reg(ctrl_wb_exec),
        .ctrl_mem_reg(ctrl_mem_exec),
        .ctrl_executor_stall(ctrl_executor_stall)
    );

    wire [4:0] rd_mem;
    wire [31:0] res_mem;
    wire ctrl_is_nop_mem;
    wire ctrl_wb_mem;

    memory mem_stage(
        .res_alu(res_exec),
        .src(src_exec),
        .rd(rd_exec),
        .dcache_addr(dcache_addr),
        .dcache_wdata(dcache_wdata),
        .dcache_ws(dcache_ws),
        .dcache_req(dcache_req),
        .dcache_wr(dcache_wr),
        .dcache_rdata(dcache_rdata),
        .dcache_rdy(dcache_rdy),
        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_stall(ctrl_mem_stall_in),
        .ctrl_next_stage_stall(0),
        .ctrl_is_nop(ctrl_is_nop_exec),
        .ctrl_wb(ctrl_wb_exec),
        .ctrl_mem(ctrl_mem_exec),
        .res_reg(res_mem),
        .rd_reg(rd_mem),
        .ctrl_is_nop_reg(ctrl_is_nop_mem),
        .ctrl_wb_reg(ctrl_wb_mem),
        .ctrl_mem_stall(ctrl_mem_stall)
    );

    writeback wb_stage(
        .res(res_mem),
        .rd(rd_mem),
        .ctrl_stall(0),
        .ctrl_wb(ctrl_wb_mem),
        .ctrl_is_nop(ctrl_is_nop_mem),
        .reg_waddr(reg_waddr),
        .reg_wdata(reg_wdata),
        .reg_wen(reg_wen),
        .ctrl_writeback_stall(ctrl_writeback_stall)
    );
endmodule

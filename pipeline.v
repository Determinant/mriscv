module pc_selector_fetch(
    input [31:0] pc_jump_target,
    input [31:0] pc_inc_fetch,
    input ctrl_is_jump,

    output [31:0] pc_next_fetch
);
    assign pc_next_fetch = ctrl_is_jump ? pc_jump_target : pc_inc_fetch;
endmodule

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
            regs[reg_waddr] <= reg_wdata;
    end
endmodule

`define LUI     7'b0110111
`define AUIPC   7'b0010111
`define JAL     7'b1101111
`define JALR    7'b1100111
`define BXX     7'b1100011 // BEQ, BNE, BLT, BGE, BLTU, BGEU
`define LX     7'b0000011 // LB, LH, LW, LBU, LHU
`define SX      7'b0100011 // SB, SH, SW
`define XXXI    7'b0010011 // ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
`define XXX     7'b0110011 // ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND

`define FEN     7'b0001111 // FENCE
`define EXX     7'b1110011 // ECALL, EBREAK

`define FADDI   3'b000
`define FSLTI   3'b010
`define FSLTIU  3'b011
`define FXORI   3'b100
`define FORI    3'b110
`define FANDI   3'b111

`define FSLLI   3'b001
`define FSRXI   3'b101 // FSRLI & SRAI

`define FBEQ    3'b000
`define FBNE    3'b001
`define FBLT    3'b100
`define FBGE    3'b101
`define FBLTU   3'b110
`define FBGEU   3'b111

function is_stall (
    input ctrl_fetcher_stall,
    input ctrl_decoder_stall,
    input ctrl_executor_stall,
    input ctrl_mem_stall,
    input ctrl_writeback_stall,
);
    is_stall =
        ctrl_fetcher_stall ||
        ctrl_decoder_stall ||
        ctrl_executor_stall ||
        ctrl_mem_stall ||
        ctrl_writeback_stall;
endfunction

module fetcher(
    input [31:0] pc_in,
    input ctrl_clk,
    input ctrl_stall,

    // i-cache communication
    output [31:0] icache_addr,
    input [31:0] icache_data,
    input icache_rdy,

    output reg [31:0] inst,
    output reg [31:0] pc,
    output ctrl_fetcher_stall
);
    assign icache_addr = pc_in;
    assign ctrl_fetcher_stall = !icache_rdy;
    always_ff @ (posedge ctrl_clk) begin
        if (!ctrl_stall) begin
            inst <= icache_data;
            pc <= pc_in;
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
    input ctrl_stall,

    output reg [31:0] op1_reg,
    output reg [31:0] op2_reg,
    output reg [4:0] rd_reg,
    output reg [31:0] src_reg,
    output reg [2:0] ctrl_alu_func_reg,
    output reg ctrl_is_nop_reg,
    output reg ctrl_wb_reg,
    output reg [1:0] ctrl_mem_reg,
    output [31:0] ctrl_pc_jump_target,
    output ctrl_is_jump,
    output ctrl_decoder_stall
);
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
    //wire [6:0] funct7 = inst[31:25];
    wire [4:0] rs1 = inst[19:15];
    wire [4:0] rs2 = inst[24:20];
    wire [4:0] rd = inst[11:7];
    wire [31:0] lui = {(opcode[4:0] == 5'b10111) ? inst[31:12] : 20'bx, 12'b0};
    wire [31:0] xxxi = (opcode == `XXXI) ? {{20{inst[31]}}, inst[31:20]} : 'bx;
    wire [20:0] jal_offset = {inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
    wire [11:0] jalr_offset = inst[31:20];
    wire [12:0] b_offset = {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
    wire [11:0] l_offset = inst[31:20];
    wire [11:0] s_offset = {inst[31:25], inst[11:7]};
    wire is_nop = 
        (opcode == `XXXI && funct3 == `FADDI && inst[31:15] == 0 && rd == 0) || ctrl_skip_next_reg;
    // wire to read from register file
    assign reg1_raddr = rs1;
    assign reg2_raddr = rs2;
    assign ctrl_pc_jump_target = opcode == `JAL ?
                            pc + {{11{jal_offset[20]}}, jal_offset} :
                            opcode == `JALR ?
                            reg1_rdata + {{20{jalr_offset[11]}}, jalr_offset}:
                            opcode == `BXX ?
                            pc + {{19{b_offset[12]}}, b_offset} : 'bx;

    assign ctrl_is_jump = opcode == `JAL ? 1 :
                          opcode == `JALR ? 1 :
                          opcode == `BXX ?
                                funct3[2:1] == 0 ? (rs1 == rs2) ^ (funct3[0]) : // BEQ & BNE
                                 (funct3[1] == 0 ?
                                    ($signed(rs1) < $signed(rs2)) ^ (funct3[0]) : // BLT & BGE
                                    (rs1 < rs2) ^ (funct3[0])) : 0; // BLTU & BGEU

    always_ff @ (posedge ctrl_clk) begin
        if (!ctrl_stall) begin
            case (opcode)
                `XXXI: begin
                    op1_reg <= reg1_rdata;
                    op2_reg <= xxxi;
                    ctrl_alu_func_reg <= funct3;
                    ctrl_wb_reg <= 1;
                    ctrl_mem_reg <= 2'b00;
                end
                `XXX: begin
                    op1_reg <= reg1_rdata;
                    op2_reg <= reg2_rdata;
                    ctrl_alu_func_reg <= funct3;
                    ctrl_wb_reg <= 1;
                    ctrl_mem_reg <= 2'b00;
                end
                `LUI: begin
                    op1_reg <= lui;
                    op2_reg <= 0;
                    ctrl_alu_func_reg <= `FADDI;
                    ctrl_wb_reg <= 1;
                    ctrl_mem_reg <= 2'b00;
                end
                `AUIPC: begin
                    op1_reg <= lui;
                    op2_reg <= pc;
                    ctrl_alu_func_reg <= `FADDI;
                    ctrl_wb_reg <= 1;
                    ctrl_mem_reg <= 2'b00;
                end
                `LX: begin
                    op1_reg <= reg1_rdata;
                    op2_reg <= {{20{l_offset[11]}}, l_offset};
                    ctrl_alu_func_reg <= `FADDI;
                    ctrl_wb_reg <= 1;
                    ctrl_mem_reg <= 2'b10;
                end
                `SX: begin
                    op1_reg <= reg1_rdata;
                    op2_reg <= {{20{s_offset[11]}}, s_offset};
                    src_reg <= reg2_rdata;
                    ctrl_alu_func_reg <= `FADDI;
                    ctrl_wb_reg <= 0;
                    ctrl_mem_reg <= 2'b11;
                end
                default: begin
                    ctrl_wb_reg <= 0;
                    ctrl_mem_reg <= 2'b00;
                end
            endcase
            rd_reg <= rd;
            ctrl_skip_next_reg <= ctrl_is_jump;
            ctrl_is_nop_reg <= is_nop;
        end
    end

    assign ctrl_decoder_stall = 0;
endmodule

module executor(
    input [31:0] op1,
    input [31:0] op2,
    input [31:0] src,
    input [4:0] rd,
    input ctrl_clk,
    input ctrl_stall,
    input [2:0] ctrl_alu_func,
    input ctrl_is_nop,
    input ctrl_wb,
    input [1:0] ctrl_mem,

    output reg [31:0] res_reg,
    output reg [31:0] src_reg,
    output reg [4:0] rd_reg,
    output reg ctrl_is_nop_reg,
    output reg ctrl_wb_reg,
    output reg [1:0] ctrl_mem_reg
);
    always_ff @ (posedge ctrl_clk) begin
        if (!ctrl_stall && !ctrl_is_nop) begin
            case (ctrl_alu_func)
                `FADDI: res_reg <= op1 + op2;
                `FSLTI: res_reg <= $signed(op1) < $signed(op2) ? 1 : 0;
                `FSLTIU: res_reg <= op1 < op2 ? 1 : 0;
                `FXORI: res_reg <= op1 ^ op2;
                `FORI: res_reg <= op1 | op2;
                `FANDI: res_reg <= op1 & op2;
                `FSLLI: res_reg <= op1 << op2[4:0];
                `FSRXI: res_reg <= op2[10] == 0 ? op1 >> op2[4:0] : $signed(op1) >> op2[4:0];
            endcase
            src_reg <= src;
            rd_reg <= rd;
            ctrl_is_nop_reg <= ctrl_is_nop;
            ctrl_wb_reg <= ctrl_wb;
            ctrl_mem_reg <= ctrl_mem;
        end
    end
endmodule

module memory(
    input [31:0] res_alu,
    input [31:0] src,
    input [4:0] rd,

    output [31:0] dcache_addr,
    input [31:0] dcache_rdata,
    output [31:0] dcache_wdata,
    input dcache_rdy,
    output dcache_en,
    output dcache_wr,

    input ctrl_clk,
    input ctrl_stall,
    input ctrl_wb,
    input ctrl_is_nop,
    input [1:0] ctrl_mem,

    output reg [31:0] res_reg,
    output reg [4:0] rd_reg,
    output reg ctrl_is_nop_reg,
    output reg ctrl_wb_reg,
    output ctrl_mem_stall
);
    assign dcache_en = ctrl_mem[1];
    assign dcache_wr = ctrl_mem[0];
    assign dcache_wdata = src;
    assign dcache_addr = res_alu;
    assign ctrl_mem_stall = !dcache_rdy;
    always_ff @ (posedge ctrl_clk) begin
        if (!ctrl_stall && !ctrl_is_nop) begin
            if (ctrl_mem[1] == 1) begin
                if (ctrl_mem[0] == 0) begin
                    res_reg <= dcache_rdata;
                end
            else
                res_reg <= res_alu;
            end
            rd_reg <= rd;
            ctrl_is_nop_reg <= ctrl_is_nop;
            ctrl_wb_reg <= ctrl_wb;
        end
    end
endmodule

module writeback(
    input [31:0] res,
    input [4:0] rd,
    input ctrl_clk,
    input ctrl_stall,
    input ctrl_wb,
    input ctrl_is_nop,

    output [4:0] reg_waddr,
    output [31:0] reg_wdata,
    output reg_wen
);
    assign reg_waddr = rd;
    assign reg_wdata = res;
    assign reg_wen = !ctrl_stall && !ctrl_is_nop && ctrl_wb;
endmodule

module pipeline (
    input clock,

    output [31:0] icache_addr,
    input [31:0] icache_data,
    input icache_rdy,

    output [31:0] dcache_addr,
    input [31:0] dcache_rdata,
    output [31:0] dcache_wdata,
    input dcache_rdy,
    output dcache_en,
    output dcache_wr
);
    // program counter

    reg [31:0] pc_reg_fetch; //< PC register before fetch stage //
    wire [31:0] pc_reg_decode; //< PC register before decoding stage //
    wire [31:0] inst_reg_decode; //< instruction register before decoding stage //

    wire [31:0] pc_reg_next_fetch;
    wire [31:0] pc_jump_target;
    wire ctrl_is_jump;
    wire ctrl_fetcher_stall;
    wire ctrl_decoder_stall;
    wire ctrl_executor_stall;
    wire ctrl_mem_stall;
    wire ctrl_writeback_stall;
    wire ctrl_stall = is_stall(
        ctrl_fetcher_stall,
        ctrl_decoder_stall,
        ctrl_executor_stall,
        ctrl_mem_stall,
        ctrl_writeback_stall
    );
    // define PC Mux
    pc_selector_fetch pc_sel(
        .pc_jump_target(pc_jump_target),
        .pc_inc_fetch(pc_reg_fetch + 4),
        .pc_next_fetch(pc_reg_next_fetch),
        .ctrl_is_jump(ctrl_is_jump)
    );

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

    fetcher fetch_stage(
        .pc_in(pc_reg_fetch),
        .ctrl_clk(clock),
        .ctrl_stall(ctrl_stall),
        .ctrl_fetcher_stall(ctrl_fetcher_stall),
        .icache_addr(icache_addr),
        .icache_rdy(icache_rdy),
        .icache_data(icache_data),
        .inst(inst_reg_decode),
        .pc(pc_reg_decode)
    );

    wire [31:0] op1;
    wire [31:0] op2;
    wire [31:0] src_decode;
    wire [4:0] rd_decode;
    wire [2:0] ctrl_alu_func;
    wire ctrl_is_nop_decode;
    wire ctrl_wb_decode;
    wire [1:0] ctrl_mem_decode;

    decoder decode_stage(
        .inst(inst_reg_decode),
        .pc(pc_reg_decode),
        .reg1_raddr(reg1_raddr),
        .reg2_raddr(reg2_raddr),
        .reg1_rdata(reg1_rdata),
        .reg2_rdata(reg2_rdata),
        .ctrl_clk(clock),
        .ctrl_stall(ctrl_stall),
        .op1_reg(op1),
        .op2_reg(op2),
        .src_reg(src_decode),
        .rd_reg(rd_decode),
        .ctrl_alu_func_reg(ctrl_alu_func),
        .ctrl_is_nop_reg(ctrl_is_nop_decode),
        .ctrl_wb_reg(ctrl_wb_decode),
        .ctrl_mem_reg(ctrl_mem_decode),
        .ctrl_pc_jump_target(pc_jump_target),
        .ctrl_is_jump(ctrl_is_jump),
        .ctrl_decoder_stall(ctrl_decoder_stall)
    );

    wire [31:0] res;
    wire [31:0] src_execute;
    wire [4:0] rd_execute;
    wire ctrl_is_nop_execute;
    wire ctrl_wb_execute;
    wire [1:0] ctrl_mem_execute;

    executor execute_stage(
        .op1(op1),
        .op2(op2),
        .src(src_decode),
        .rd(rd_decode),
        .ctrl_clk(clock),
        .ctrl_stall(ctrl_stall),
        .ctrl_alu_func(ctrl_alu_func),
        .ctrl_is_nop(ctrl_is_nop_decode),
        .ctrl_wb(ctrl_wb_decode),
        .ctrl_mem(ctrl_mem_decode),
        .res_reg(res),
        .src_reg(src_execute),
        .ctrl_is_nop_reg(ctrl_is_nop_execute),
        .ctrl_wb_reg(ctrl_wb_execute),
        .ctrl_mem_reg(ctrl_mem_execute),
        .rd_reg(rd_execute)
    );

    wire [4:0] rd_mem;
    wire [31:0] res_mem;
    wire ctrl_is_nop_mem;
    wire ctrl_wb_mem;

    memory mem_stage(
        .res_alu(res),
        .src(src_execute),
        .rd(rd_execute),
        .dcache_addr(dcache_addr),
        .dcache_rdata(dcache_rdata),
        .dcache_wdata(dcache_wdata),
        .dcache_rdy(dcache_rdy),
        .dcache_en(dcache_en),
        .dcache_wr(dcache_wr),
        .ctrl_clk(clock),
        .ctrl_stall(ctrl_stall),
        .ctrl_is_nop(ctrl_is_nop_execute),
        .ctrl_wb(ctrl_wb_execute),
        .ctrl_mem(ctrl_mem_execute),
        .res_reg(res_mem),
        .rd_reg(rd_mem),
        .ctrl_is_nop_reg(ctrl_is_nop_mem),
        .ctrl_wb_reg(ctrl_wb_mem),
        .ctrl_mem_stall(ctrl_mem_stall)
    );

    writeback wb_stage(
        .res(res_mem),
        .rd(rd_mem),
        .ctrl_clk(clock),
        .ctrl_stall(ctrl_stall),
        .ctrl_wb(ctrl_wb_mem),
        .ctrl_is_nop(ctrl_is_nop_mem),
        .reg_waddr(reg_waddr),
        .reg_wdata(reg_wdata),
        .reg_wen(reg_wen)
    );

    always_ff @ (posedge clock) begin
        pc_reg_fetch <= pc_reg_next_fetch;
    end
endmodule

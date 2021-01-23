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
    input ctrl_executer_stall,
    input ctrl_mem_stall,
    input ctrl_writeback_stall,
);
    is_stall =
        ctrl_fetcher_stall ||
        ctrl_decoder_stall ||
        ctrl_executer_stall ||
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
    assign ctrl_fetcher_stall = icache_rdy;
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
    output reg [2:0] ctrl_alu_func_reg,
    output [31:0] pc_jump_target,
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

    wire [6:0] opcode = inst[6:0];
    wire [2:0] funct3 = inst[14:12];
    wire [6:0] funct7 = inst[31:25];
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
    wire shift_sign = inst[31:25] == 'b0100000;
    wire is_nop = opcode == `XXXI && funct3 == `FADDI && inst[31:15] == 0 && rd == 0;
    // wire to read from register file
    assign reg1_raddr = rs1;
    assign reg2_raddr = rs2;
    assign pc_jump_target = opcode == `JAL ?
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
                end
                `XXX: begin
                    op1_reg <= reg1_rdata;
                    op2_reg <= reg2_rdata;
                end
                `LUI: op1_reg <= lui;
                `AUIPC: op1_reg <= lui;
                `LX: begin
                    op1_reg <= reg1_rdata + {{20{l_offset[11]}}, l_offset};
                end
                `SX: begin
                    op1_reg <= reg1_rdata + {{20{s_offset[11]}}, s_offset};
                end
                default:;
            endcase
        end
    end

    assign ctrl_decoder_stall = 0;
endmodule

module executor(
    input [31:0] op1,
    input [31:0] op2,
    input [2:0] ctrl_alu_func,

    output reg [31:0] res_reg
);
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
    output dcache_wen
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
    wire ctrl_executer_stall;
    wire ctrl_mem_stall;
    wire ctrl_writeback_stall;
    wire ctrl_stall = is_stall(
        ctrl_fetcher_stall,
        ctrl_decoder_stall,
        ctrl_executer_stall,
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

    register_file main_reg(
        .reg1_raddr(reg1_raddr),
        .reg2_raddr(reg2_raddr),
        .reg1_rdata(reg1_rdata),
        .reg2_rdata(reg2_rdata),
        .reg_waddr(),
        .reg_wdata(),
        .reg_wen(),
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

    decoder decode_stage(
        .inst(inst_reg_decode),
        .pc(pc_reg_decode),
        .pc_jump_target(pc_jump_target),
        .reg1_raddr(reg1_raddr),
        .reg2_raddr(reg2_raddr),
        .reg1_rdata(reg1_rdata),
        .reg2_rdata(reg2_rdata),
        .ctrl_clk(clock),
        .ctrl_stall(ctrl_stall),
        .ctrl_decoder_stall(ctrl_decoder_stall),
        .ctrl_is_jump(ctrl_is_jump),
        .op1_reg(),
        .op2_reg(),
        .ctrl_alu_func_reg()
    );

    executor execute_stage(
        .op1(),
        .op2(),
        .ctrl_alu_func(),
        .res_reg()
    );

    always_ff @ (posedge clock) begin
        pc_reg_fetch <= pc_reg_next_fetch;
    end
endmodule

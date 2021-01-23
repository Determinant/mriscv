module pc_selector_fetch(
    input [31:0] pc_jump_target,
    input [31:0] pc_inc_fetch,
    output [31:0] pc_next_fetch,
    // control signal
    input ctrl_pc_selector
);
    assign pc_next_fetch = pc_inc_fetch;
endmodule

module register_file(
    input [4:0] reg1_raddr,
    input [4:0] reg2_raddr,
    output [31:0] reg1_rdata,
    output [31:0] reg2_rdata,

    input [4:0] reg_waddr,
    input [31:0] reg_wdata,

    input ctrl_wen, //< write enable
    input ctrl_clk //< clock
);
    // register file
    reg [31:0] regs [31:0];
    assign reg1_rdata = regs[reg1_raddr];
    assign reg2_rdata = regs[reg2_raddr];
    assign regs[0] = 0;
    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_wen && reg_waddr != 0)
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

module decoder(
    input [31:0] inst,
    input [31:0] pc,
    output [31:0] op1,
    output [31:0] op2,
    output [31:0] pc_target,

    output [4:0] reg1_raddr,
    output [4:0] reg2_raddr,
    input [31:0] reg1_rdata,
    input [31:0] reg2_rdata,

    input ctrl_clk,
    output [31:0] ctrl_alu_func
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

    reg [31:0] op1_reg;
    reg [31:0] op2_reg;

    assign op1 = op1_reg;
    assign op2 = op2_reg;

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
    assign pc_target = opcode == `JAL ?
                       pc + {{11{jal_offset[20]}}, jal_offset} :
                       opcode == `JALR ?
                       reg1_rdata + {{20{jalr_offset[11]}}, jalr_offset}:
                       opcode == `BXX ?
                       pc + {{19{b_offset[12]}}, b_offset} : 'bx;

    always_ff @ (posedge ctrl_clk) begin
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
endmodule

module pipeline (
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
    // program counter
    
    reg [31:0] pc_reg_fetch; //< PC register before fetch stage //
    reg [31:0] pc_reg_decode; //< PC register before decoding stage //
    reg [31:0] inst_reg_decode; //< instruction register before decoding stage //


    wire [31:0] pc_reg_next_fetch;
    wire [31:0] pc_jump_target;

    // define PC Mux
    pc_selector_fetch pc_sel(
        .pc_jump_target(pc_jump_target),
        .pc_inc_fetch(pc_reg_fetch + 4),
        .pc_next_fetch(pc_reg_next_fetch),
        .ctrl_pc_selector()
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
        .ctrl_wen(),
        .ctrl_clk()
    );

    decoder dec(
        .inst(),
        .pc(pc_reg_decode),
        .pc_target(pc_jump_target),
        .op1(),
        .op2(),
        .reg1_raddr(reg1_raddr),
        .reg2_raddr(reg2_raddr),
        .reg1_rdata(reg1_rdata),
        .reg2_rdata(reg2_rdata),
        .ctrl_clk(clock),
        .ctrl_alu_func()
    );

    assign inst_addr = pc_reg_fetch;
    assign pc_reg_decode = pc_reg_fetch;

    always_ff @ (posedge clock) begin
        pc_reg_fetch <= pc_reg_next_fetch;
        pc_reg_decode <= pc_reg_fetch;
    end
endmodule

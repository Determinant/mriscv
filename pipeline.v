module pc_selector_fetch(
    input [31:0] jalr_target_exec,
    input [31:0] br_target_exec,
    input [31:0] jal_target_decode,
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
`define LXX     7'b0000011 // LB, LH, LW, LBU, LHU
`define SX      7'b0100011 // SB, SH, SW
`define XXXI    7'b0010011 // ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
`define XXX     7'b0110011 // ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND

`define FEN     7'b0001111 // FENCE
`define EXX     7'b1110011 // ECALL, EBREAK

module decoder(
    input [31:0] inst,
    output [31:0] op1,
    output [31:0] op2,

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
    wire [31:0] lui;
    assign lui[11:0] = 0;
    assign lui[31:12] = (opcode[4:0] == 5'b10111) ? inst[31:12] : 20'bx;
    wire [31:0] xxxi = (opcode == `XXXI) ? {{20{inst[31]}}, inst[31:20]} : 'bx;
    always_ff @ (posedge ctrl_clk) begin
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
    reg [31:0] pc;
    
    reg [31:0] pc_reg_fetch; //< PC register before fetch stage //

    reg [31:0] pc_reg_decode; //< PC register before decoding stage //
    reg [31:0] inst_reg_decode; //< instruction register before decoding stage //


    wire [31:0] pc_reg_next_fetch;
    // define PC Mux
    pc_selector_fetch pc_sel(
        .jalr_target_exec(),
        .br_target_exec(),
        .jal_target_decode(),
        .pc_inc_fetch(pc_reg_fetch + 4),
        .pc_next_fetch(pc_reg_next_fetch),
        .ctrl_pc_selector()
    );

    register_file main_reg(
        .reg1_raddr(),
        .reg2_raddr(),
        .reg1_rdata(),
        .reg2_rdata(),
        .reg_waddr(),
        .reg_wdata(),
        .ctrl_wen(),
        .ctrl_clk()
    );

    decoder dec(
        .inst(),
        .op1(),
        .op2(),
        .ctrl_clk(clock),
        .ctrl_alu_func()
    );

    assign inst_addr = pc_reg_fetch;
    assign pc_reg_decode = pc_reg_fetch;

    always_ff @ (posedge clock) begin
        pc_reg_fetch <= pc_reg_next_fetch;
    end
endmodule

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
    // register file
    reg [31:0] regs [31:0];

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

    assign inst_addr = pc_reg_fetch;
    assign pc_reg_decode = pc_reg_fetch;

    always_ff @ (posedge clock) begin
        pc_reg_fetch <= pc_reg_next_fetch;
    end
endmodule

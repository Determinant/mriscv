`include "csr_file.sv"
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
`define SYS     7'b1110011 // ECALL, EBREAK, CSRx

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

`define CSRRW   2'b01
`define CSRRS   2'b10
`define CSRRC   2'b11

`ifndef SYNTHESIS
    `ifdef DEBUG
        `define PIPELINE_DEBUG
    `endif
`endif

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
    logic [31:0] regs [31:0];
    wire writing = reg_wen && reg_waddr != 0;
    assign reg1_rdata = writing && reg_waddr == reg1_raddr ? reg_wdata : regs[reg1_raddr];
    assign reg2_rdata = writing && reg_waddr == reg2_raddr ? reg_wdata : regs[reg2_raddr];
    assign regs[0] = 0;
    always_ff @ (posedge ctrl_clk) begin
        if (writing) begin
            `ifdef PIPELINE_DEBUG
                $display(
                    "[%0t] *[wb] write 0x%h to register %d",
                    $time, reg_wdata, reg_waddr
                );
            `endif
            regs[reg_waddr] <= reg_wdata;
        end else begin
            `ifdef PIPELINE_DEBUG
                $display("[%0t]  [wb] idle", $time);
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

    output logic [31:0] inst_reg,
    output logic [31:0] pc_reg,
    output logic ctrl_is_nop_reg,
    output ctrl_fetcher_stall
);
    logic [31:0] pc;
    assign icache_addr = pc;
    assign icache_req = 1;
    assign ctrl_fetcher_stall = !icache_rdy;
    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            `ifdef PIPELINE_DEBUG
                $display("[%0t] reset pc", $time);
            `endif
            pc <= 32'h00000000; // initialize PC to 0x00000000
            ctrl_is_nop_reg <= 1;
        end else if (!ctrl_stall) begin
            `ifdef PIPELINE_DEBUG
                $display(
                    "[%0t] *[fetch] stage works on pc=0x%h jump=(0x%h,%b)",
                    $time, pc, pc_jump_target, ctrl_is_jump);
            `endif
            pc <= ctrl_is_jump ? pc_jump_target : pc + 4;
            inst_reg <= icache_data;
            pc_reg <= pc;
            ctrl_is_nop_reg <= 0;
        end else begin
            `ifdef PIPELINE_DEBUG
                $display("[%0t] =[fetch] stage stalls on pc=0x%h jump=(0x%h,%b)",
                    $time, pc, pc_jump_target, ctrl_is_jump);
            `endif
            if (!ctrl_next_stage_stall) // insert a bubble
                ctrl_is_nop_reg <= 1;
        end
    end
endmodule;

module decoder(
    input [31:0] inst,
    input [31:0] pc,
    input [31:0] reg1_rdata,
    input [31:0] reg2_rdata,
    input [31:0] csr_rdata,
    output [4:0] reg1_raddr,
    output [4:0] reg2_raddr,
    output [11:0] csr_addr,
    output [31:0] csr_wdata,
    output csr_wen,

    // register forwarding from executor
    input [31:0] forward_data_exec,
    input [4:0] ctrl_forward_rd_exec,
    input ctrl_forward_valid_exec,
    input ctrl_forward_mload_stall,

    input [31:0] forward_data_mem,
    input [4:0] ctrl_forward_rd_mem,
    input ctrl_forward_valid_mem,
    //

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input ctrl_is_nop,
    output [31:0]_debug_pc_addr,

    output logic [31:0] op1_reg,
    output logic [31:0] op2_reg,
    output logic [4:0] rd_reg,
    output logic [31:0] src_reg,
    output logic [2:0] ctrl_alu_func_reg,
    output logic ctrl_alu_sign_ext_reg,
    output logic ctrl_is_nop_reg,
    output logic ctrl_wb_reg,
    output logic [4:0] ctrl_mem_reg,
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

    // instruction fields
    logic ctrl_skip_next_reg;
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
    wire [11:0] csr = inst[31:20];
    wire is_nop = ctrl_is_nop ||
                  ctrl_skip_next_reg ||
                  (opcode == `XXXI && inst[31:7] == 0) ||
                  opcode == `FEN;

    assign ctrl_decoder_stall = ctrl_forward_mload_stall &&
                                (((opcode == `XXXI ||
                                  opcode == `XXX ||
                                  opcode == `LX ||
                                  opcode == `SX ||
                                  opcode == `JALR ||
                                  opcode == `BXX) && ctrl_forward_rd_exec == rs1) ||
                                ((opcode == `XXX ||
                                  opcode == `SX ||
                                  opcode == `JALR ||
                                  opcode == `BXX) && ctrl_forward_rd_exec == rs2));

    // wire to read from register file
    assign reg1_raddr = rs1;
    assign reg2_raddr = rs2;
    assign csr_addr = csr;

    wire [31:0] op1 = (opcode == `SYS && funct3 != 0) ? csr_rdata :
                      (ctrl_forward_valid_exec && rs1 == ctrl_forward_rd_exec) ? forward_data_exec:
                      (ctrl_forward_valid_mem && rs1 == ctrl_forward_rd_mem) ? forward_data_mem:
                                                                               reg1_rdata;
    wire [31:0] op2 = (ctrl_forward_valid_exec && rs2 == ctrl_forward_rd_exec) ? forward_data_exec:
                      (ctrl_forward_valid_mem && rs2 == ctrl_forward_rd_mem) ? forward_data_mem:
                                                                               reg2_rdata;
    // CSR
    wire [1:0] csr_func = funct3[1:0];
    wire [31:0] csr_src = funct3[2] ? {27'b0, rs1} : op1;
    assign csr_wdata =
        csr_func == `CSRRW ? csr_src :
        csr_func == `CSRRS ? (csr_rdata | csr_src) :
        csr_func == `CSRRC ? (csr_rdata & (~csr_src)) : 'bx;
    assign csr_wen = csr_func != 0 && rs1 != 0;

    // set jump signal for control transfer instructions
    assign ctrl_pc_jump_target =
        opcode == `JAL ? pc + $signed({{11{jal_offset[20]}}, jal_offset}) :
        opcode == `JALR ? op1 + $signed({{20{jalr_offset[11]}}, jalr_offset}):
        opcode == `BXX ? pc + $signed({{19{b_offset[12]}}, b_offset}) : 'bx;
    assign ctrl_is_jump = (!ctrl_skip_next_reg) &&
        (opcode == `JAL ? 1 :
        opcode == `JALR ? 1 :
        opcode == `BXX ?
              funct3[2:1] == 0 ? ((op1 == op2) ^ (funct3[0])) : // BEQ & BNE
               (funct3[1] == 0 ?
                  (($signed(op1) < $signed(op2)) ^ (funct3[0])) : // BLT & BGE
                  ((op1 < op2) ^ (funct3[0]))) : 0); // BLTU & BGEU

    assign _debug_pc_addr = is_nop ? 'hffffffff : pc;

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            ctrl_skip_next_reg <= 0;
            ctrl_is_nop_reg <= 1;
        end else if (!ctrl_stall) begin
            if (!is_nop) begin
                `ifdef PIPELINE_DEBUG
                    $display(
                        "[%0t] *[decode] stage works on inst=0x%h pc=0x%h rs1=0x%h rs2=0x%h skip=%b",
                        $time, inst, pc, op1, op2, ctrl_skip_next_reg);
                `endif
                case (opcode)
                    `XXXI: begin
                        op1_reg <= op1;
                        op2_reg <= xxxi;
                        ctrl_alu_func_reg <= funct3;
                        ctrl_alu_sign_ext_reg <= funct3 == `FSRX ? inst[30] : 0;
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= 5'b00000;
                    end
                    `XXX: begin
                        op1_reg <= op1;
                        op2_reg <= op2;
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
                        op1_reg <= op1;
                        op2_reg <= {{20{l_offset[11]}}, l_offset};
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_mem_reg <= {funct3, 2'b10};
                    end
                    `SX: begin
                        op1_reg <= op1;
                        op2_reg <= {{20{s_offset[11]}}, s_offset};
                        src_reg <= op2;
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
                    `SYS: begin
                        op1_reg <= csr_rdata;
                        op2_reg <= 0;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= funct3 != 0; // all CSR instructions writes to rd
                        ctrl_mem_reg <= 5'b00000;
                    end
                    default: begin // TODO: illegal instruction detection
                        ctrl_wb_reg <= 0;
                        ctrl_mem_reg <= 5'b00000;
                    end
                endcase
                rd_reg <= rd;
                ctrl_skip_next_reg <= ctrl_is_jump;
            end else begin // NOP or a bubble
                `ifdef PIPELINE_DEBUG
                    $display("[%0t]  [decode] stage idle", $time);
                `endif
                ctrl_skip_next_reg <= 0;
            end
            ctrl_is_nop_reg <= is_nop;
        end else begin // stalled
            `ifdef PIPELINE_DEBUG
                $display(
                    "[%0t] =[decode] stage stalls on inst=0x%h pc=0x%h rs1=0x%h rs2=0x%h skip=%b",
                    $time, inst, pc, op1, op2, ctrl_skip_next_reg);
            `endif
            if (!ctrl_next_stage_stall) // insert a bubble
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

    output logic [31:0] res_reg,
    output logic [31:0] src_reg,
    output logic [4:0] rd_reg,
    output logic ctrl_is_nop_reg,
    output logic ctrl_wb_reg,
    output logic [4:0] ctrl_mem_reg,
    output [31:0] forward_data,
    output [4:0] ctrl_forward_rd,
    output ctrl_forward_valid,
    output ctrl_forward_mload_stall,
    output ctrl_executor_stall
);
    assign ctrl_executor_stall = 0;
    assign ctrl_forward_valid = ctrl_wb && (!ctrl_mem[1]);
    assign ctrl_forward_mload_stall = (!ctrl_is_nop) && ctrl_mem[1:0] == 'b10;
    assign ctrl_forward_rd = rd;
    // ALU
    wire [31:0] res = ctrl_alu_func == `FADD ? (ctrl_alu_sign_ext ? op1 - op2 : op1 + op2) :
               ctrl_alu_func == `FSLT ? ($signed(op1) < $signed(op2) ? 1 : 0) :
               ctrl_alu_func == `FSLTU ? (op1 < op2 ? 1 : 0) :
               ctrl_alu_func == `FXOR ? (op1 ^ op2) :
               ctrl_alu_func == `FOR ? (op1 | op2) :
               ctrl_alu_func == `FAND ? (op1 & op2) :
               ctrl_alu_func == `FSLL ? (op1 << op2[4:0]) :
               ctrl_alu_func == `FSRX ? (ctrl_alu_sign_ext ? $signed(op1) >> op2[4:0] : op1 >> op2[4:0]) : 'bx;
    assign forward_data = res;

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset)
            ctrl_is_nop_reg <= 1;
        else if (!ctrl_stall) begin
            if (!ctrl_is_nop) begin
                `ifdef PIPELINE_DEBUG
                    $display(
                        "[%0t] *[execution] stage works on op1=0x%h op2=0x%h src=0x%h rd=%d sign=%b",
                        $time, op1, op2, src, rd, ctrl_alu_sign_ext
                    );
                `endif
                res_reg <= res;
                src_reg <= src;
                rd_reg <= rd;
                ctrl_wb_reg <= ctrl_wb;
                ctrl_mem_reg <= ctrl_mem;
            end else begin
                `ifdef PIPELINE_DEBUG
                    $display("[%0t]  [execution] stage idle", $time);
                `endif
            end
            ctrl_is_nop_reg <= ctrl_is_nop;
        end else begin
            `ifdef PIPELINE_DEBUG
                $display(
                    "[%0t] =[execution] stage stalls on op1=0x%h op2=0x%h src=0x%h rd=%d sign=%b",
                    $time, op1, op2, src, rd, ctrl_alu_sign_ext
                );
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

    output logic [31:0] res_reg,
    output logic [4:0] rd_reg,
    output logic ctrl_is_nop_reg,
    output logic ctrl_wb_reg,
    output [31:0] forward_data,
    output [4:0] ctrl_forward_rd,
    output ctrl_forward_valid,
    output ctrl_mem_stall
);
    assign dcache_req = !ctrl_is_nop && ctrl_mem[1];
    assign dcache_wr = ctrl_mem[0];
    assign dcache_ws = ctrl_mem[3:2]; // SB/SH/SW
    assign dcache_wdata = src;
    assign dcache_addr = res_alu;

    assign ctrl_mem_stall = dcache_req && (!dcache_rdy);
    assign ctrl_forward_valid = ctrl_wb;
    assign ctrl_forward_rd = rd;

    wire sgn = ctrl_mem[4];

    wire [31:0] res_mem =
        dcache_ws == 'b00 ? {{24{sgn ? dcache_rdata[7] : 1'b0}}, dcache_rdata[7:0]} : // LB/LBU
        dcache_ws == 'b01 ? {{16{sgn ? dcache_rdata[15] : 1'b0}}, dcache_rdata[15:0]} : // LH/LHU
        dcache_ws == 'b10 ? dcache_rdata : // LW
                            'bx;
    wire [31:0] res = (ctrl_mem[1:0] == 2'b10) ? res_mem : res_alu;
    assign forward_data = res;

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset)
            ctrl_is_nop_reg <= 1;
        else if (!ctrl_stall) begin
            if (!ctrl_is_nop) begin
                `ifdef PIPELINE_DEBUG
                    $display(
                        "[%0t] *[memory] stage works on res_alu=0x%h src=0x%h rd=%d ctrl_wb=%b ctrl_mem=%5b",
                        $time, res_alu, src, rd, ctrl_wb, ctrl_mem
                    );
                `endif
                res_reg <= res;
                rd_reg <= rd;
                ctrl_wb_reg <= ctrl_wb;
            end else begin
                `ifdef PIPELINE_DEBUG
                    $display("[%0t]  [memory] stage idle", $time);
                `endif
            end
            ctrl_is_nop_reg <= ctrl_is_nop;
        end else begin
            `ifdef PIPELINE_DEBUG
                $display(
                    "[%0t] =[memory] stage stalls on res_alu=0x%h src=0x%h rd=%d ctrl_wb=%b ctrl_mem=%5b",
                    $time, res_alu, src, rd, ctrl_wb, ctrl_mem);
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
    output [31:0]_debug_pc_addr,

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

    wire [31:0] pc_jump_target;
    wire ctrl_is_jump;
    wire ctrl_fetcher_stall;
    wire ctrl_decoder_stall;
    wire ctrl_executor_stall;
    wire ctrl_mem_stall;
    wire ctrl_writeback_stall;
    wire [4:0] reg1_raddr;
    wire [4:0] reg2_raddr;
    wire [11:0] csr_addr;
    wire [31:0] reg1_rdata;
    wire [31:0] reg2_rdata;
    wire [31:0] csr_rdata;
    wire [31:0] csr_wdata;
    wire csr_wen;
    wire [4:0] reg_waddr;
    wire [31:0] reg_wdata;
    wire reg_wen;

    csr_file csv_reg(
        .addr(csr_addr),
        .wdata(csr_wdata),
        .wen(csr_wen),
        .rdata(csr_rdata),
        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_mie(),
        .ctrl_mpie()
    );

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
                                 ctrl_fetcher_stall || // because jumps could change PC
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

    wire [31:0] forward_data_exec;
    wire [4:0] ctrl_forward_rd_exec;
    wire ctrl_forward_valid_exec;
    wire ctrl_forward_mload_stall;

    wire [31:0] forward_data_mem;
    wire [4:0] ctrl_forward_rd_mem;
    wire ctrl_forward_valid_mem;

    decoder decode_stage(
        .inst(inst_reg_fetch),
        .pc(pc_reg_fetch),
        .reg1_raddr(reg1_raddr),
        .reg2_raddr(reg2_raddr),
        .csr_addr(csr_addr),
        .reg1_rdata(reg1_rdata),
        .reg2_rdata(reg2_rdata),
        .csr_rdata(csr_rdata),
        .csr_wdata(csr_wdata),
        .csr_wen(csr_wen),

        .forward_data_exec(forward_data_exec),
        .ctrl_forward_rd_exec(ctrl_forward_rd_exec),
        .ctrl_forward_valid_exec(ctrl_forward_valid_exec),
        .ctrl_forward_mload_stall(ctrl_forward_mload_stall),
        .forward_data_mem(forward_data_mem),
        .ctrl_forward_rd_mem(ctrl_forward_rd_mem),
        .ctrl_forward_valid_mem(ctrl_forward_valid_mem),

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
        .ctrl_decoder_stall(ctrl_decoder_stall),
        ._debug_pc_addr(_debug_pc_addr)
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
        .forward_data(forward_data_exec),
        .ctrl_forward_rd(ctrl_forward_rd_exec),
        .ctrl_forward_valid(ctrl_forward_valid_exec),
        .ctrl_forward_mload_stall(ctrl_forward_mload_stall),
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
        .forward_data(forward_data_mem),
        .ctrl_forward_rd(ctrl_forward_rd_mem),
        .ctrl_forward_valid(ctrl_forward_valid_mem),
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

`include "csr.sv"

`ifndef SYNTHESIS
    `ifdef DEBUG
        `define PIPELINE_DEBUG
    `endif
`endif

`define PC_RESET        32'h00100000
`define MTIME_ADDR      29'h400
`define MTIMECMP_ADDR   29'h401
`define MSIP_ADDR       30'h804
`define EIRQ_CTL_ADDR   30'h805

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
`define SYS     7'b1110011 // WFI, ECALL, EBREAK, CSRx
// NOTE: fence will be interpreted as a nop
`define FEN     7'b0001111 // FENCE

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

// funct12 code
`define ECALL  12'b000000000000
`define EBREAK 12'b000000000001
`define MRET   12'b001100000010
`define WFI    12'b000100000101

`define IRQ_M_SOFT		3  // Machine software interrupt
`define IRQ_M_TIMER		7  // Machine timer interrupt
`define IRQ_M_EXT		11 // Machine external interrupt

`define EXC_REG_INS_ILL    0
`define EXC_REG_INS_ALIGN  1
`define EXC_REG_M_ECALL    2
`define EXC_REG_M_EBKPT    3
`define EXC_REG_ST_ALIGN   4
`define EXC_REG_LD_ALIGN   5

`define EXC_INS_ILL    2
`define EXC_INS_ALIGN  0
`define EXC_M_ECALL    11
`define EXC_M_EBKPT    3
`define EXC_ST_ALIGN   6
`define EXC_LD_ALIGN   4

module register_file(
    input [4:0] raddr1,
    input [4:0] raddr2,
    output [31:0] rdata1,
    output [31:0] rdata2,

    input [4:0] waddr,
    input [31:0] wdata,
    input wen, //< write enable

    input ctrl_clk //< clock
);
    // register file
    logic [31:0] regs [31:0];
    wire writing = wen && waddr != 0;
    assign rdata1 = writing && waddr == raddr1 ? wdata : regs[raddr1];
    assign rdata2 = writing && waddr == raddr2 ? wdata : regs[raddr2];
    assign regs[0] = 0;
    always_ff @ (posedge ctrl_clk) begin
        if (writing) begin
            `ifdef PIPELINE_DEBUG
                $display(
                    "[%0t] ! REG  writes 0x%h to register %d",
                    $time, wdata, waddr
                );
            `endif
            regs[waddr] <= wdata;
        end
    end
endmodule

module exception_decode(input [5:0] eid, output [3:0] code);
    assign code = eid[0] ? `EXC_INS_ILL :
                  eid[1] ? `EXC_INS_ALIGN :
                  eid[2] ? `EXC_M_ECALL :
                  eid[3] ? `EXC_M_EBKPT :
                  eid[4] ? `EXC_ST_ALIGN :
                  eid[5] ? `EXC_LD_ALIGN : 4'bx;
endmodule

module interrupt_decode(input [2:0] eid, output [3:0] code);
    assign code = eid[0] ? `IRQ_M_EXT:
                  eid[1] ? `IRQ_M_SOFT:
                  eid[2] ? `IRQ_M_TIMER: 4'bx;
endmodule


module fetcher(
    input [31:0] pc_jump_target,
    input [31:0] pc_exc_target,

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input ctrl_jump,
    input ctrl_trap,

    // i-cache communication
    output [31:0] icache_addr,
    output icache_req,
    input [31:0] icache_data,
    input icache_rdy,

    output logic [31:0] inst_reg,
    output logic [31:0] pc_reg,
    output logic [5:0] exc_reg,
    output logic ctrl_nop_reg,
    output ctrl_fetcher_stall
);
    logic [31:0] pc;

    wire ins_aligned = pc[1:0] == 0;
    assign icache_addr = pc;
    assign icache_req = ins_aligned;
    assign ctrl_fetcher_stall = ins_aligned && !icache_rdy;
    wire [31:0] next_pc = ctrl_jump ? pc_jump_target : (pc + 4);
    `ifdef PIPELINE_DEBUG
        `define if_print_stat(m, prefix) \
            $display("[%0t] %s[IF ] %s on pc=0x%h (j=0x%h,%b;t=0x%h,%b)", \
                $time, m, prefix, pc, pc_jump_target, ctrl_jump, pc_exc_target, ctrl_trap)
    `endif
    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            `ifdef PIPELINE_DEBUG
                $display("[%0t] reset pc", $time);
            `endif
            pc <= `PC_RESET; // initialize PC
            ctrl_nop_reg <= 1;
        end else if (!ctrl_stall) begin
            `ifdef PIPELINE_DEBUG
                `if_print_stat("*", "works");
            `endif
            pc <= ctrl_trap ? pc_exc_target : next_pc;
            inst_reg <= icache_data;
            pc_reg <= pc;
            exc_reg <= {5'b0, !ins_aligned} << `EXC_REG_INS_ALIGN;
            ctrl_nop_reg <= 0;
        end else begin
            `ifdef PIPELINE_DEBUG
                `if_print_stat("=", "stalls");
            `endif
            if (!ctrl_next_stage_stall) // insert a bubble
                ctrl_nop_reg <= 1;
        end
    end
endmodule;

module decoder(
    input [31:0] inst,
    input [31:0] pc,
    input [5:0]  exc,

    input [31:0] reg_rdata1,
    input [31:0] reg_rdata2,
    input [31:0] csr_rdata,
    output [4:0] reg_raddr1,
    output [4:0] reg_raddr2,
    output [11:0] csr_raddr,

    // register forwarding
    input ctrl_forward_valid_exec,
    input [4:0] ctrl_forward_rd_exec,
    input [31:0] forward_data_exec,

    input ctrl_forward_valid_mem,
    input [4:0] ctrl_forward_rd_mem,
    input [31:0] forward_data_mem,
    input ctrl_forward_mload_stall,
    //
    
    // CSR forwarding
    input ctrl_forward_csr_valid_exec,
    input [11:0] ctrl_forward_csr_rd_exec,
    input [31:0] forward_csr_data_exec,

    input ctrl_forward_csr_valid_mem,
    input [11:0] ctrl_forward_csr_rd_mem,
    input [31:0] forward_csr_data_mem,
    // no stall as CSR instructions do not access memory
    //

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_trap,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input ctrl_nop,
    input ctrl_csr_addr_valid,

    // stage output data
    output logic [31:0] op1_reg,    // op1 for ALU
    output logic [31:0] op2_reg,    // op2 for ALU
    output logic [4:0] rd_reg,      // destination general register for writeback
    output logic [11:0] rd_csr_reg, // destination CSR register for writeback
    output logic [31:0] tmp_reg,    // keeps some additional value that skips ALU
    output logic [31:0] pc_reg,     // pass on the instruction address
    output logic [5:0] exc_reg,     // exceptions

    output logic [2:0] ctrl_alu_func_reg,
    output logic ctrl_alu_sign_ext_reg,
    output logic ctrl_nop_reg,
    output logic ctrl_wb_reg,
    output logic ctrl_wb_csr_reg,
    output logic ctrl_mret_reg,
    output logic ctrl_wfi_reg,
    output logic [4:0] ctrl_mem_reg,
    output [31:0] ctrl_pc_jump_target,
    output ctrl_jump,
    output ctrl_decoder_stall
);
    logic ctrl_skip_next_reg;

    wire [6:0] opcode = inst[6:0];
    wire [2:0] funct3 = inst[14:12];
    wire [11:0] funct12 = inst[31:20];
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
    wire is_wfi = opcode == `SYS && funct3 == 0 && funct12 == `WFI;
    wire is_nop = ctrl_nop ||
                  ctrl_skip_next_reg ||
                  ctrl_trap ||
                  (opcode == `XXXI && inst[31:7] == 0) ||
                  opcode == `FEN;

    assign ctrl_decoder_stall = ctrl_forward_mload_stall &&
                                (((opcode == `XXXI ||
                                  opcode == `XXX ||
                                  opcode == `LX ||
                                  opcode == `SX ||
                                  opcode == `JALR ||
                                  opcode == `BXX ||
                                  opcode == `SYS) && ctrl_forward_rd_exec == rs1) ||
                                ((opcode == `XXX ||
                                  opcode == `SX ||
                                  opcode == `JALR ||
                                  opcode == `BXX) && ctrl_forward_rd_exec == rs2));

    // wire to read from register file
    assign reg_raddr1 = rs1;
    assign reg_raddr2 = rs2;

    wire [31:0] op1 = (ctrl_forward_valid_exec && rs1 == ctrl_forward_rd_exec) ? forward_data_exec:
                      (ctrl_forward_valid_mem && rs1 == ctrl_forward_rd_mem) ? forward_data_mem:
                                                                               reg_rdata1;
    wire [31:0] op2 = (ctrl_forward_valid_exec && rs2 == ctrl_forward_rd_exec) ? forward_data_exec:
                      (ctrl_forward_valid_mem && rs2 == ctrl_forward_rd_mem) ? forward_data_mem:
                                                                               reg_rdata2;
    // CSR
    wire mret = funct3 == 0 && funct12 == `MRET;
    wire [5:0] sys_exc = funct3 != 0 ? ({5'b0, !ctrl_csr_addr_valid} << `EXC_REG_INS_ILL) : (
        funct12 == `MRET ? 0 :
        funct12 == `WFI ? 0 :
        funct12 == `ECALL ? (1 << `EXC_REG_M_ECALL) :
        funct12 == `EBREAK ? (1 << `EXC_REG_M_EBKPT) :
                             (1 << `EXC_REG_INS_ILL));

    assign csr_raddr = csr;
    wire [31:0] csr_op = (ctrl_forward_csr_valid_exec && csr == ctrl_forward_csr_rd_exec) ? forward_csr_data_exec:
                         (ctrl_forward_csr_valid_mem && csr == ctrl_forward_csr_rd_mem) ? forward_csr_data_mem:
                                                                                          csr_rdata;
    wire [1:0] csr_func = funct3[1:0];
    wire [31:0] csr_tmp = funct3[2] ? {27'b0, rs1} : op1;

    // set jump signal for control transfer instructions
    assign ctrl_pc_jump_target =
        opcode == `JAL ? pc + $signed({{11{jal_offset[20]}}, jal_offset}) :
        opcode == `JALR ? op1 + $signed({{20{jalr_offset[11]}}, jalr_offset}) :
        opcode == `BXX ? pc + $signed({{19{b_offset[12]}}, b_offset}) : 'bx;
    assign ctrl_jump = (!ctrl_skip_next_reg) &&
        (opcode == `JAL ? 1 :
        opcode == `JALR ? 1 :
        opcode == `BXX ?
            (funct3[2:1] == 0 ? ((op1 == op2) ^ (funct3[0])) : // BEQ & BNE
             (funct3[1] == 0 ?
                  (($signed(op1) < $signed(op2)) ^ (funct3[0])) : // BLT & BGE
                  ((op1 < op2) ^ (funct3[0])))) : // BLTU & BGEU
                  0);

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            ctrl_skip_next_reg <= 0;
            ctrl_nop_reg <= 1;
        end else if (!ctrl_stall) begin
            `ifdef PIPELINE_DEBUG
                `define id_print_stat(m, prefix) \
                    $display(\
                        "[%0t] %s[ID ] %s on ",  $time, m, prefix, \
                        "pc=0x%h exc=%b op1=0x%h op2=0x%h csr_op=0x%h rs1=%0d rs2=%0d "
                        pc, exc, op1, op2, csr_op, rs1, rs2, \
                        "fwd=(0x%h,%0d,%b;0x%h,%0d,%b;0x%h,0x%h,%b;0x%h,0x%h,%b) rd=%0d skip=%b", \
                        forward_data_exec, ctrl_forward_rd_exec, ctrl_forward_valid_exec, \
                        forward_data_mem, ctrl_forward_rd_mem, ctrl_forward_valid_mem,\
                        forward_csr_data_exec, ctrl_forward_csr_rd_exec, ctrl_forward_csr_valid_exec, \
                        forward_csr_data_mem, ctrl_forward_csr_rd_mem, ctrl_forward_csr_valid_mem, \
                        inst, rd, ctrl_skip_next_reg)
            `endif

            if (!is_nop) begin
                `ifdef PIPELINE_DEBUG
                    `id_print_stat("*", "works");
                `endif
                case (opcode)
                    `XXXI: begin
                        op1_reg <= op1;
                        op2_reg <= xxxi;
                        exc_reg <= exc;
                        ctrl_alu_func_reg <= funct3;
                        ctrl_alu_sign_ext_reg <= (funct3 == `FSRX) && inst[30];
                        ctrl_wb_reg <= 1;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= 0;
                    end
                    `XXX: begin
                        op1_reg <= op1;
                        op2_reg <= op2;
                        exc_reg <= exc;
                        ctrl_alu_func_reg <= funct3;
                        ctrl_alu_sign_ext_reg <= inst[30];
                        ctrl_wb_reg <= 1;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= 0;
                    end
                    `LUI: begin
                        op1_reg <= ui;
                        op2_reg <= 0;
                        exc_reg <= exc;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= 0;
                    end
                    `AUIPC: begin
                        op1_reg <= ui;
                        op2_reg <= pc;
                        exc_reg <= exc;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= 0;
                    end
                    `LX: begin
                        op1_reg <= op1;
                        op2_reg <= {{20{l_offset[11]}}, l_offset};
                        exc_reg <= exc | ({5'b0, funct3[1:0] == 2'b11} << `EXC_REG_INS_ILL);
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= {funct3, 2'b10};
                    end
                    `SX: begin
                        op1_reg <= op1;
                        op2_reg <= {{20{s_offset[11]}}, s_offset};
                        exc_reg <= exc | ({5'b0, funct3[1:0] == 2'b11} << `EXC_REG_INS_ILL);
                        tmp_reg <= op2;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 0;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= {funct3, 2'b11};
                    end
                    `JAL, `JALR: begin
                        op1_reg <= pc;
                        op2_reg <= 4;
                        exc_reg <= exc;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= 1;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= 0; //{funct3, 2'b00};
                    end
                    `SYS: begin
                        op1_reg <= csr_op;
                        op2_reg <= 0;
                        exc_reg <= exc | sys_exc;
                        rd_csr_reg <= csr;
                        ctrl_alu_func_reg <= `FADD;
                        ctrl_alu_sign_ext_reg <= 0;
                        ctrl_wb_reg <= funct3 != 0; // all CSR instructions writes to rd
                        ctrl_wb_csr_reg <= csr_func != 0 && rs1 != 0;
                        ctrl_mem_reg <= 0;
                        // CSR
                        tmp_reg <=
                            csr_func == `CSRRW ? csr_tmp :
                            csr_func == `CSRRS ? (csr_rdata | csr_tmp) :
                            csr_func == `CSRRC ? (csr_rdata & (~csr_tmp)) : 'bx;
                    end
                    `BXX: begin
                        exc_reg <= exc;
                        ctrl_wb_reg <= 0;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= 0;
                    end
                    default: begin // WIP: illegal instruction detection
                        exc_reg <= exc | (1 << `EXC_REG_INS_ILL);
                        ctrl_wb_reg <= 0;
                        ctrl_wb_csr_reg <= 0;
                        ctrl_mem_reg <= 0;
                    end
                endcase
                rd_reg <= rd;
                pc_reg <= pc;
                ctrl_mret_reg <= opcode == `SYS && mret;
                ctrl_wfi_reg <= is_wfi;
                ctrl_skip_next_reg <= ctrl_jump;
            end else begin // NOP or a bubble
                `ifdef PIPELINE_DEBUG
                    $display("[%0t]  [ID ] idle", $time);
                `endif
                ctrl_skip_next_reg <= ctrl_trap;
            end
            ctrl_nop_reg <= is_nop;
        end else begin // stalled
            `ifdef PIPELINE_DEBUG
                `id_print_stat("=", "stalls");
            `endif
            if (!ctrl_next_stage_stall) // insert a bubble
                ctrl_nop_reg <= 1;
        end
    end
endmodule

module executor(
    input [31:0] op1,
    input [31:0] op2,
    input [4:0] rd,
    input [11:0] rd_csr,
    input [31:0] tmp,
    input [31:0] pc,
    input [5:0] exc,

    input ctrl_clk,
    input ctrl_reset,
    input ctrl_trap,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input [2:0] ctrl_alu_func,
    input ctrl_alu_sign_ext,
    input ctrl_nop,
    input ctrl_wb,
    input ctrl_wb_csr,
    input ctrl_mret,
    input ctrl_wfi,
    input [4:0] ctrl_mem,

    // stage output data
    output logic [31:0] res_reg,    // the result from ALU
    output logic [4:0] rd_reg,
    output logic [11:0] rd_csr_reg,
    output logic [31:0] tmp_reg,
    output logic [31:0] pc_reg,
    output logic [5:0] exc_reg,

    output logic ctrl_nop_reg,
    output logic ctrl_wb_reg,
    output logic ctrl_wb_csr_reg,
    output logic ctrl_mret_reg,
    output logic ctrl_wfi_reg,
    output logic [4:0] ctrl_mem_reg,

    output ctrl_forward_valid,
    output [4:0] ctrl_forward_rd,
    output [31:0] forward_data,
    output ctrl_forward_mload_stall,

    output ctrl_forward_csr_valid,
    output [11:0] ctrl_forward_csr_rd,
    output [31:0] forward_csr_data,

    output ctrl_executor_stall
);
    wire is_nop = ctrl_nop || ctrl_trap;
    assign ctrl_executor_stall = 0;
    // the second clause is for LX (has to wait after memory load, and ALU's result should be ignored)
    assign ctrl_forward_valid = (!is_nop) && ctrl_wb && (!ctrl_mem[1]) && rd != 0;
    assign ctrl_forward_mload_stall = (!is_nop) && ctrl_mem[1:0] == 2'b10;
    assign ctrl_forward_rd = rd;
    assign forward_data = res;

    assign ctrl_forward_csr_valid = (!is_nop) && ctrl_wb_csr;
    assign ctrl_forward_csr_rd = rd_csr;
    assign forward_csr_data = tmp;

    // ALU
    wire [31:0] res = ctrl_alu_func == `FADD ? (ctrl_alu_sign_ext ? op1 - op2 : op1 + op2) :
               ctrl_alu_func == `FSLT ? ($signed(op1) < $signed(op2) ? 1 : 0) :
               ctrl_alu_func == `FSLTU ? (op1 < op2 ? 1 : 0) :
               ctrl_alu_func == `FXOR ? (op1 ^ op2) :
               ctrl_alu_func == `FOR ? (op1 | op2) :
               ctrl_alu_func == `FAND ? (op1 & op2) :
               ctrl_alu_func == `FSLL ? (op1 << op2[4:0]) :
               ctrl_alu_func == `FSRX ? (ctrl_alu_sign_ext ? $signed(op1) >> op2[4:0] : op1 >> op2[4:0]) : 'bx;

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset)
            ctrl_nop_reg <= 1;
        else if (!ctrl_stall) begin
            `ifdef PIPELINE_DEBUG
                `define ex_print_stat(m, prefix) \
                    $display( \
                        "[%0t] %s[EX ] %s on ", $time, m, prefix, \
                        "pc=0x%h exc=%b op1=0x%h op2=0x%h res=0x%h tmp=0x%h rd=%0d sign=%b", \
                        pc, exc, op1, op2, res, tmp, rd, ctrl_alu_sign_ext)

            `endif
            if (!is_nop) begin
                `ifdef PIPELINE_DEBUG
                    `ex_print_stat("*", "works");
                `endif
                res_reg <= res;
                rd_reg <= rd;
                rd_csr_reg <= rd_csr;
                tmp_reg <= tmp;
                pc_reg <= pc;
                exc_reg <= exc;
                ctrl_wb_reg <= ctrl_wb;
                ctrl_wb_csr_reg <= ctrl_wb_csr;
                ctrl_mret_reg <= ctrl_mret;
                ctrl_wfi_reg <= ctrl_wfi;
                ctrl_mem_reg <= ctrl_mem;
            end else begin
                `ifdef PIPELINE_DEBUG
                    $display("[%0t]  [EX ] idle", $time);
                `endif
            end
            ctrl_nop_reg <= is_nop;
        end else begin
            `ifdef PIPELINE_DEBUG
                `ex_print_stat("=", "stalls");
            `endif
            if (!ctrl_next_stage_stall)
                ctrl_nop_reg <= 1;
        end
    end
endmodule

module memory(
    input [31:0] res_alu,
    input [4:0] rd,
    input [11:0] rd_csr,
    input [31:0] tmp,
    input [31:0] pc,
    input [5:0] exc,

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
    input ctrl_trap,
    input ctrl_stall,
    input ctrl_next_stage_stall,
    input ctrl_wb,
    input ctrl_wb_csr,
    input ctrl_mret,
    input ctrl_wfi,
    input ctrl_nop,
    input ctrl_ext_irq_trigger,
    input [4:0] ctrl_mem,

    // statge output data
    output logic [31:0] res_reg,
    output logic [4:0] rd_reg,
    output logic [11:0] rd_csr_reg,
    output logic [31:0] tmp_reg,
    output logic [31:0] pc_reg,
    output logic [5:0] exc_reg,

    output logic ctrl_nop_reg,
    output logic ctrl_wb_reg,
    output logic ctrl_wb_csr_reg,
    output logic ctrl_mret_reg,
    output logic ctrl_wfi_reg,

    output ctrl_forward_valid,
    output [4:0] ctrl_forward_rd,
    output [31:0] forward_data,

    output ctrl_forward_csr_valid,
    output [11:0] ctrl_forward_csr_rd,
    output [31:0] forward_csr_data,

    output ctrl_ext_irq,
    output ctrl_sw_irq,
    output ctrl_timer_irq,
    output ctrl_mem_stall
);
    wire is_nop = ctrl_nop || ctrl_trap;

    logic [63:0] mtime_regs [1:0];
    logic meip;
    logic msip;
    wire meip_clear = (!ctrl_nop) && valid && dcache_wr &&
                      (mreg_addr == 'b111 && dcache_wdata != 0);
    assign ctrl_ext_irq = (meip_clear ? 0 : meip) || ctrl_ext_irq_trigger;
    assign ctrl_sw_irq = msip;

    wire [2:0] mreg_addr = dcache_addr[31:3] == `MTIME_ADDR ?    'b001 :
                           dcache_addr[31:3] == `MTIMECMP_ADDR ? 'b101 :
                           dcache_addr[31:2] == `MSIP_ADDR ?     'b011 :
                           dcache_addr[31:2] == `EIRQ_CTL_ADDR ? 'b111 : 0;
    assign ctrl_timer_irq = mtime_regs[0] >= mtime_regs[1];

    wire aligned = dcache_ws == 2'b01 ? (dcache_addr[0] == 0) :
                   dcache_ws == 2'b10 ? (dcache_addr[1:0] == 0) : 1;
    wire valid = ctrl_mem[1] && exc == 0 && aligned;
    assign dcache_req = (!is_nop) && valid && (!mreg_addr[0]);
    assign dcache_wr = ctrl_mem[0];
    assign dcache_ws = ctrl_mem[3:2]; // SB/SH/SW
    assign dcache_wdata = tmp;
    assign dcache_addr = res_alu;

    assign ctrl_mem_stall = dcache_req && (!dcache_rdy);

    assign ctrl_forward_valid = (!is_nop) && ctrl_wb && rd != 0;
    assign ctrl_forward_rd = rd;
    assign forward_data = res;

    assign ctrl_forward_csr_valid = (!is_nop) && ctrl_wb_csr;
    assign ctrl_forward_csr_rd = rd_csr;
    assign forward_csr_data = tmp;

    wire sgn = ctrl_mem[4];

    wire [5:0] mreg_off = {dcache_addr[2:0], 3'b0};
    wire [31:0] rdata = mreg_addr[0] ?
                        (mreg_addr[1] == 0 ? mtime_regs[mreg_addr[2]][(mreg_off + 31) -: 32] : // mtime/mtimecmp
                                             (mreg_addr[2] ? 0 : {31'b0, msip})) : // msip
                        dcache_rdata;

    wire [31:0] res_mem =
        dcache_ws == 2'b00 ? {{24{sgn ? rdata[7] : 1'b0}}, rdata[7:0]} : // LB/LBU
        dcache_ws == 2'b01 ? {{16{sgn ? rdata[15] : 1'b0}}, rdata[15:0]} : // LH/LHU
                             rdata; // LW
    wire [31:0] res = (ctrl_mem[1:0] == 2'b10) ? res_mem : res_alu;

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            ctrl_nop_reg <= 1;
            mtime_regs[0] <= 0;
            meip <= 0;
            msip <= 0;
        end
        else if (!ctrl_stall) begin
            `ifdef PIPELINE_DEBUG
                `define mem_print_stat(m, prefix) \
                    $display("[%0t] %s[MEM] %s on ", $time, m, prefix, \
                            "pc=0x%h, res_alu=0x%h tmp=0x%h rd=%0d ctrl_wb=(%b,%b) ctrl_mem=%5b mtime=%h mtimecmp=%h", \
                            pc, res_alu, tmp, rd, ctrl_wb, ctrl_wb_csr, ctrl_mem, mtime_regs[0], mtime_regs[1])
            `endif
            if (!is_nop) begin
                `ifdef PIPELINE_DEBUG
                    `mem_print_stat("*", "works");
                `endif
                if (mreg_addr[0] && valid && dcache_wr) begin
                    `ifdef PIPELINE_DEBUG
                        $display(
                            "[%0t] !MREG  writes 0x%h to register %b (w=%0d)",
                            $time, dcache_wdata, mreg_addr, dcache_ws
                        );
                    `endif
                    if (!mreg_addr[1]) begin
                        case (dcache_ws)
                            'b00: mtime_regs[mreg_addr[2]][(mreg_off + 7) -: 8] <= dcache_wdata[7:0];
                            'b01: mtime_regs[mreg_addr[2]][(mreg_off + 15) -: 16] <= dcache_wdata[15:0];
                            'b10: mtime_regs[mreg_addr[2]][(mreg_off + 31) -: 32] <= dcache_wdata[31:0];
                        endcase
                    end else begin
                        case (mreg_addr[2])
                            'b0: msip <= dcache_wdata[0];
                            'b1:;
                        endcase
                    end
                end
                res_reg <= res;
                rd_reg <= rd;
                rd_csr_reg <= rd_csr;
                tmp_reg <= tmp;
                pc_reg <= pc;
                exc_reg <= exc | ({6{ctrl_mem[1]}} & (
                    {5'b0, !aligned} << (dcache_wr ? `EXC_ST_ALIGN : `EXC_LD_ALIGN)));
                ctrl_wb_reg <= ctrl_wb;
                ctrl_wb_csr_reg <= ctrl_wb_csr;
                ctrl_mret_reg <= ctrl_mret;
                ctrl_wfi_reg <= ctrl_wfi;
            end else begin
                `ifdef PIPELINE_DEBUG
                    $display("[%0t]  [MEM] idle", $time);
                `endif
            end
            ctrl_nop_reg <= is_nop;
        end else begin
            `ifdef PIPELINE_DEBUG
                `mem_print_stat("=", "stalls");
            `endif
            if (!ctrl_next_stage_stall)
                ctrl_nop_reg <= 1;
        end
        if (is_nop || !(mreg_addr == 'b001 && valid && dcache_wr))
            mtime_regs[0] <= mtime_regs[0] + 1; // tick the timer
        meip <= ctrl_ext_irq;
    end
endmodule

module writeback(
    input [31:0] res,
    input [4:0] rd,
    input [11:0] rd_csr,
    input [31:0] tmp,
    input [31:0] pc,
    input [5:0] exc,

    input ctrl_clk,
    input ctrl_stall,
    input ctrl_wb,
    input ctrl_wb_csr,
    input ctrl_mret,
    input ctrl_wfi,
    input ctrl_nop,

    input ctrl_ext_irq,
    input ctrl_sw_irq,
    input ctrl_timer_irq,
    input ctrl_mie,
    input [2:0] ctrl_mxie,

    // Reg file
    output [4:0] reg_waddr,
    output [31:0] reg_wdata,
    output reg_wen,

    // CSR
    output [11:0] csr_raddr,
    input [31:0] csr_rdata,
    output [11:0] csr_waddr,
    output [31:0] csr_wdata,
    output csr_wen,
    output [31:0] csr_trap_pc,
    output [4:0] csr_trap_info,

    output ctrl_writeback_stall,
    output ctrl_trap,
    output [31:0] ctrl_pc_exc_target,
    output ctrl_wfi_stall,
    output [2:0] ctrl_mxip
);
    assign ctrl_mxip = {
        ctrl_timer_irq,
        ctrl_sw_irq,
        ctrl_ext_irq
    };

    wire is_exc = (!ctrl_nop) && exc != 0;
    wire is_mret = (!ctrl_nop) && ctrl_mret;
    wire is_trap = is_exc || ((ctrl_mxie & ctrl_mxip) != 0 && ctrl_mie);
    wire valid = (!ctrl_nop) && (!ctrl_stall) && exc == 0;
    // write back registers
    assign reg_waddr = rd;
    assign reg_wdata = res;
    assign reg_wen = valid && ctrl_wb;
    assign ctrl_writeback_stall = 0;

    assign csr_waddr = rd_csr;
    assign csr_wdata = tmp;
    assign csr_wen = valid && ctrl_wb_csr;

    // If there is an exception:
    // 1. change PC of fetcher for the next cycle according to mtvec.
    // 2. mark all instructions in previous stages as NOP.
    // 3. push a special flag to writeback stage (so it writes back all CSR changes in the next cycle).
    exception_decode _exc_decode(exc, ecause);
    interrupt_decode _irq_decode(ctrl_mxie & ctrl_mxip, icause);
    wire [31:0] mtvec_base = {csr_rdata[31:2], 2'b00};
    wire [3:0] icause;
    wire [3:0] ecause;
    wire [3:0] cause = is_exc ? ecause : icause;
    assign csr_raddr = is_trap ? `CSR_MTVEC : `CSR_MEPC;
    assign csr_trap_pc = pc;
    assign csr_trap_info = {!is_exc, cause};
    assign ctrl_pc_exc_target = is_trap ? (csr_rdata[0] ? (mtvec_base + {26'b0, cause, 2'b0}) : mtvec_base) :
                                          (csr_rdata + 4);
    assign ctrl_trap = is_trap || is_mret; // traps and mret will clear the pipeline
    assign ctrl_wfi_stall = (!ctrl_nop) && ctrl_wfi && (!is_trap);
    `ifdef PIPELINE_DEBUG
        `define wb_print_stat(m, prefix) \
            $display("[%0t] %s[WB ] %s on ", $time, m, prefix, \
                    "pc=0x%h res=0x%h tmp=0x%h e=0x%h exc=%b rd=%0d rd_csr=0x%h mxie=%b mxip=%b mret=%b", \
                    pc, res, tmp, ctrl_pc_exc_target, exc, rd, rd_csr, ctrl_mxie, ctrl_mxip, ctrl_mret \
            )
        always_ff @ (posedge ctrl_clk) begin
            if (!ctrl_nop)
                if (!ctrl_stall)
                    `wb_print_stat("*", "works");
                else
                    `wb_print_stat("=", "stalls");
            else
                $display("[%0t] %s[WB ] idle is_trap=%b", $time, ctrl_wfi_stall ? "w" : " ", is_trap);
        end
    `endif
endmodule

module core (
    input clock,
    input reset,
    input irq,
    output [31:0] _debug_pc,

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
    wire [31:0] pc_exc_target;
    wire ctrl_jump;
    wire ctrl_trap;
    wire ctrl_fetcher_stall;
    wire ctrl_decoder_stall;
    wire ctrl_executor_stall;
    wire ctrl_mem_stall;
    wire ctrl_writeback_stall;
    wire ctrl_ext_irq;
    wire ctrl_sw_irq;
    wire ctrl_timer_irq;
    wire ctrl_mie;
    wire [2:0] ctrl_mxie;
    wire [2:0] ctrl_mxip;

    wire [4:0] reg_raddr1;
    wire [4:0] reg_raddr2;
    wire [31:0] reg_rdata1;
    wire [31:0] reg_rdata2;
    wire [4:0] reg_waddr;
    wire [31:0] reg_wdata;
    wire reg_wen;

    wire [11:0] csr_raddr1;
    wire [11:0] csr_raddr2;
    wire [31:0] csr_rdata1;
    wire [31:0] csr_rdata2;
    wire [11:0] csr_waddr;
    wire [31:0] csr_wdata;
    wire csr_wen;
    wire [31:0] csr_trap_pc;
    wire [4:0] csr_trap_info;
    wire ctrl_csr_addr_valid;

    csr csr_reg(
        .raddr1(csr_raddr1),
        .raddr2(csr_raddr2),
        .rdata1(csr_rdata1),
        .rdata2(csr_rdata2),
        .waddr(csr_waddr),
        .wdata(csr_wdata),
        .wen(csr_wen),
        .trap_pc(csr_trap_pc),
        .trap_info(csr_trap_info),
        .ctrl_mxip(ctrl_mxip),

        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_trap(ctrl_trap),
        .ctrl_mret((!ctrl_nop_mem_o) && ctrl_mret_mem_o),
        .ctrl_mie(ctrl_mie),
        .ctrl_mxie(ctrl_mxie),
        .ctrl_addr_valid(ctrl_csr_addr_valid)
    );

    register_file main_reg(
        .raddr1(reg_raddr1),
        .raddr2(reg_raddr2),
        .rdata1(reg_rdata1),
        .rdata2(reg_rdata2),
        .waddr(reg_waddr),
        .wdata(reg_wdata),
        .wen(reg_wen),
        .ctrl_clk(clock)
    );

    wire ctrl_wfi_stall;

    wire ctrl_fetcher_stall_in = ctrl_fetcher_stall ||
                                 ctrl_decoder_stall ||
                                 ctrl_executor_stall ||
                                 ctrl_mem_stall ||
                                 ctrl_writeback_stall ||
                                 ctrl_wfi_stall;

    wire ctrl_decoder_stall_in = ctrl_decoder_stall ||
                                 ctrl_fetcher_stall || // because jumps could change PC
                                 ctrl_executor_stall ||
                                 ctrl_mem_stall ||
                                 ctrl_writeback_stall ||
                                 ctrl_wfi_stall;

    wire ctrl_executor_stall_in = ctrl_executor_stall ||
                                  ctrl_mem_stall ||
                                  ctrl_writeback_stall ||
                                  ctrl_wfi_stall;

    wire ctrl_mem_stall_in = ctrl_mem_stall ||
                             ctrl_writeback_stall ||
                             ctrl_wfi_stall;

    wire ctrl_wb_stall_in = ctrl_writeback_stall ||
                            ctrl_wfi_stall;

    wire [31:0] inst_if_o;
    wire [31:0] pc_if_o;
    wire [5:0] exc_if_o;
    wire ctrl_nop_if_o;

    fetcher if_stage(
        .pc_jump_target(pc_jump_target),
        .pc_exc_target(pc_exc_target),

        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_stall(ctrl_fetcher_stall_in),
        .ctrl_next_stage_stall(ctrl_decoder_stall_in),
        .ctrl_jump(ctrl_jump),
        .ctrl_trap(ctrl_trap),

        .icache_addr(icache_addr),
        .icache_req(icache_req),
        .icache_data(icache_data),
        .icache_rdy(icache_rdy),

        .inst_reg(inst_if_o),
        .pc_reg(pc_if_o),
        .exc_reg(exc_if_o),

        .ctrl_nop_reg(ctrl_nop_if_o),
        .ctrl_fetcher_stall(ctrl_fetcher_stall)
    );

    wire [31:0] op1_id_o;
    wire [31:0] op2_id_o;
    wire [4:0] rd_id_o;
    wire [11:0] rd_csr_id_o;
    wire [31:0] tmp_id_o;
    wire [31:0] pc_id_o;
    wire [5:0] exc_id_o;

    wire [2:0] ctrl_alu_func_id_o;
    wire ctrl_alu_sign_ext_id_o;
    wire ctrl_nop_id_o;
    wire ctrl_wb_id_o;
    wire ctrl_wb_csr_id_o;
    wire ctrl_mret_id_o;
    wire ctrl_wfi_id_o;
    wire [4:0] ctrl_mem_id_o;

    wire ctrl_forward_valid_exec;
    wire [4:0] ctrl_forward_rd_exec;
    wire [31:0] forward_data_exec;
    wire ctrl_forward_mload_stall;

    wire ctrl_forward_valid_mem;
    wire [4:0] ctrl_forward_rd_mem;
    wire [31:0] forward_data_mem;

    wire ctrl_forward_csr_valid_exec;
    wire [11:0] ctrl_forward_csr_rd_exec;
    wire [31:0] forward_csr_data_exec;

    wire ctrl_forward_csr_valid_mem;
    wire [11:0] ctrl_forward_csr_rd_mem;
    wire [31:0] forward_csr_data_mem;

    decoder id_stage(
        .inst(inst_if_o),
        .pc(pc_if_o),
        .exc(exc_if_o),

        .reg_rdata1(reg_rdata1),
        .reg_rdata2(reg_rdata2),
        .csr_rdata(csr_rdata1),
        .reg_raddr1(reg_raddr1),
        .reg_raddr2(reg_raddr2),
        .csr_raddr(csr_raddr1),

        .ctrl_forward_valid_exec(ctrl_forward_valid_exec),
        .ctrl_forward_rd_exec(ctrl_forward_rd_exec),
        .forward_data_exec(forward_data_exec),

        .ctrl_forward_valid_mem(ctrl_forward_valid_mem),
        .ctrl_forward_rd_mem(ctrl_forward_rd_mem),
        .forward_data_mem(forward_data_mem),
        .ctrl_forward_mload_stall(ctrl_forward_mload_stall),

        .ctrl_forward_csr_valid_exec(ctrl_forward_csr_valid_exec),
        .ctrl_forward_csr_rd_exec(ctrl_forward_csr_rd_exec),
        .forward_csr_data_exec(forward_csr_data_exec),

        .ctrl_forward_csr_valid_mem(ctrl_forward_csr_valid_mem),
        .ctrl_forward_csr_rd_mem(ctrl_forward_csr_rd_mem),
        .forward_csr_data_mem(forward_csr_data_mem),

        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_trap(ctrl_trap),
        .ctrl_stall(ctrl_decoder_stall_in),
        .ctrl_next_stage_stall(ctrl_executor_stall_in),
        .ctrl_nop(ctrl_nop_if_o),
        .ctrl_csr_addr_valid(ctrl_csr_addr_valid),

        .op1_reg(op1_id_o),
        .op2_reg(op2_id_o),
        .rd_reg(rd_id_o),
        .rd_csr_reg(rd_csr_id_o),
        .tmp_reg(tmp_id_o),
        .pc_reg(pc_id_o),
        .exc_reg(exc_id_o),

        .ctrl_alu_func_reg(ctrl_alu_func_id_o),
        .ctrl_alu_sign_ext_reg(ctrl_alu_sign_ext_id_o),
        .ctrl_nop_reg(ctrl_nop_id_o),
        .ctrl_wb_reg(ctrl_wb_id_o),
        .ctrl_wb_csr_reg(ctrl_wb_csr_id_o),
        .ctrl_mret_reg(ctrl_mret_id_o),
        .ctrl_wfi_reg(ctrl_wfi_id_o),
        .ctrl_mem_reg(ctrl_mem_id_o),
        .ctrl_pc_jump_target(pc_jump_target),
        .ctrl_jump(ctrl_jump),
        .ctrl_decoder_stall(ctrl_decoder_stall)
    );

    wire [31:0] res_ex_o;
    wire [4:0] rd_ex_o;
    wire [11:0] rd_csr_ex_o;
    wire [31:0] tmp_ex_o;
    wire [31:0] pc_ex_o;
    wire [5:0] exc_ex_o;
    wire ctrl_nop_ex_o;
    wire ctrl_wb_ex_o;
    wire ctrl_wb_csr_ex_o;
    wire ctrl_mret_ex_o;
    wire ctrl_wfi_ex_o;
    wire [4:0] ctrl_mem_ex_o;

    executor ex_stage(
        .op1(op1_id_o),
        .op2(op2_id_o),
        .rd(rd_id_o),
        .rd_csr(rd_csr_id_o),
        .tmp(tmp_id_o),
        .pc(pc_id_o),
        .exc(exc_id_o),

        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_trap(ctrl_trap),
        .ctrl_stall(ctrl_executor_stall_in),
        .ctrl_next_stage_stall(ctrl_mem_stall_in),
        .ctrl_alu_func(ctrl_alu_func_id_o),
        .ctrl_alu_sign_ext(ctrl_alu_sign_ext_id_o),
        .ctrl_nop(ctrl_nop_id_o),
        .ctrl_wb(ctrl_wb_id_o),
        .ctrl_wb_csr(ctrl_wb_csr_id_o),
        .ctrl_mret(ctrl_mret_id_o),
        .ctrl_wfi(ctrl_wfi_id_o),
        .ctrl_mem(ctrl_mem_id_o),

        .res_reg(res_ex_o),
        .rd_reg(rd_ex_o),
        .rd_csr_reg(rd_csr_ex_o),
        .tmp_reg(tmp_ex_o),
        .pc_reg(pc_ex_o),
        .exc_reg(exc_ex_o),

        .ctrl_nop_reg(ctrl_nop_ex_o),
        .ctrl_wb_reg(ctrl_wb_ex_o),
        .ctrl_wb_csr_reg(ctrl_wb_csr_ex_o),
        .ctrl_mret_reg(ctrl_mret_ex_o),
        .ctrl_wfi_reg(ctrl_wfi_ex_o),
        .ctrl_mem_reg(ctrl_mem_ex_o),

        .ctrl_forward_valid(ctrl_forward_valid_exec),
        .ctrl_forward_rd(ctrl_forward_rd_exec),
        .forward_data(forward_data_exec),
        .ctrl_forward_mload_stall(ctrl_forward_mload_stall),

        .ctrl_forward_csr_valid(ctrl_forward_csr_valid_exec),
        .ctrl_forward_csr_rd(ctrl_forward_csr_rd_exec),
        .forward_csr_data(forward_csr_data_exec),

        .ctrl_executor_stall(ctrl_executor_stall)
    );

    wire [31:0] res_mem_o;
    wire [4:0] rd_mem_o;
    wire [11:0] rd_csr_mem_o;
    wire [31:0] tmp_mem_o;
    wire [31:0] pc_mem_o;
    wire [5:0] exc_mem_o;

    wire ctrl_nop_mem_o;
    wire ctrl_wb_mem_o;
    wire ctrl_wb_csr_mem_o;
    wire ctrl_mret_mem_o;
    wire ctrl_wfi_mem_o;

    memory mem_stage(
        .res_alu(res_ex_o),
        .rd(rd_ex_o),
        .rd_csr(rd_csr_ex_o),
        .tmp(tmp_ex_o),
        .pc(pc_ex_o),
        .exc(exc_ex_o),

        .dcache_addr(dcache_addr),
        .dcache_wdata(dcache_wdata),
        .dcache_ws(dcache_ws),
        .dcache_req(dcache_req),
        .dcache_wr(dcache_wr),
        .dcache_rdata(dcache_rdata),
        .dcache_rdy(dcache_rdy),

        .ctrl_clk(clock),
        .ctrl_reset(reset),
        .ctrl_trap(ctrl_trap),
        .ctrl_stall(ctrl_mem_stall_in),
        .ctrl_next_stage_stall(ctrl_wb_stall_in),
        .ctrl_nop(ctrl_nop_ex_o),
        .ctrl_ext_irq_trigger(irq),
        .ctrl_wb(ctrl_wb_ex_o),
        .ctrl_wb_csr(ctrl_wb_csr_ex_o),
        .ctrl_mret(ctrl_mret_ex_o),
        .ctrl_wfi(ctrl_wfi_ex_o),
        .ctrl_mem(ctrl_mem_ex_o),

        .res_reg(res_mem_o),
        .rd_reg(rd_mem_o),
        .rd_csr_reg(rd_csr_mem_o),
        .tmp_reg(tmp_mem_o),
        .pc_reg(pc_mem_o),
        .exc_reg(exc_mem_o),

        .ctrl_nop_reg(ctrl_nop_mem_o),
        .ctrl_wb_reg(ctrl_wb_mem_o),
        .ctrl_wb_csr_reg(ctrl_wb_csr_mem_o),
        .ctrl_mret_reg(ctrl_mret_mem_o),
        .ctrl_wfi_reg(ctrl_wfi_mem_o),

        .ctrl_forward_valid(ctrl_forward_valid_mem),
        .ctrl_forward_rd(ctrl_forward_rd_mem),
        .forward_data(forward_data_mem),

        .ctrl_forward_csr_valid(ctrl_forward_csr_valid_mem),
        .ctrl_forward_csr_rd(ctrl_forward_csr_rd_mem),
        .forward_csr_data(forward_csr_data_mem),

        .ctrl_ext_irq(ctrl_ext_irq),
        .ctrl_sw_irq(ctrl_sw_irq),
        .ctrl_timer_irq(ctrl_timer_irq),
        .ctrl_mem_stall(ctrl_mem_stall)
    );

    writeback wb_stage(
        .res(res_mem_o),
        .rd(rd_mem_o),
        .rd_csr(rd_csr_mem_o),
        .tmp(tmp_mem_o),
        .pc(pc_mem_o),
        .exc(exc_mem_o),

        .ctrl_clk(clock),
        .ctrl_stall(ctrl_wb_stall_in),
        .ctrl_wb(ctrl_wb_mem_o),
        .ctrl_wb_csr(ctrl_wb_csr_mem_o),
        .ctrl_mret(ctrl_mret_mem_o),
        .ctrl_wfi(ctrl_wfi_mem_o),
        .ctrl_nop(ctrl_nop_mem_o),

        .ctrl_ext_irq(ctrl_ext_irq),
        .ctrl_sw_irq(ctrl_sw_irq),
        .ctrl_timer_irq(ctrl_timer_irq),
        .ctrl_mie(ctrl_mie),
        .ctrl_mxie(ctrl_mxie),

        .reg_waddr(reg_waddr),
        .reg_wdata(reg_wdata),
        .reg_wen(reg_wen),

        .csr_raddr(csr_raddr2),
        .csr_rdata(csr_rdata2),
        .csr_waddr(csr_waddr),
        .csr_wdata(csr_wdata),
        .csr_wen(csr_wen),
        .csr_trap_pc(csr_trap_pc),
        .csr_trap_info(csr_trap_info),

        .ctrl_writeback_stall(ctrl_writeback_stall),
        .ctrl_trap(ctrl_trap),
        .ctrl_pc_exc_target(pc_exc_target),
        .ctrl_wfi_stall(ctrl_wfi_stall),
        .ctrl_mxip(ctrl_mxip)
    );

    assign _debug_pc = ctrl_nop_mem_o ? 'hffffffff : pc_mem_o;
endmodule

// Definitions copied from Linux kernel:
// https://github.com/torvalds/linux/blob/master/arch/riscv/include/asm/csr.h
// Comments copied from RISC-V Volume II

// M-mode
`define CSR_MSTATUS		'h300 // Machine status register.
`define CSR_MISA		'h301 // ISA and extensions. (WARL)
`define CSR_MIE			'h304 // Machine interrupt-enable register.
`define CSR_MTVEC		'h305 // Machine trap-handler base address.
`define CSR_MSCRATCH	'h340 // Scratch register for machine trap handlers.
`define CSR_MEPC		'h341 // Machine exception program counter.
`define CSR_MCAUSE		'h342 // Machine trap cause.
`define CSR_MTVAL		'h343 // Machine bad address or instruction.
`define CSR_MIP			'h344 // Machine interrupt pending.
`define CSR_PMPCFG0		'h3a0 // Physical memory protection configuration.
`define CSR_PMPADDR0	'h3b0 // Physical memory protection address register.
`define CSR_MHARTID		'hf14 // Hardware thread ID.

// interrupts
/* Interrupt causes (minus the high bit) */
`define IRQ_M_SOFT		3  // Machine software interrupt
`define IRQ_M_TIMER		7  // Machine timer interrupt
`define IRQ_M_EXT		11 // Machine external interrupt

// WPRI: Reserved Writes Preserve Values, Reads Ignore Values
// WLRL: Write/Read Only Legal Values
// WARL: Write Any Values, Reads Legal Values

module csr_file(
    input ctrl_clk,
    input ctrl_reset,
    input [11:0] addr,
    input [31:0] wdata,
    input wen,

    output [31:0] rdata,

    output ctrl_mie,
    output ctrl_mpie
);
    // misa: RV32I
    logic [31:0] misa;
    localparam [31:0] misa_mask = 'b00_0000_00000000000000000000000000;

    // mstatus
    logic [31:0] mstatus;
    localparam [31:0] mstatus_mask = 'b0_00000000_1_1_1_1_1_1_00_00_00_00_0_1_0_0_0_1_0_0_0;
    localparam ctrl_tsr = 0;
    localparam ctrl_tw = 0;
    localparam ctrl_tvm = 0;
    localparam ctrl_mxr = 0;
    localparam ctrl_sum = 0;
    localparam ctrl_mprv = 0;
    localparam ctrl_mpp = 'b11;
    assign ctrl_tsr = mstatus[22]; // trap sret (TSR is hard-wired to 0 when S-mode is not supported)
    assign ctrl_tw = mstatus[21]; // timeout wait (TW is hard-wired to 0 when there are no modes less privileged than M)
    assign ctrl_tvm = mstatus[20]; // trap virtual memory (TVM is hard-wired to 0 when S-mode is not supported)
    assign ctrl_mxr = mstatus[19]; // make executable readable (MXR is hardwired to 0 if S-mode is not supported)
    assign ctrl_sum = mstatus[18]; // supervisor user memory access (SUM is hardwired to 0 if S-mode is not supported)
    assign ctrl_mprv = mstatus[17]; // modify privilege (MPRV is hardwired to 0 if U-mode is not supported)
    assign ctrl_mpp = mstatus[12:11]; // previous privilege mode (hardwired to M)
    assign ctrl_mpie = mstatus[7]; // interrupt-enable bit active prior to the trap
    assign ctrl_mie = mstatus[3]; // interrupt-enable bit
    //assign ctrl_spp = mstatus[8];
    //assign ctrl_spie = mstatus[5];
    //assign ctrl_sie = mstatus[1];
    // SD1_(WPRI)_TSR1_TW1_TVM1_MXR1_SUM1_MPRV1_XS2_FS2_MPP2_(WPRI)2_SPP1_MPIE1_(WPRI)1_SPIE1_UPIE1_MIE1_(WPRI)1_SIE1_UIE1
    // FS is WARL (should always be 0 as we don't support floating-point units)
    // XS is RO
    // SD is zero
    // UIE is zero
    // UPIE is zero
    // MPP/SPP are WARL

    // mip/mie
    logic [31:0] mip;
    logic [31:0] mie;
    localparam [31:0] mi_mask = {{20{1'b0}}, 12'b0_0_0_0_0_0_0_0_0_0_0_0};
    // MEIP_(WPRI)_SEIP_UEIP_MTIP_WPRI_STIP_UTIP_MSIP_(WPRI)_SSIP_USIP

    logic [31:0] mtvec;
    logic [31:0] mscratch;
    logic [31:0] mepc;
    logic [31:0] mcause;
    logic [31:0] mtval;

    localparam mhartid = 0; // Hart ID is always 0

    assign rdata = (addr == `CSR_MSTATUS) ? mstatus :
                   (addr == `CSR_MISA) ? misa :
                   (addr == `CSR_MIE) ? mie :
                   (addr == `CSR_MTVEC) ? mtvec :
                   (addr == `CSR_MSCRATCH) ? mscratch :
                   (addr == `CSR_MEPC) ? mepc :
                   (addr == `CSR_MCAUSE) ? mcause :
                   (addr == `CSR_MTVAL) ? mtval :
                   (addr == `CSR_MIP) ? mip :
                   (addr == `CSR_MHARTID) ? mhartid :
                                            'bx;
    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            misa <= 'b01_0000_00000000000000000100000000; // RV32I
            mstatus <= 'b0_00000000_0_0_0_0_0_0_00_00_00_00_0_0_0_0_0_0_0_0_0;
            mip <= {32{1'b0}};
            mie <= {32{1'b0}};
            mtvec <= 'h00000000;
            mscratch <= 'h00000000;
            mepc <= 'h00000000;
        end
        if (wen) begin
            case (addr)
                `CSR_MSTATUS: mstatus <= (mstatus & (~mstatus_mask)) | (wdata & mstatus_mask);
                `CSR_MISA: misa <= (misa & (~misa_mask)) | (wdata & misa_mask);
                `CSR_MIE: mie <= (mie & (~mi_mask)) | (wdata & mi_mask);
                `CSR_MTVEC: mtvec <= (wdata[1:0] < 2) ? wdata : mtvec;
                `CSR_MSCRATCH: mscratch <= wdata;
                `CSR_MEPC: mepc <= mepc & {{30{1'b1}}, 2'b00};
                `CSR_MCAUSE: mcause <=
                    (wdata[31] ? (wdata[30:0] < 12 &&
                                 wdata[30:0] != 2 &&
                                 wdata[30:0] != 6 &&
                                 wdata[30:0] != 10) :
                                 (wdata[30:0] < 16 &&
                                     wdata[30:0] != 10 &&
                                     wdata[30:0] != 14)) ? wdata : mcause;
                `CSR_MTVAL: mtval <= wdata;
                `CSR_MIP: mip <= (mip & (~mi_mask)) | (wdata & mi_mask);
                default:;
            endcase
        end
    end
endmodule

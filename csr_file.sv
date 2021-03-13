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
    input [11:0] raddr1,
    input [11:0] raddr2,
    output [31:0] rdata1,
    output [31:0] rdata2,

    input wen,
    input [11:0] waddr,
    input [31:0] wdata,

    input [31:0] pc,
    input ctrl_clk,
    input ctrl_reset,
    input ctrl_trap,
    output ctrl_mie, // whether interrupts are globally enabled
    output ctrl_mpie // holds the value of the interrupt-enable bit active prior to the trap
);
    // misa: RV32I
    logic [31:0] misa;
    localparam [31:0] misa_mask = 'b00_0000_00000000000000000000000000;

    // mstatus
    logic [31:0] mstatus;
    localparam [31:0] mstatus_mask = 'b0_00000000_0_0_0_0_0_0_00_00_00_00_0_1_0_0_0_1_0_0_0;
    // trap sret (TSR is hard-wired to 0 when S-mode is not supported)
    // timeout wait (TW is hard-wired to 0 when there are no modes less privileged than M)
    // trap virtual memory (TVM is hard-wired to 0 when S-mode is not supported)
    // make executable readable (MXR is hardwired to 0 if S-mode is not supported)
    // supervisor user memory access (SUM is hardwired to 0 if S-mode is not supported)
    // modify privilege (MPRV is hardwired to 0 if U-mode is not supported)
    // previous privilege mode (hardwired to M)
    assign ctrl_mpie = mstatus[7]; // interrupt-enable bit active prior to the trap
    assign ctrl_mie = mstatus[3]; // interrupt-enable bit
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
    localparam [31:0] mi_mask = {{16{1'b1}}, 4'b0, 12'b1_0_0_0_1_0_0_0_1_0_0_0};
    // MEIx_(WPRI)_SEIx_UEIx_MTIx_WPRI_STIx_UTIx_MSIx_(WPRI)_SSIx_USIx

    logic [31:0] mtvec;
    logic [31:0] mscratch;
    logic [31:0] mepc;
    logic [31:0] mcause;
    logic [31:0] mtval;

    localparam mhartid = 0; // Hart ID is always 0
    localparam _mhartid = mhartid;

    // when trap:
    // 0. it is guaranteed that no read/write is on-going and all previous writes are visible
    // 1. the value of mstatus.MIE is copied into mcause.MPIE, and then
    // mstatus.MIE is cleared, effectively disabling interrupts.
    // 2. the current pc is copied into the mepc register

    wire [31:0] _mstatus = ctrl_trap ? {mstatus[31:8], mstatus[3], mstatus[6:4], 1'b0, mstatus[2:0]} : 
                                       ((mstatus & (~mstatus_mask)) | (wdata & mstatus_mask));
    wire [31:0] _misa = (misa & (~misa_mask)) | (wdata & misa_mask);
    wire [31:0] _mie = (mie & (~mi_mask)) | (wdata & mi_mask);
    wire [31:0] _mtvec = (wdata[1:0] < 2) ? wdata : mtvec;
    wire [31:0] _mscratch = wdata;
    wire [31:0] _mepc = ctrl_trap ? pc : (mepc & {{30{1'b1}}, 2'b00});
    wire [31:0] _mcause = (wdata[31] ? (wdata[30:0] < 12 &&
                                 wdata[30:0] != 2 &&
                                 wdata[30:0] != 6 &&
                                 wdata[30:0] != 10) :
                                 (wdata[30:0] < 16 &&
                                     wdata[30:0] != 10 &&
                                     wdata[30:0] != 14)) ? wdata : mcause;
    wire [31:0] _mtval = wdata;
    wire [31:0] _mip = (mip & (~mi_mask)) | (wdata & mi_mask);

    `define read_csr(raddr, reg_addr, reg) \
        (raddr == reg_addr) ? (raddr == waddr ? _``reg : reg)

    `define read_csr_with_bypass(raddr) \
        `read_csr(raddr, `CSR_MSTATUS, mstatus) : \
        `read_csr(raddr, `CSR_MISA, misa) : \
        `read_csr(raddr, `CSR_MIE, mie) : \
        `read_csr(raddr, `CSR_MTVEC, mtvec) : \
        `read_csr(raddr, `CSR_MSCRATCH, mscratch) : \
        `read_csr(raddr, `CSR_MEPC, mepc) : \
        `read_csr(raddr, `CSR_MCAUSE, mcause) : \
        `read_csr(raddr, `CSR_MTVAL, mtval) : \
        `read_csr(raddr, `CSR_MIP, mip) : \
        `read_csr(raddr, `CSR_MHARTID, mhartid) : 'bx


    assign rdata1 = `read_csr_with_bypass(raddr1);
    assign rdata2 = `read_csr_with_bypass(raddr2);

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            misa <= 'b01_0000_00000000000000000100000000; // RV32I
            mstatus <= 'b0_00000000_0_0_0_0_0_0_00_00_11_00_0_0_0_0_0_0_0_0_0;
            mip <= {32{1'b0}};
            mie <= {32{1'b0}};
            mtvec <= 'h00000000;
            mscratch <= 'h00000000;
            mepc <= 'h00000000;
            mcause <= 'h00000000;
            mtval <= 'h00000000;
        end
        if (wen) begin
            case (waddr)
                `CSR_MSTATUS: mstatus <= _mstatus;
                `CSR_MISA: misa <= _misa;
                `CSR_MIE: mie <= _mie;
                `CSR_MTVEC: mtvec <= _mtvec;
                `CSR_MSCRATCH: mscratch <= _mscratch;
                `CSR_MEPC: mepc <= _mepc;
                `CSR_MCAUSE: mcause <= _mcause;
                `CSR_MTVAL: mtval <= _mtval;
                `CSR_MIP: mip <= _mip;
                default:;
            endcase
        end
    end
endmodule

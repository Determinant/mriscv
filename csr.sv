`ifndef SYNTHESIS
    `ifdef DEBUG
        `define CSR_DEBUG
    `endif
`endif

// RISC-V Volume II
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

// WPRI: Reserved Writes Preserve Values, Reads Ignore Values
// WLRL: Write/Read Only Legal Values
// WARL: Write Any Values, Reads Legal Values

module csr(
    input [11:0] raddr1,
    input [11:0] raddr2,
    output [31:0] rdata1,
    output [31:0] rdata2,

    input wen,
    input [11:0] waddr,
    input [31:0] wdata,

    input [31:0] trap_pc,
    input [4:0] trap_info,
    input [2:0] ctrl_mxip, // [0] external interrupt pending
                           // [1] software interrupt pending
                           // [2] timer interrupt pending
    input ctrl_clk,
    input ctrl_reset,
    input ctrl_trap,
    input ctrl_mret,
    output ctrl_mie, // whether interrupts are globally enabled
    output [2:0] ctrl_mxie,
    output ctrl_addr_valid
);
    // misa: RV32I
    logic [31:0] misa;
    localparam [31:0] misa_mask = 'b00_0000_00000000000000000000000000;

    // mstatus
    logic [31:0] mstatus;
    localparam [31:0] mstatus_mask = 'b0_00000000_0_0_0_0_0_0_00_00_00_00_0_1_0_0_0_1_0_0_0;
    // TSR is hard-wired to 0 when S-mode is not supported.
    // TW is hard-wired to 0 when there are no modes less privileged than M.
    // TVM is hard-wired to 0 when S-mode is not supported.
    // MXR is hardwired to 0 if S-mode is not supported.
    // SUM is hardwired to 0 if S-mode is not supported.
    // MPRV is hardwired to 0 if U-mode is not supported.
    // MPP is hardwired to M.
    // FS should always be 0 as we don't support floating-point units.
    // XS is RO
    // SD is zero
    // UIE is zero
    // UPIE is zero
    assign ctrl_mie = mstatus[3]; // interrupt-enable bit

    // mie
    logic [31:0] mie;
    localparam [31:0] mie_mask = {{16{1'b1}}, 4'b0, 12'b1_0_0_0_1_0_0_0_1_0_0_0};
    assign ctrl_mxie[0] = mie[11];
    assign ctrl_mxie[1] = mie[3];
    assign ctrl_mxie[2] = mie[7];
    // MEIx_(WPRI)_SEIx_UEIx_MTIx_WPRI_STIx_UTIx_MSIx_(WPRI)_SSIx_USIx

    logic [31:0] mtvec;
    logic [31:0] mscratch;
    logic [31:0] mepc;
    logic [31:0] mcause;
    logic [31:0] mtval;

    localparam [31:0] mhartid = 0; // Hart ID is always 0
    localparam _mhartid = mhartid;

    wire [31:0] _mstatus = (mstatus & (~mstatus_mask)) | (wdata & mstatus_mask);
    wire [31:0] _misa = (misa & (~misa_mask)) | (wdata & misa_mask);
    wire [31:0] _mie = (mie & (~mie_mask)) | (wdata & mie_mask);
    wire [31:0] _mtvec = (wdata[1:0] < 2) ? wdata : mtvec;
    wire [31:0] _mscratch = wdata;
    wire [31:0] _mepc = {wdata[31:2], 2'b00};
    wire [31:0] _mcause = (wdata[31] ? (wdata[30:0] < 12 &&
                                 wdata[30:0] != 2 &&
                                 wdata[30:0] != 6 &&
                                 wdata[30:0] != 10) :
                                 (wdata[30:0] < 16 &&
                                     wdata[30:0] != 10 &&
                                     wdata[30:0] != 14)) ? wdata : mcause;
    wire [31:0] _mtval = wdata;
    wire [31:0] _mip = {20'b0, ctrl_mxip[0], 3'b0, ctrl_mxip[2], 3'b0, ctrl_mxip[1], 3'b0};

    // used by trap handling logic
    wire [31:0] trap_mstatus = {mstatus[31:8], mstatus[3], mstatus[6:4], 1'b0, mstatus[2:0]};
    wire [31:0] trap_mcause = {trap_info[4], 27'b0, trap_info[3:0]};
    wire [31:0] mret_mstatus = {mstatus[31:8], 1'b0, mstatus[6:4], mstatus[7], mstatus[2:0]};

    `define read_csr(raddr, reg_addr, reg) \
        (raddr == reg_addr) ? {(raddr == waddr ? _``reg : reg), 1'b1}

    `define read_csr_with_bypass(raddr) ( \
        `read_csr(raddr, `CSR_MSTATUS, mstatus) : \
        `read_csr(raddr, `CSR_MISA, misa) : \
        `read_csr(raddr, `CSR_MIE, mie) : \
        `read_csr(raddr, `CSR_MTVEC, mtvec) : \
        `read_csr(raddr, `CSR_MSCRATCH, mscratch) : \
        `read_csr(raddr, `CSR_MEPC, mepc) : \
        `read_csr(raddr, `CSR_MCAUSE, mcause) : \
        `read_csr(raddr, `CSR_MTVAL, mtval) : \
        (raddr == `CSR_MIP) ? {_mip, 1'b1} : \
        `read_csr(raddr, `CSR_MHARTID, mhartid) : 33'b0)


    wire [32:0] read1 = `read_csr_with_bypass(raddr1);
    wire [32:0] read2 = `read_csr_with_bypass(raddr2);
    assign rdata1 = read1[32:1];
    assign rdata2 = read2[32:1];
    wire _unused = read2[0];
    assign ctrl_addr_valid = read1[0];

    always_ff @ (posedge ctrl_clk) begin
        if (ctrl_reset) begin
            misa <= 'b01_0000_00000000000000000100000000; // RV32I
            mstatus <= 'b0_00000000_0_0_0_0_0_0_00_00_11_00_0_0_0_0_0_0_0_0_0;
            mie <= {32{1'b0}};
            mtvec <= 'h00000000;
            mscratch <= 'h00000000;
            mepc <= 'h00000000;
            mcause <= 'h00000000;
            mtval <= 'h00000000;
        end
        if (ctrl_trap) begin
            // On trap:
            // 0. it is guaranteed that no read/write is on-going and all previous writes are visible
            // 1. the value of mstatus.MIE is copied into mcause.MPIE, and then
            // mstatus.MIE is cleared, effectively disabling interrupts.
            // 2. the current pc is copied into the mepc register
            // Exit trap:
            // 1. mstatus.mie = mstatus.mpie, restore interrupt enable
            // 2. mstatus.mpie = 0
            if (ctrl_mret) begin
                `ifdef CSR_DEBUG
                    $display("[%0t] ! CSR  mret: mstatus=0x%h to CSR", $time, mret_mstatus);
                `endif
                mstatus <= mret_mstatus;
            end else begin
                `ifdef CSR_DEBUG
                    $display("[%0t] ! CSR  trap: mstatus=0x%h mepc=0x%h mcause=0x%h to CSR",
                        $time, trap_mstatus, trap_pc, trap_mcause);
                `endif
                mstatus <= trap_mstatus;
                mepc <= trap_pc;
                mcause <= trap_mcause;
            end
        end else if (wen) begin
            `ifdef CSR_DEBUG
                $display("[%0t] ! CSR  writes 0x%h to csr[0x%h]", $time, wdata, waddr);
            `endif
            case (waddr)
                `CSR_MSTATUS: mstatus <= _mstatus;
                `CSR_MISA: misa <= _misa;
                `CSR_MIE: mie <= _mie;
                `CSR_MTVEC: mtvec <= _mtvec;
                `CSR_MSCRATCH: mscratch <= _mscratch;
                `CSR_MEPC: mepc <= _mepc;
                `CSR_MCAUSE: mcause <= _mcause;
                `CSR_MTVAL: mtval <= _mtval;
                default:;
            endcase
        end
    end
endmodule

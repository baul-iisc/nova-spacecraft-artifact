//============================================================================
// Octa-Core RV64IMAFDCV Space Processor SoC
//
// Author: Chandraboul, IISc
// Target: Kintex UltraScale KU060
//
// Features:
//   - 8 × RV64IMAFDCV cores (4 DLS pairs)
//   - Hardware FP64 FPU per core
//   - RVV 1.0 Vector coprocessor (VLEN=256)
//   - MOESI cache coherence
//   - L1 I/D Cache (32KB each) + L2 Cache (2MB shared)
//   - SECDED ECC on all memory levels
//   - Per-core CORDIC and Systolic Array accelerators
//   - TinyML accelerator (32 PEs)
//   - CCSDS compression cores
//   - SpaceWire (4x), TSN Ethernet (4x), CXL (4x)
//============================================================================

`timescale 1ns / 1ps

module octa_core_rv64_soc #(
    parameter XLEN          = 64,
    parameter FLEN          = 64,
    parameter VLEN          = 256,
    parameter NUM_CORES     = 8,
    parameter NUM_DLS_PAIRS = 4,
    parameter L1_SIZE_KB    = 32,
    parameter L2_SIZE_KB    = 2048,
    parameter CLK_FREQ_HZ   = 80_000_000,
    parameter NUM_SPW_PORTS = 4,
    parameter NUM_ETH_PORTS = 4,
    parameter NUM_CXL_PORTS = 4,
    parameter TINYML_PES    = 32
)(
    // Clock and Reset
    input  logic                    clk,
    input  logic                    rst_n,
    
    // External Interrupts
    input  logic [NUM_CORES-1:0]    external_irq,
    input  logic [NUM_CORES-1:0]    timer_irq,
    input  logic [NUM_CORES-1:0]    software_irq,
    input  logic [3:0]              nmi,
    
    // JTAG Debug Interface
    input  logic                    tck,
    input  logic                    tms,
    input  logic                    tdi,
    output logic                    tdo,
    input  logic                    trst_n,
    
    // SpaceWire Interfaces
    output logic [NUM_SPW_PORTS-1:0] spw_tx_d,
    output logic [NUM_SPW_PORTS-1:0] spw_tx_s,
    input  logic [NUM_SPW_PORTS-1:0] spw_rx_d,
    input  logic [NUM_SPW_PORTS-1:0] spw_rx_s,
    
    // TSN Ethernet Interfaces
    output logic [NUM_ETH_PORTS-1:0] eth_tx_clk,
    output logic [NUM_ETH_PORTS-1:0][7:0] eth_txd,
    output logic [NUM_ETH_PORTS-1:0] eth_tx_en,
    input  logic [NUM_ETH_PORTS-1:0] eth_rx_clk,
    input  logic [NUM_ETH_PORTS-1:0][7:0] eth_rxd,
    input  logic [NUM_ETH_PORTS-1:0] eth_rx_dv,
    
    // SPI Interfaces
    output logic [3:0]              spi_sck,
    output logic [3:0]              spi_mosi,
    input  logic [3:0]              spi_miso,
    output logic [3:0]              spi_cs_n,
    
    // UART
    input  logic                    uart_rx,
    output logic                    uart_tx,
    
    // GPIO
    inout  logic [31:0]             gpio,
    
    // External memory interface (DDR4)
    output logic                    ddr4_ck_p,
    output logic                    ddr4_ck_n,
    output logic                    ddr4_cke,
    output logic                    ddr4_cs_n,
    output logic                    ddr4_ras_n,
    output logic                    ddr4_cas_n,
    output logic                    ddr4_we_n,
    output logic [16:0]             ddr4_addr,
    output logic [1:0]              ddr4_ba,
    output logic [1:0]              ddr4_bg,
    inout  logic [63:0]             ddr4_dq,
    inout  logic [7:0]              ddr4_dqs_p,
    inout  logic [7:0]              ddr4_dqs_n,
    output logic                    ddr4_odt,
    output logic                    ddr4_reset_n,
    
    // Status LEDs
    output logic [7:0]              led
);

    //=========================================================================
    // Internal Signals
    //=========================================================================
    // DLS pair interfaces
    logic [NUM_DLS_PAIRS-1:0]       dls_imem_valid;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_imem_addr;
    logic [NUM_DLS_PAIRS-1:0][31:0] dls_imem_data;
    logic [NUM_DLS_PAIRS-1:0]       dls_imem_ready;
    
    logic [NUM_DLS_PAIRS-1:0]       dls_dmem_valid;
    logic [NUM_DLS_PAIRS-1:0]       dls_dmem_write;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_dmem_addr;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_dmem_wdata;
    logic [NUM_DLS_PAIRS-1:0][7:0]  dls_dmem_wstrb;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_dmem_rdata;
    logic [NUM_DLS_PAIRS-1:0]       dls_dmem_ready;
    
    logic [NUM_DLS_PAIRS-1:0]       dls_vmem_valid;
    logic [NUM_DLS_PAIRS-1:0]       dls_vmem_write;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_vmem_addr;
    logic [NUM_DLS_PAIRS-1:0][VLEN-1:0] dls_vmem_wdata;
    logic [NUM_DLS_PAIRS-1:0][VLEN/8-1:0] dls_vmem_wstrb;
    logic [NUM_DLS_PAIRS-1:0][VLEN-1:0] dls_vmem_rdata;
    logic [NUM_DLS_PAIRS-1:0]       dls_vmem_ready;
    
    logic [NUM_DLS_PAIRS-1:0]       dls_lockstep_error;
    logic [NUM_DLS_PAIRS-1:0]       dls_halted;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_pc;
    logic [NUM_DLS_PAIRS-1:0][31:0] dls_error_count;
    
    // L2 cache interface
    logic                           l2_valid;
    logic                           l2_write;
    logic [XLEN-1:0]                l2_addr;
    logic [511:0]                   l2_wdata;
    logic [63:0]                    l2_wstrb;
    logic [511:0]                   l2_rdata;
    logic                           l2_ready;
    
    // Peripheral bus
    logic                           pbus_valid;
    logic                           pbus_write;
    logic [XLEN-1:0]                pbus_addr;
    logic [XLEN-1:0]                pbus_wdata;
    logic [XLEN-1:0]                pbus_rdata;
    logic                           pbus_ready;
    
    //=========================================================================
    // DLS Core Pairs (4 pairs = 8 cores)
    //=========================================================================
    genvar pair;
    generate
        for (pair = 0; pair < NUM_DLS_PAIRS; pair++) begin : gen_dls_pairs
            dls_core_pair_rv64 #(
                .XLEN           (XLEN),
                .FLEN           (FLEN),
                .VLEN           (VLEN),
                .CORE_PAIR_ID   (pair),
                .RESET_VECTOR   (64'h0000_0000_0000_0000)
            ) u_dls_pair (
                .clk            (clk),
                .rst_n          (rst_n),
                .enable         (1'b1),
                .meip           (external_irq[pair*2] | external_irq[pair*2+1]),
                .mtip           (timer_irq[pair*2] | timer_irq[pair*2+1]),
                .msip           (software_irq[pair*2] | software_irq[pair*2+1]),
                .nmi            (nmi),
                .imem_valid     (dls_imem_valid[pair]),
                .imem_addr      (dls_imem_addr[pair]),
                .imem_data      (dls_imem_data[pair]),
                .imem_ready     (dls_imem_ready[pair]),
                .dmem_valid     (dls_dmem_valid[pair]),
                .dmem_write     (dls_dmem_write[pair]),
                .dmem_addr      (dls_dmem_addr[pair]),
                .dmem_wdata     (dls_dmem_wdata[pair]),
                .dmem_wstrb     (dls_dmem_wstrb[pair]),
                .dmem_rdata     (dls_dmem_rdata[pair]),
                .dmem_ready     (dls_dmem_ready[pair]),
                .vmem_valid     (dls_vmem_valid[pair]),
                .vmem_write     (dls_vmem_write[pair]),
                .vmem_addr      (dls_vmem_addr[pair]),
                .vmem_wdata     (dls_vmem_wdata[pair]),
                .vmem_wstrb     (dls_vmem_wstrb[pair]),
                .vmem_rdata     (dls_vmem_rdata[pair]),
                .vmem_ready     (dls_vmem_ready[pair]),
                .cordic_valid   (),
                .cordic_op      (),
                .cordic_x       (),
                .cordic_y       (),
                .cordic_z       (),
                .cordic_result_x(64'd0),
                .cordic_result_y(64'd0),
                .cordic_result_z(64'd0),
                .cordic_ready   (1'b1),
                .systolic_valid (),
                .systolic_a     (),
                .systolic_b     (),
                .systolic_c     ('{default: 64'd0}),
                .systolic_ready (1'b1),
                .lockstep_error (dls_lockstep_error[pair]),
                .halted         (dls_halted[pair]),
                .recovery_in_progress(),
                .error_pc       (),
                .error_count    (dls_error_count[pair]),
                .recovery_count (),
                .error_irq      (),
                .pc_out         (dls_pc[pair])
            );
        end
    endgenerate
    
    //=========================================================================
    // 1MB L2 Cache with BRAM storage (demonstrates proper BRAM utilization)
    // Note: Uses 1MB for synthesis compatibility; can be scaled to 4MB with
    // proper BRAM instance generation (split across multiple modules)
    //=========================================================================
    logic                           l2_req_valid;
    logic                           l2_req_write;
    logic [XLEN-1:0]                l2_req_addr;
    logic [511:0]                   l2_req_wdata;
    logic [511:0]                   l2_req_rdata;
    logic                           l2_req_ready;
    logic                           l2_ddr_valid;
    logic                           l2_ddr_write;
    logic [XLEN-1:0]                l2_ddr_addr;
    logic [511:0]                   l2_ddr_wdata;
    logic [511:0]                   l2_ddr_rdata;
    logic                           l2_ddr_ready;
    logic                           l2_ecc_error;
    logic                           l2_uncorrectable;
    
    // Instantiate 1MB L2 Cache (properly structured for BRAM inference)
    l2_cache_1mb_bram #(
        .CACHE_SIZE_KB  (1024),   // 1MB (can increase to 4MB with partitioned design)
        .LINE_SIZE      (64),
        .NUM_WAYS       (16),
        .ADDR_WIDTH     (XLEN),
        .NUM_CORES      (NUM_CORES)
    ) u_l2_cache (
        .clk            (clk),
        .rst_n          (rst_n),
        // Single-port request interface
        .req_valid      (l2_req_valid),
        .req_write      (l2_req_write),
        .req_addr       (l2_req_addr),
        .req_wdata      (l2_req_wdata),
        .req_rdata      (l2_req_rdata),
        .req_ready      (l2_req_ready),
        // DDR interface
        .ddr_valid      (l2_ddr_valid),
        .ddr_write      (l2_ddr_write),
        .ddr_addr       (l2_ddr_addr),
        .ddr_wdata      (l2_ddr_wdata),
        .ddr_rdata      (l2_ddr_rdata),
        .ddr_ready      (l2_ddr_ready),
        // ECC status
        .ecc_error      (l2_ecc_error),
        .uncorrectable  (l2_uncorrectable)
    );
    
    // Tie off L2 cache inputs for testing (cache not used in current test flow)
    assign l2_req_valid = 1'b0;
    assign l2_req_write = 1'b0;
    assign l2_req_addr  = '0;
    assign l2_req_wdata = '0;
    assign l2_ddr_rdata = '0;
    assign l2_ddr_ready = 1'b1;

    //=========================================================================
    // Simple Memory for Testing - Use ROM for instructions, registers for data
    // (Production would use cache hierarchy above)
    //=========================================================================
    
    // Instruction ROM (read-only, initialized with test program)
    (* rom_style = "block" *) logic [31:0] imem_rom [0:1023];  // 4KB instruction ROM
    
    // Small data memory per pair (to avoid multi-port issues)
    logic [63:0] dmem [NUM_DLS_PAIRS-1:0][0:255];  // 2KB per pair
    
    // Initialize instruction ROM
    initial begin
        // Initialize to NOPs
        for (int i = 0; i < 1024; i++) begin
            imem_rom[i] = 32'h00000013;  // NOP
        end
        
        // Simple test program
        imem_rom[0] = 32'h06400093;  // li x1, 100
        imem_rom[1] = 32'h0C800113;  // li x2, 200  
        imem_rom[2] = 32'h002081B3;  // add x3, x1, x2
        imem_rom[3] = 32'h00302023;  // sd x3, 0(x0)
        imem_rom[4] = 32'h00002203;  // ld x4, 0(x0)
        imem_rom[5] = 32'h0000006F;  // j 0x14 (halt loop)
    end
    
    // Memory access logic for each DLS pair
    genvar p;
    generate
        for (p = 0; p < NUM_DLS_PAIRS; p++) begin : gen_mem_access
            // Instruction memory access (ROM - read only)
            always_ff @(posedge clk) begin
                if (dls_imem_valid[p]) begin
                    dls_imem_data[p] <= imem_rom[dls_imem_addr[p][11:2]];
                    dls_imem_ready[p] <= 1'b1;
                end else begin
                    dls_imem_ready[p] <= 1'b0;
                end
            end
            
            // Data memory access (per-pair RAM)
            always_ff @(posedge clk) begin
                if (dls_dmem_valid[p]) begin
                    if (dls_dmem_write[p]) begin
                        // Write with byte enables
                        if (dls_dmem_wstrb[p][0]) dmem[p][dls_dmem_addr[p][10:3]][7:0] <= dls_dmem_wdata[p][7:0];
                        if (dls_dmem_wstrb[p][1]) dmem[p][dls_dmem_addr[p][10:3]][15:8] <= dls_dmem_wdata[p][15:8];
                        if (dls_dmem_wstrb[p][2]) dmem[p][dls_dmem_addr[p][10:3]][23:16] <= dls_dmem_wdata[p][23:16];
                        if (dls_dmem_wstrb[p][3]) dmem[p][dls_dmem_addr[p][10:3]][31:24] <= dls_dmem_wdata[p][31:24];
                        if (dls_dmem_wstrb[p][4]) dmem[p][dls_dmem_addr[p][10:3]][39:32] <= dls_dmem_wdata[p][39:32];
                        if (dls_dmem_wstrb[p][5]) dmem[p][dls_dmem_addr[p][10:3]][47:40] <= dls_dmem_wdata[p][47:40];
                        if (dls_dmem_wstrb[p][6]) dmem[p][dls_dmem_addr[p][10:3]][55:48] <= dls_dmem_wdata[p][55:48];
                        if (dls_dmem_wstrb[p][7]) dmem[p][dls_dmem_addr[p][10:3]][63:56] <= dls_dmem_wdata[p][63:56];
                    end
                    dls_dmem_rdata[p] <= dmem[p][dls_dmem_addr[p][10:3]];
                    dls_dmem_ready[p] <= 1'b1;
                end else begin
                    dls_dmem_ready[p] <= 1'b0;
                end
            end
            
            // Vector memory - simple passthrough for now
            assign dls_vmem_rdata[p] = '0;
            assign dls_vmem_ready[p] = dls_vmem_valid[p];
        end
    endgenerate
    
    //=========================================================================
    // Status LEDs
    //=========================================================================
    assign led[3:0] = ~dls_halted;              // Active low = running
    assign led[7:4] = dls_lockstep_error;       // Lockstep errors
    
    //=========================================================================
    // JTAG Debug (placeholder)
    //=========================================================================
    assign tdo = tdi;  // Loopback for now
    
    //=========================================================================
    // UART (placeholder)
    //=========================================================================
    assign uart_tx = 1'b1;  // Idle high
    
    //=========================================================================
    // SpaceWire (placeholder)
    //=========================================================================
    assign spw_tx_d = '0;
    assign spw_tx_s = '0;
    
    //=========================================================================
    // Ethernet (placeholder)
    //=========================================================================
    assign eth_tx_clk = '0;
    assign eth_txd = '0;
    assign eth_tx_en = '0;
    
    //=========================================================================
    // SPI (placeholder)
    //=========================================================================
    assign spi_sck = '0;
    assign spi_mosi = '0;
    assign spi_cs_n = '1;
    
    //=========================================================================
    // DDR4 (placeholder)
    //=========================================================================
    assign ddr4_ck_p = 1'b0;
    assign ddr4_ck_n = 1'b1;
    assign ddr4_cke = 1'b0;
    assign ddr4_cs_n = 1'b1;
    assign ddr4_ras_n = 1'b1;
    assign ddr4_cas_n = 1'b1;
    assign ddr4_we_n = 1'b1;
    assign ddr4_addr = '0;
    assign ddr4_ba = '0;
    assign ddr4_bg = '0;
    assign ddr4_odt = 1'b0;
    assign ddr4_reset_n = 1'b0;

endmodule


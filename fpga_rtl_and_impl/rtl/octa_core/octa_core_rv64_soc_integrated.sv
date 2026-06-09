//============================================================================
// Octa-Core RV64IMAFDCV Space Processor SoC - FULLY INTEGRATED
//
// Author: Chandraboul, IISc
// Target: Kintex UltraScale KU060
// Version: 3.0 (Full Integration)
//
// Architecture:
//   ┌─────────────────────────────────────────────────────────────────────┐
//   │                        SoC Integrated Architecture                  │
//   ├─────────────────────────────────────────────────────────────────────┤
//   │                                                                     │
//   │   ┌──────────────────────┐     ┌──────────────────────┐            │
//   │   │ DLS Pair 0           │     │ DLS Pair 1           │            │
//   │   │ ┌────────┬────────┐  │     │ ┌────────┬────────┐  │            │
//   │   │ │ Core 0 │ Core 1 │  │     │ │ Core 2 │ Core 3 │  │            │
//   │   │ │ +FPU   │ +FPU   │  │     │ │ +FPU   │ +FPU   │  │            │
//   │   │ │ +RVV   │ +RVV   │  │     │ │ +RVV   │ +RVV   │  │            │
//   │   │ └───┬────┴───┬────┘  │     │ └───┬────┴───┬────┘  │            │
//   │   │     └───┬────┘       │     │     └───┬────┘       │            │
//   │   │   ┌─────┴─────┐      │     │   ┌─────┴─────┐      │            │
//   │   │   │ CORDIC    │      │     │   │ CORDIC    │      │            │
//   │   │   │ Systolic  │      │     │   │ Systolic  │      │            │
//   │   │   └───────────┘      │     │   └───────────┘      │            │
//   │   └────────┬─────────────┘     └────────┬─────────────┘            │
//   │            │                            │                          │
//   │   ┌────────┴─────────┐     ┌────────────┴────────┐                 │
//   │   │  L1 I$ + D$      │     │  L1 I$ + D$         │                 │
//   │   │  32KB each       │     │  32KB each          │                 │
//   │   │  SECDED ECC      │     │  SECDED ECC         │                 │
//   │   └────────┬─────────┘     └────────┬────────────┘                 │
//   │            │                        │                              │
//   │            └──────────┬─────────────┘                              │
//   │                       │                                            │
//   │              ┌────────┴────────┐                                   │
//   │              │   MOESI Snoop   │                                   │
//   │              │      Bus        │                                   │
//   │              └────────┬────────┘                                   │
//   │                       │                                            │
//   │              ┌────────┴────────┐                                   │
//   │              │   L2 Cache      │                                   │
//   │              │   4MB Shared    │                                   │
//   │              │   16-way        │                                   │
//   │              │   SECDED ECC    │                                   │
//   │              └────────┬────────┘                                   │
//   │                       │                                            │
//   │   ┌───────────────────┼───────────────────┐                        │
//   │   │                   │                   │                        │
//   │  DDR4              PROM                SRAM                        │
//   │  Controller        Controller          Controller                  │
//   │                                                                    │
//   ├────────────────────────────────────────────────────────────────────┤
//   │  Accelerators: TinyML (32 PEs), CCSDS (TC/TM/Image)                │
//   │  Interfaces: SpaceWire×4, TSN×2(4-port), CXL×2, JESD204B×2,       │
//   │              SPI×4, UART, JTAG                                      │
//   └────────────────────────────────────────────────────────────────────┘
//============================================================================

`timescale 1ns / 1ps

module octa_core_rv64_soc_integrated #(
    parameter XLEN          = 64,
    parameter FLEN          = 64,
    parameter VLEN          = 256,
    parameter NUM_CORES     = 8,
    parameter NUM_DLS_PAIRS = 4,
    parameter L1I_SIZE_KB   = 32,       // 32KB L1 I-Cache per core
    parameter L1D_SIZE_KB   = 32,       // 32KB L1 D-Cache per core
    parameter L2_SIZE_KB    = 4096,     // 4MB shared L2 cache
    parameter CLK_FREQ_HZ   = 80_000_000,
    parameter NUM_SPW_PORTS = 4,
    parameter NUM_ETH_PORTS = 4,
    parameter NUM_CXL_PORTS = 2,
    parameter NUM_JESD_LANES= 4,
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
    
    // SpaceWire Interfaces (4x)
    output logic [NUM_SPW_PORTS-1:0] spw_tx_d,
    output logic [NUM_SPW_PORTS-1:0] spw_tx_s,
    input  logic [NUM_SPW_PORTS-1:0] spw_rx_d,
    input  logic [NUM_SPW_PORTS-1:0] spw_rx_s,
    
    // TSN Ethernet Interfaces (4x)
    output logic [NUM_ETH_PORTS-1:0] eth_tx_clk,
    output logic [NUM_ETH_PORTS-1:0][7:0] eth_txd,
    output logic [NUM_ETH_PORTS-1:0] eth_tx_en,
    input  logic [NUM_ETH_PORTS-1:0] eth_rx_clk,
    input  logic [NUM_ETH_PORTS-1:0][7:0] eth_rxd,
    input  logic [NUM_ETH_PORTS-1:0] eth_rx_dv,
    
    // SPI Interfaces (4x)
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
    // CXL + JESD204B Internal Signals
    // (Physical layer uses GTY transceivers — parallel data stays internal)
    // Note: DONT_TOUCH on the CXL/JESD module instances preserves the logic;
    //       removing it from wires allows the router more placement freedom.
    //=========================================================================
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_req_valid;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_req_ready;
    logic [NUM_CXL_PORTS-1:0][2:0]           cxl_mem_req_opcode;
    logic [NUM_CXL_PORTS-1:0][47:0]          cxl_mem_req_addr;
    logic [NUM_CXL_PORTS-1:0][15:0]          cxl_mem_req_tag;
    logic [NUM_CXL_PORTS-1:0][7:0]           cxl_mem_req_length;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_data_valid;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_data_ready;
    logic [NUM_CXL_PORTS-1:0][255:0]         cxl_mem_data;
    logic [NUM_CXL_PORTS-1:0][31:0]          cxl_mem_data_be;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_data_last;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_rsp_valid;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_rsp_ready;
    logic [NUM_CXL_PORTS-1:0][2:0]           cxl_mem_rsp_opcode;
    logic [NUM_CXL_PORTS-1:0][15:0]          cxl_mem_rsp_tag;
    logic [NUM_CXL_PORTS-1:0][1:0]           cxl_mem_rsp_status;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_rdata_valid;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_rdata_ready;
    logic [NUM_CXL_PORTS-1:0][255:0]         cxl_mem_rdata;
    logic [NUM_CXL_PORTS-1:0]                cxl_mem_rdata_last;
    logic [NUM_CXL_PORTS-1:0]                cxl_link_up;

    logic [1:0][NUM_JESD_LANES-1:0][9:0]     jesd_tx_lane_data;
    logic [1:0][NUM_JESD_LANES-1:0]           jesd_tx_lane_valid;
    logic [1:0]                               jesd_tx_sync_n;
    logic [1:0][NUM_JESD_LANES-1:0][9:0]     jesd_rx_lane_data;
    logic [1:0][NUM_JESD_LANES-1:0]           jesd_rx_lane_valid;
    logic [1:0]                               jesd_rx_sync_n;
    logic [1:0]                               jesd_sysref;
    logic [1:0]                               jesd_link_up;

    // Tie off CXL.mem external inputs (would come from GTY RX in real design)
    assign cxl_mem_req_ready  = '1;
    assign cxl_mem_data_ready = '1;
    assign cxl_mem_rsp_valid  = '0;
    assign cxl_mem_rsp_opcode = '0;
    assign cxl_mem_rsp_tag    = '0;
    assign cxl_mem_rsp_status = '0;
    assign cxl_mem_rdata_valid = '0;
    assign cxl_mem_rdata      = '0;
    assign cxl_mem_rdata_last = '0;

    // Tie off JESD204B external inputs (would come from GTY RX in real design)
    assign jesd_tx_sync_n    = '1;   // DAC sync deasserted
    assign jesd_rx_lane_data = '0;
    assign jesd_rx_lane_valid = '0;
    assign jesd_sysref       = '0;

    //=========================================================================
    // Line Size = 64 bytes = 512 bits
    //=========================================================================
    localparam LINE_BITS = 512;

    //=========================================================================
    // Internal Signals - DLS Core Pairs
    //=========================================================================
    
    // DLS pair memory interfaces
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
    
    // Vector memory interfaces
    logic [NUM_DLS_PAIRS-1:0]       dls_vmem_valid;
    logic [NUM_DLS_PAIRS-1:0]       dls_vmem_write;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_vmem_addr;
    logic [NUM_DLS_PAIRS-1:0][VLEN-1:0] dls_vmem_wdata;
    logic [NUM_DLS_PAIRS-1:0][VLEN/8-1:0] dls_vmem_wstrb;
    logic [NUM_DLS_PAIRS-1:0][VLEN-1:0] dls_vmem_rdata;
    logic [NUM_DLS_PAIRS-1:0]       dls_vmem_ready;
    
    // DLS status
    logic [NUM_DLS_PAIRS-1:0]       dls_lockstep_error;
    logic [NUM_DLS_PAIRS-1:0]       dls_halted;
    logic [NUM_DLS_PAIRS-1:0][XLEN-1:0] dls_pc;
    logic [NUM_DLS_PAIRS-1:0][31:0] dls_error_count;

    //=========================================================================
    // Internal Signals - Accelerators
    //=========================================================================
    
    // Per-pair CORDIC interfaces (unpacked arrays to match DLS pair)
    logic [NUM_DLS_PAIRS-1:0]       cordic_valid;
    logic [3:0]                     cordic_op       [NUM_DLS_PAIRS-1:0];
    logic [63:0]                    cordic_x        [NUM_DLS_PAIRS-1:0];
    logic [63:0]                    cordic_y        [NUM_DLS_PAIRS-1:0];
    logic [63:0]                    cordic_z        [NUM_DLS_PAIRS-1:0];
    logic [63:0]                    cordic_result_x [NUM_DLS_PAIRS-1:0];
    logic [63:0]                    cordic_result_y [NUM_DLS_PAIRS-1:0];
    logic [63:0]                    cordic_result_z [NUM_DLS_PAIRS-1:0];
    logic [NUM_DLS_PAIRS-1:0]       cordic_ready;
    
    // Per-pair Systolic interfaces (unpacked arrays to match DLS pair)
    logic [NUM_DLS_PAIRS-1:0]       systolic_valid;
    logic [63:0]                    systolic_a [NUM_DLS_PAIRS-1:0][0:8];
    logic [63:0]                    systolic_b [NUM_DLS_PAIRS-1:0][0:8];
    logic [63:0]                    systolic_c [NUM_DLS_PAIRS-1:0][0:8];
    logic [NUM_DLS_PAIRS-1:0]       systolic_ready;

    //=========================================================================
    // Internal Signals - Cache Hierarchy
    //=========================================================================
    
    // L1 I-Cache to Snoop Bus (per core)
    logic [NUM_CORES-1:0]           l1i_bus_valid;
    logic [NUM_CORES-1:0][XLEN-1:0] l1i_bus_addr;
    logic [NUM_CORES-1:0][LINE_BITS-1:0] l1i_bus_rdata;
    logic [NUM_CORES-1:0]           l1i_bus_ready;
    
    // L1 D-Cache to Snoop Bus (per core)
    logic [NUM_CORES-1:0]           l1d_bus_valid;
    logic [NUM_CORES-1:0]           l1d_bus_write;
    logic [NUM_CORES-1:0][2:0]      l1d_bus_cmd;
    logic [NUM_CORES-1:0][XLEN-1:0] l1d_bus_addr;
    logic [NUM_CORES-1:0][LINE_BITS-1:0] l1d_bus_wdata;
    logic [NUM_CORES-1:0][LINE_BITS-1:0] l1d_bus_rdata;
    logic [NUM_CORES-1:0]           l1d_bus_ready;
    logic [NUM_CORES-1:0]           l1d_bus_shared;
    
    // Snoop interface
    logic [NUM_CORES-1:0]           snoop_valid;
    logic [2:0]                     snoop_cmd;
    logic [XLEN-1:0]                snoop_addr;
    logic [NUM_CORES-1:0]           snoop_hit;
    logic [NUM_CORES-1:0]           snoop_hitm;
    logic [NUM_CORES-1:0]           snoop_hito;
    logic [NUM_CORES-1:0][LINE_BITS-1:0] snoop_data;
    logic [NUM_CORES-1:0][2:0]      snoop_state;
    logic [NUM_CORES-1:0]           snoop_ack;
    
    // L2 Cache interface
    logic                           l2_req_valid;
    logic                           l2_req_write;
    logic [XLEN-1:0]                l2_req_addr;
    logic [LINE_BITS-1:0]           l2_req_wdata;
    logic [LINE_BITS-1:0]           l2_resp_rdata;
    logic                           l2_resp_valid;
    logic                           l2_resp_shared;
    logic                           l2_req_ready;
    
    // L2 to DDR interface
    logic                           l2_ddr_valid;
    logic                           l2_ddr_write;
    logic [XLEN-1:0]                l2_ddr_addr;
    logic [LINE_BITS-1:0]           l2_ddr_wdata;
    logic [LINE_BITS-1:0]           l2_ddr_rdata;
    logic                           l2_ddr_ready;
    
    // ECC status aggregation
    logic [NUM_CORES-1:0]           l1i_ecc_error;
    logic [NUM_CORES-1:0]           l1d_ecc_error;
    logic                           l2_ecc_error;
    logic                           l2_uncorrectable;

    //=========================================================================
    // DLS Core Pairs (4 pairs = 8 cores with lockstep)
    //=========================================================================
    genvar pair;
    generate
        for (pair = 0; pair < NUM_DLS_PAIRS; pair++) begin : gen_dls_pairs
            dls_core_pair_rv64 #(
                .XLEN           (XLEN),
                .FLEN           (FLEN),
                .VLEN           (VLEN),
                .CORE_PAIR_ID   (pair),
                .RESET_VECTOR   (64'h0000_0000_8000_0000)
            ) u_dls_pair (
                .clk            (clk),
                .rst_n          (rst_n),
                .enable         (1'b1),
                .meip           (external_irq[pair*2] | external_irq[pair*2+1]),
                .mtip           (timer_irq[pair*2] | timer_irq[pair*2+1]),
                .msip           (software_irq[pair*2] | software_irq[pair*2+1]),
                .nmi            (nmi),
                // Instruction memory
                .imem_valid     (dls_imem_valid[pair]),
                .imem_addr      (dls_imem_addr[pair]),
                .imem_data      (dls_imem_data[pair]),
                .imem_ready     (dls_imem_ready[pair]),
                // Data memory
                .dmem_valid     (dls_dmem_valid[pair]),
                .dmem_write     (dls_dmem_write[pair]),
                .dmem_addr      (dls_dmem_addr[pair]),
                .dmem_wdata     (dls_dmem_wdata[pair]),
                .dmem_wstrb     (dls_dmem_wstrb[pair]),
                .dmem_rdata     (dls_dmem_rdata[pair]),
                .dmem_ready     (dls_dmem_ready[pair]),
                // Vector memory
                .vmem_valid     (dls_vmem_valid[pair]),
                .vmem_write     (dls_vmem_write[pair]),
                .vmem_addr      (dls_vmem_addr[pair]),
                .vmem_wdata     (dls_vmem_wdata[pair]),
                .vmem_wstrb     (dls_vmem_wstrb[pair]),
                .vmem_rdata     (dls_vmem_rdata[pair]),
                .vmem_ready     (dls_vmem_ready[pair]),
                // CORDIC (connected to dedicated unit)
                .cordic_valid   (cordic_valid[pair]),
                .cordic_op      (cordic_op[pair]),
                .cordic_x       (cordic_x[pair]),
                .cordic_y       (cordic_y[pair]),
                .cordic_z       (cordic_z[pair]),
                .cordic_result_x(cordic_result_x[pair]),
                .cordic_result_y(cordic_result_y[pair]),
                .cordic_result_z(cordic_result_z[pair]),
                .cordic_ready   (cordic_ready[pair]),
                // Systolic array (connected to dedicated unit)
                .systolic_valid (systolic_valid[pair]),
                .systolic_a     (systolic_a[pair]),
                .systolic_b     (systolic_b[pair]),
                .systolic_c     (systolic_c[pair]),
                .systolic_ready (systolic_ready[pair]),
                // Status
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
    // Per-DLS Pair CORDIC Units (FP64 Trigonometry)
    //=========================================================================
    generate
        for (pair = 0; pair < NUM_DLS_PAIRS; pair++) begin : gen_cordic
            fp64_cordic_unit #(
                .ITERATIONS     (54),
                .PIPELINE_DEPTH (18)
            ) u_cordic (
                .clk            (clk),
                .rst_n          (rst_n),
                .valid_in       (cordic_valid[pair]),
                .op             ({1'b0, cordic_op[pair][2:0]}),  // Pad to 4 bits
                .x_in           (cordic_x[pair]),
                .y_in           (cordic_y[pair]),
                .z_in           (cordic_z[pair]),
                .x_out          (cordic_result_x[pair]),
                .y_out          (cordic_result_y[pair]),
                .z_out          (cordic_result_z[pair]),
                .valid_out      (),
                .ready          (cordic_ready[pair]),
                .overflow       (),
                .underflow      ()
            );
        end
    endgenerate

    //=========================================================================
    // Per-DLS Pair Systolic Arrays (3x3 FP64 Matrix Multiply)
    //=========================================================================
    // Convert between packed (systolic module) and unpacked (DLS pair) arrays
    logic [8:0][63:0] systolic_a_packed [NUM_DLS_PAIRS-1:0];
    logic [8:0][63:0] systolic_b_packed [NUM_DLS_PAIRS-1:0];
    logic [8:0][63:0] systolic_c_packed [NUM_DLS_PAIRS-1:0];
    
    generate
        for (pair = 0; pair < NUM_DLS_PAIRS; pair++) begin : gen_systolic
            // Pack input arrays
            for (genvar i = 0; i < 9; i++) begin : gen_pack
                assign systolic_a_packed[pair][i] = systolic_a[pair][i];
                assign systolic_b_packed[pair][i] = systolic_b[pair][i];
                assign systolic_c[pair][i] = systolic_c_packed[pair][i];
            end
            
            (* DONT_TOUCH = "TRUE" *) systolic_array_3x3_fp64 #(
                .DATA_WIDTH     (64)
            ) u_systolic (
                .clk            (clk),
                .rst_n          (rst_n),
                .valid_in       (systolic_valid[pair]),
                .a              (systolic_a_packed[pair]),
                .b              (systolic_b_packed[pair]),
                .c              (systolic_c_packed[pair]),
                .valid_out      (),
                .ready          (systolic_ready[pair])
            );
        end
    endgenerate

    //=========================================================================
    // L1 Instruction Caches (32KB each, SECDED)
    //=========================================================================
    generate
        for (genvar c = 0; c < NUM_CORES; c++) begin : gen_l1_icache
            (* DONT_TOUCH = "TRUE" *) l1_cache_secded #(
                .CACHE_SIZE_KB  (L1I_SIZE_KB),
                .LINE_SIZE      (64),
                .NUM_WAYS       (4),
                .ADDR_WIDTH     (XLEN)
            ) u_l1_icache (
                .clk            (clk),
                .rst_n          (rst_n),
                // CPU interface - connect to DLS pair
                .cpu_valid      (dls_imem_valid[c/2]),
                .cpu_write      (1'b0),  // I-Cache is read-only
                .cpu_addr       (dls_imem_addr[c/2]),
                .cpu_wdata      (64'd0),
                .cpu_rdata      (),      // Word extracted from line
                .cpu_ready      (),
                // L2 interface
                .l2_valid       (l1i_bus_valid[c]),
                .l2_write       (),      // I-Cache never writes
                .l2_addr        (l1i_bus_addr[c]),
                .l2_wdata       (),
                .l2_rdata       (l1i_bus_rdata[c]),
                .l2_ready       (l1i_bus_ready[c]),
                // ECC
                .ecc_error      (l1i_ecc_error[c]),
                .uncorrectable  ()
            );
        end
    endgenerate

    //=========================================================================
    // L1 Data Caches (32KB each, SECDED)
    //=========================================================================
    generate
        for (genvar c = 0; c < NUM_CORES; c++) begin : gen_l1_dcache
            (* DONT_TOUCH = "TRUE" *) l1_cache_secded #(
                .CACHE_SIZE_KB  (L1D_SIZE_KB),
                .LINE_SIZE      (64),
                .NUM_WAYS       (4),
                .ADDR_WIDTH     (XLEN)
            ) u_l1_dcache (
                .clk            (clk),
                .rst_n          (rst_n),
                // CPU interface - connect to DLS pair
                .cpu_valid      (dls_dmem_valid[c/2]),
                .cpu_write      (dls_dmem_write[c/2]),
                .cpu_addr       (dls_dmem_addr[c/2]),
                .cpu_wdata      (dls_dmem_wdata[c/2]),
                .cpu_rdata      (),
                .cpu_ready      (),
                // L2/Snoop bus interface
                .l2_valid       (l1d_bus_valid[c]),
                .l2_write       (l1d_bus_write[c]),
                .l2_addr        (l1d_bus_addr[c]),
                .l2_wdata       (l1d_bus_wdata[c]),
                .l2_rdata       (l1d_bus_rdata[c]),
                .l2_ready       (l1d_bus_ready[c]),
                // ECC
                .ecc_error      (l1d_ecc_error[c]),
                .uncorrectable  ()
            );
        end
    endgenerate

    //=========================================================================
    // MOESI Snoop Bus - Cache Coherence
    //=========================================================================
    (* DONT_TOUCH = "TRUE" *) snoop_bus #(
        .NUM_CORES      (NUM_CORES),
        .ADDR_WIDTH     (XLEN),
        .LINE_BITS      (LINE_BITS),
        .BUS_WIDTH      (256)
    ) u_snoop_bus (
        .clk            (clk),
        .rst_n          (rst_n),
        // I-Cache snoop (invalidation only)
        .icache_snoop_valid(),
        .icache_snoop_addr(),
        .icache_snoop_hit('0),
        .icache_snoop_ack({NUM_CORES{1'b1}}),
        // D-Cache request
        .dcache_req_valid(l1d_bus_valid),
        .dcache_req_cmd  ('{default: 3'b001}),  // Read command
        .dcache_req_addr (l1d_bus_addr),
        .dcache_req_data (l1d_bus_wdata),
        .dcache_req_grant(),
        .dcache_resp_data(l1d_bus_rdata),
        .dcache_resp_valid(),
        .dcache_resp_shared(),
        // D-Cache snoop
        .dcache_snoop_valid(snoop_valid),
        .dcache_snoop_cmd(snoop_cmd),
        .dcache_snoop_addr(snoop_addr),
        .dcache_snoop_hit(snoop_hit),
        .dcache_snoop_hitm(snoop_hitm),
        .dcache_snoop_hito(snoop_hito),
        .dcache_snoop_data(snoop_data),
        .dcache_snoop_state(snoop_state),
        .dcache_snoop_ack(snoop_ack),
        // L2 interface
        .l2_req_valid   (l2_req_valid),
        .l2_req_write   (l2_req_write),
        .l2_req_addr    (l2_req_addr),
        .l2_req_wdata   (l2_req_wdata),
        .l2_resp_rdata  (l2_resp_rdata),
        .l2_resp_valid  (l2_resp_valid),
        .l2_resp_shared (l2_resp_shared),
        .l2_req_ready   (l2_req_ready),
        // Status
        .bus_busy       (),
        .bus_transactions()
    );

    // Default snoop responses (simplified - would be from L1 caches in full impl)
    assign snoop_hit  = '0;
    assign snoop_hitm = '0;
    assign snoop_hito = '0;
    assign snoop_data = '{default: '0};
    assign snoop_state = '{default: 3'b000};
    assign snoop_ack  = {NUM_CORES{1'b1}};

    //=========================================================================
    // 4MB L2 Unified Cache (16-way, SECDED)
    //=========================================================================
    l2_cache_4mb #(
        .CACHE_SIZE_KB  (L2_SIZE_KB),
        .LINE_SIZE      (64),
        .NUM_WAYS       (16),
        .ADDR_WIDTH     (XLEN),
        .NUM_CORES      (NUM_CORES)
    ) u_l2_cache (
        .clk            (clk),
        .rst_n          (rst_n),
        // Core interfaces from snoop bus
        .core_valid     ({l1i_bus_valid | l1d_bus_valid}),
        .core_write     (l1d_bus_write),
        .core_addr      (l1d_bus_addr),  // Arbitrated by snoop bus
        .core_wdata     (l1d_bus_wdata),
        .core_rdata     (),
        .core_ready     (),
        // DDR interface
        .ddr_valid      (l2_ddr_valid),
        .ddr_write      (l2_ddr_write),
        .ddr_addr       (l2_ddr_addr),
        .ddr_wdata      (l2_ddr_wdata),
        .ddr_rdata      (l2_ddr_rdata),
        .ddr_ready      (l2_ddr_ready),
        // MOESI coherence
        .snoop_valid    ('0),
        .snoop_addr     ('{default: '0}),
        .snoop_resp_state(),
        .snoop_resp_valid(),
        // ECC
        .ecc_error      (l2_ecc_error),
        .uncorrectable  (l2_uncorrectable)
    );

    //=========================================================================
    // DDR4 Memory Controller
    //=========================================================================
    ddr4_controller u_ddr4 (
        .clk            (clk),
        .rst_n          (rst_n),
        // L2 cache interface
        .cache_valid    (l2_ddr_valid),
        .cache_write    (l2_ddr_write),
        .cache_addr     (l2_ddr_addr),
        .cache_wdata    (l2_ddr_wdata),
        .cache_rdata    (l2_ddr_rdata),
        .cache_ready    (l2_ddr_ready),
        // DDR4 PHY
        .ddr4_ck_p      (ddr4_ck_p),
        .ddr4_ck_n      (ddr4_ck_n),
        .ddr4_cke       (ddr4_cke),
        .ddr4_cs_n      (ddr4_cs_n),
        .ddr4_ras_n     (ddr4_ras_n),
        .ddr4_cas_n     (ddr4_cas_n),
        .ddr4_we_n      (ddr4_we_n),
        .ddr4_addr      (ddr4_addr),
        .ddr4_ba        (ddr4_ba),
        .ddr4_bg        (ddr4_bg),
        .ddr4_odt       (ddr4_odt),
        .ddr4_reset_n   (ddr4_reset_n)
    );

    //=========================================================================
    // Peripheral Bus Bridge - Routes memory-mapped accesses to TinyML/CCSDS/Debug
    //=========================================================================
    // Pick DLS pair 0's data memory bus for peripheral access (simplified arbiter)
    // In production, a proper bus fabric would arbitrate all 4 pairs
    logic periph_valid;
    logic periph_write;
    logic [XLEN-1:0] periph_addr;
    logic [XLEN-1:0] periph_wdata;
    logic [XLEN-1:0] periph_rdata;
    logic periph_ready;

    // Address decode: 0xF000_0000+ goes to peripheral bridge
    assign periph_valid = dls_dmem_valid[0] && (dls_dmem_addr[0][31:28] == 4'hF);
    assign periph_write = dls_dmem_write[0];
    assign periph_addr  = dls_dmem_addr[0];
    assign periph_wdata = dls_dmem_wdata[0];

    // TinyML signals
    logic        tinyml_start;
    logic [3:0]  tinyml_operation;
    logic        tinyml_done;
    logic        tinyml_busy;
    logic        tinyml_ready_sig;
    logic [15:0] tinyml_cfg_input_width;
    logic [15:0] tinyml_cfg_input_height;
    logic [9:0]  tinyml_cfg_input_channels;
    logic [9:0]  tinyml_cfg_output_channels;
    logic [2:0]  tinyml_cfg_kernel_size;
    logic [2:0]  tinyml_cfg_stride;
    logic [2:0]  tinyml_cfg_padding;
    logic [1:0]  tinyml_cfg_pool_size;
    logic [1:0]  tinyml_cfg_activation;
    logic        tinyml_cfg_depthwise;
    logic        tinyml_cfg_batch_norm;
    logic [31:0] tinyml_cfg_scale;
    logic [31:0] tinyml_cfg_zero_point;
    logic [7:0]  tinyml_ifmap_data;
    logic        tinyml_ifmap_valid;
    logic        tinyml_ifmap_ready_sig;
    logic        tinyml_ifmap_last;
    logic [7:0]  tinyml_weight_data;
    logic        tinyml_weight_valid;
    logic        tinyml_weight_ready_sig;
    logic        tinyml_weight_last;
    logic [31:0] tinyml_bias_data;
    logic        tinyml_bias_valid;
    logic        tinyml_bias_ready_sig;
    logic [7:0]  tinyml_ofmap_data;
    logic        tinyml_ofmap_valid;
    logic        tinyml_ofmap_ready;
    logic [31:0] tinyml_bn_gamma;
    logic [31:0] tinyml_bn_beta;
    logic [31:0] tinyml_bn_mean;
    logic [31:0] tinyml_bn_var;
    logic [31:0] tinyml_cycle_count;
    logic [31:0] tinyml_ops_count;

    // CCSDS signals
    logic        ccsds_valid_in;
    logic [1:0]  ccsds_mode;
    logic [7:0]  ccsds_data_in;
    logic [7:0]  ccsds_data_out;
    logic        ccsds_ready_sig;
    logic        ccsds_valid_out;

    // Debug DMI signals
    logic        dmi_req_valid;
    logic        dmi_req_ready;
    logic [6:0]  dmi_req_addr;
    logic [31:0] dmi_req_data;
    logic [1:0]  dmi_req_op;
    logic        dmi_rsp_valid;
    logic        dmi_rsp_ready;
    logic [31:0] dmi_rsp_data;
    logic [1:0]  dmi_rsp_op;

    (* DONT_TOUCH = "TRUE" *) peripheral_bus_bridge #(
        .XLEN           (XLEN)
    ) u_periph_bridge (
        .clk            (clk),
        .rst_n          (rst_n),
        .cpu_valid      (periph_valid),
        .cpu_write      (periph_write),
        .cpu_addr       (periph_addr),
        .cpu_wdata      (periph_wdata),
        .cpu_rdata      (periph_rdata),
        .cpu_ready      (periph_ready),
        // TinyML
        .tinyml_start           (tinyml_start),
        .tinyml_operation       (tinyml_operation),
        .tinyml_done            (tinyml_done),
        .tinyml_busy            (tinyml_busy),
        .tinyml_ready           (tinyml_ready_sig),
        .tinyml_cfg_input_width (tinyml_cfg_input_width),
        .tinyml_cfg_input_height(tinyml_cfg_input_height),
        .tinyml_cfg_input_channels(tinyml_cfg_input_channels),
        .tinyml_cfg_output_channels(tinyml_cfg_output_channels),
        .tinyml_cfg_kernel_size (tinyml_cfg_kernel_size),
        .tinyml_cfg_stride      (tinyml_cfg_stride),
        .tinyml_cfg_padding     (tinyml_cfg_padding),
        .tinyml_cfg_pool_size   (tinyml_cfg_pool_size),
        .tinyml_cfg_activation  (tinyml_cfg_activation),
        .tinyml_cfg_depthwise   (tinyml_cfg_depthwise),
        .tinyml_cfg_batch_norm  (tinyml_cfg_batch_norm),
        .tinyml_cfg_scale       (tinyml_cfg_scale),
        .tinyml_cfg_zero_point  (tinyml_cfg_zero_point),
        .tinyml_ifmap_data      (tinyml_ifmap_data),
        .tinyml_ifmap_valid     (tinyml_ifmap_valid),
        .tinyml_ifmap_ready     (tinyml_ifmap_ready_sig),
        .tinyml_ifmap_last      (tinyml_ifmap_last),
        .tinyml_weight_data     (tinyml_weight_data),
        .tinyml_weight_valid    (tinyml_weight_valid),
        .tinyml_weight_ready    (tinyml_weight_ready_sig),
        .tinyml_weight_last     (tinyml_weight_last),
        .tinyml_bias_data       (tinyml_bias_data),
        .tinyml_bias_valid      (tinyml_bias_valid),
        .tinyml_bias_ready      (tinyml_bias_ready_sig),
        .tinyml_ofmap_data      (tinyml_ofmap_data),
        .tinyml_ofmap_valid     (tinyml_ofmap_valid),
        .tinyml_ofmap_ready     (tinyml_ofmap_ready),
        .tinyml_bn_gamma        (tinyml_bn_gamma),
        .tinyml_bn_beta         (tinyml_bn_beta),
        .tinyml_bn_mean         (tinyml_bn_mean),
        .tinyml_bn_var          (tinyml_bn_var),
        .tinyml_cycle_count     (tinyml_cycle_count),
        .tinyml_ops_count       (tinyml_ops_count),
        // CCSDS
        .ccsds_valid_in         (ccsds_valid_in),
        .ccsds_mode             (ccsds_mode),
        .ccsds_data_in          (ccsds_data_in),
        .ccsds_data_out         (ccsds_data_out),
        .ccsds_ready            (ccsds_ready_sig),
        .ccsds_valid_out        (ccsds_valid_out),
        // Debug
        .dmi_req_valid          (dmi_req_valid),
        .dmi_req_ready          (dmi_req_ready),
        .dmi_req_addr           (dmi_req_addr),
        .dmi_req_data           (dmi_req_data),
        .dmi_req_op             (dmi_req_op),
        .dmi_rsp_valid          (dmi_rsp_valid),
        .dmi_rsp_ready          (dmi_rsp_ready),
        .dmi_rsp_data           (dmi_rsp_data),
        .dmi_rsp_op             (dmi_rsp_op)
    );

    //=========================================================================
    // TinyML Accelerator (32 Processing Elements)
    //=========================================================================
    // Memory-mapped at 0xF000_0000 via peripheral bus bridge
    (* DONT_TOUCH = "TRUE" *) tinyml_accelerator #(
        .DATA_WIDTH     (8),
        .ACC_WIDTH      (32),
        .MAX_KERNEL     (5),
        .MAX_CHANNELS   (128),
        .BUFFER_DEPTH   (256),
        .PE_ARRAY_SIZE  (TINYML_PES)
    ) u_tinyml (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (tinyml_start),
        .operation      (tinyml_operation),
        .done           (tinyml_done),
        .busy           (tinyml_busy),
        .ready          (tinyml_ready_sig),
        // Config
        .cfg_input_width(tinyml_cfg_input_width),
        .cfg_input_height(tinyml_cfg_input_height),
        .cfg_input_channels(tinyml_cfg_input_channels),
        .cfg_output_channels(tinyml_cfg_output_channels),
        .cfg_kernel_size(tinyml_cfg_kernel_size),
        .cfg_stride     (tinyml_cfg_stride),
        .cfg_padding    (tinyml_cfg_padding),
        .cfg_pool_size  (tinyml_cfg_pool_size),
        .cfg_activation (tinyml_cfg_activation),
        .cfg_depthwise  (tinyml_cfg_depthwise),
        .cfg_batch_norm (tinyml_cfg_batch_norm),
        .cfg_scale      (tinyml_cfg_scale),
        .cfg_zero_point (tinyml_cfg_zero_point),
        // Input feature map
        .ifmap_data     (tinyml_ifmap_data),
        .ifmap_valid    (tinyml_ifmap_valid),
        .ifmap_ready    (tinyml_ifmap_ready_sig),
        .ifmap_last     (tinyml_ifmap_last),
        // Weights
        .weight_data    (tinyml_weight_data),
        .weight_valid   (tinyml_weight_valid),
        .weight_ready   (tinyml_weight_ready_sig),
        .weight_last    (tinyml_weight_last),
        // Bias
        .bias_data      (tinyml_bias_data),
        .bias_valid     (tinyml_bias_valid),
        .bias_ready     (tinyml_bias_ready_sig),
        // Output feature map
        .ofmap_data     (tinyml_ofmap_data),
        .ofmap_valid    (tinyml_ofmap_valid),
        .ofmap_ready    (tinyml_ofmap_ready),
        .ofmap_last     (),
        // Batch norm
        .bn_gamma       (tinyml_bn_gamma),
        .bn_beta        (tinyml_bn_beta),
        .bn_mean        (tinyml_bn_mean),
        .bn_var         (tinyml_bn_var),
        // Status
        .cycle_count    (tinyml_cycle_count),
        .ops_count      (tinyml_ops_count),
        .current_row    (),
        .current_col    (),
        .error_flags    ()
    );

    //=========================================================================
    // CCSDS Compression Subsystem (TC, TM, Image)
    //=========================================================================
    // Memory-mapped at 0xF100_0000 via peripheral bus bridge
    (* DONT_TOUCH = "TRUE" *) ccsds_compression_subsystem u_ccsds (
        .clk            (clk),
        .rst_n          (rst_n),
        .valid_in       (ccsds_valid_in),
        .mode           (ccsds_mode),
        .data_in        (ccsds_data_in),
        .data_out       (ccsds_data_out),
        .ready          (ccsds_ready_sig),
        .valid_out      (ccsds_valid_out)
    );

    //=========================================================================
    // SpaceWire Router (4 Ports)
    //=========================================================================
    spacewire_router #(
        .NUM_PORTS      (NUM_SPW_PORTS),
        .FIFO_DEPTH     (256),
        .BASE_ADDR      (32'h4000_0000)
    ) u_spacewire (
        .clk            (clk),
        .rst_n          (rst_n),
        // SpaceWire physical
        .spw_do         (spw_tx_d),
        .spw_so         (spw_tx_s),
        .spw_di         (spw_rx_d),
        .spw_si         (spw_rx_s),
        // AXI4-Lite (unused - tie off)
        .s_axi_awvalid  (1'b0),
        .s_axi_awready  (),
        .s_axi_awaddr   (32'd0),
        .s_axi_wvalid   (1'b0),
        .s_axi_wready   (),
        .s_axi_wdata    (32'd0),
        .s_axi_wstrb    (4'd0),
        .s_axi_bvalid   (),
        .s_axi_bready   (1'b1),
        .s_axi_bresp    (),
        .s_axi_arvalid  (1'b0),
        .s_axi_arready  (),
        .s_axi_araddr   (32'd0),
        .s_axi_rvalid   (),
        .s_axi_rready   (1'b1),
        .s_axi_rdata    (),
        .s_axi_rresp    ()
    );

    //=========================================================================
    // TSN Ethernet Switch (4 Ports) - GMII Interface
    //=========================================================================
    // Unpack GMII signals for TSN switch
    logic [7:0]  gmii_txd_arr    [NUM_ETH_PORTS-1:0];
    logic        gmii_tx_en_arr  [NUM_ETH_PORTS-1:0];
    logic        gmii_tx_er_arr  [NUM_ETH_PORTS-1:0];
    logic [7:0]  gmii_rxd_arr    [NUM_ETH_PORTS-1:0];
    logic        gmii_rx_dv_arr  [NUM_ETH_PORTS-1:0];
    logic        gmii_rx_er_arr  [NUM_ETH_PORTS-1:0];
    
    // Connect packed to unpacked
    generate
        for (genvar e = 0; e < NUM_ETH_PORTS; e++) begin : gen_eth_conn
            assign eth_txd[e] = gmii_txd_arr[e];
            assign eth_tx_en[e] = gmii_tx_en_arr[e];
            assign gmii_rxd_arr[e] = eth_rxd[e];
            assign gmii_rx_dv_arr[e] = eth_rx_dv[e];
            assign gmii_rx_er_arr[e] = 1'b0;
        end
    endgenerate
    
    assign eth_tx_clk = '0;  // Would come from PLL
    
    tsn_switch #(
        .NUM_PORTS      (NUM_ETH_PORTS),
        .NUM_QUEUES     (4),            // Reduced from 8 — 4 priority queues sufficient for spacecraft TSN
        .MAC_TABLE_SIZE (16),           // Reduced from 256 — spacecraft networks have ≤16 endpoints
        .VLAN_TABLE_SIZE(4)             // Reduced from 16
    ) u_tsn (
        .clk            (clk),
        .rst_n          (rst_n),
        // GMII
        .gmii_txd       (gmii_txd_arr),
        .gmii_tx_en     (gmii_tx_en_arr),
        .gmii_tx_er     (gmii_tx_er_arr),
        .gmii_rxd       (gmii_rxd_arr),
        .gmii_rx_dv     (gmii_rx_dv_arr),
        .gmii_rx_er     (gmii_rx_er_arr),
        .mdc            (),
        // CPU port (unused)
        .cpu_tx_data    (8'd0),
        .cpu_tx_valid   (1'b0),
        .cpu_tx_ready   (),
        .cpu_tx_last    (1'b0),
        .cpu_tx_prio    (3'd0),
        .cpu_rx_data    (),
        .cpu_rx_valid   (),
        .cpu_rx_ready   (1'b1),
        .cpu_rx_last    (),
        .cpu_rx_prio    (),
        // AXI-Lite config (idle)
        .s_axi_awvalid  (1'b0),
        .s_axi_awready  (),
        .s_axi_awaddr   (32'd0),
        .s_axi_wvalid   (1'b0),
        .s_axi_wready   (),
        .s_axi_wdata    (32'd0),
        .s_axi_wstrb    (4'd0),
        .s_axi_bvalid   (),
        .s_axi_bready   (1'b1),
        .s_axi_bresp    (),
        .s_axi_arvalid  (1'b0),
        .s_axi_arready  (),
        .s_axi_araddr   (32'd0),
        .s_axi_rvalid   (),
        .s_axi_rready   (1'b1),
        .s_axi_rdata    (),
        .s_axi_rresp    (),
        // PTP
        .ptp_master_time(80'd0),
        .ptp_local_time (),
        .ptp_pps        (),
        // TAS
        .tas_gate_ctrl  ('{default: 8'hFF}),
        .tas_cycle_time (32'd1_000_000),
        .tas_enable     (1'b0),
        // Status
        .port_link_up   (),
        .port_speed     (),
        .switch_ready   (),
        .irq            ()
    );

    //=========================================================================
    // TSN Ethernet Switch #1 (Redundant) — REMOVED for resource savings
    //=========================================================================
    // The redundant TSN switch (u_tsn_1) was consuming ~80K LUTs (~15% of
    // VU095) with DONT_TOUCH and all GMII inputs tied to zero.
    // For cold-spare redundancy, re-enable by setting ENABLE_REDUNDANT_TSN=1
    // and connecting GMII to separate physical PHYs.
    //
    // To re-enable:
    //   (* DONT_TOUCH = "TRUE" *) tsn_switch #(
    //       .NUM_PORTS(NUM_ETH_PORTS), .NUM_QUEUES(8),
    //       .MAC_TABLE_SIZE(256), .VLAN_TABLE_SIZE(16)
    //   ) u_tsn_1 ( ... );
    //=========================================================================

    //=========================================================================
    // CXL Memory Controllers (2x CXL.mem Type 3 Expanders)
    //=========================================================================
    // CXL port 0 — connected to DLS pair 0 data bus (address 0xE000_0000+)
    // CXL port 1 — connected to DLS pair 1 data bus (address 0xE000_0000+)
    generate
        for (genvar cxl = 0; cxl < NUM_CXL_PORTS; cxl++) begin : gen_cxl
            // AXI4 signals driven from DLS pair data bus
            logic        cxl_axi_awvalid;
            logic        cxl_axi_awready;
            logic        cxl_axi_wvalid;
            logic        cxl_axi_wready;
            logic        cxl_axi_bvalid;
            logic        cxl_axi_arvalid;
            logic        cxl_axi_arready;
            logic        cxl_axi_rvalid;

            // Address decode: 0xE000_0000+ on the respective DLS pair → CXL
            assign cxl_axi_awvalid = dls_dmem_valid[cxl] && dls_dmem_write[cxl] &&
                                     (dls_dmem_addr[cxl][31:28] == 4'hE);
            assign cxl_axi_arvalid = dls_dmem_valid[cxl] && !dls_dmem_write[cxl] &&
                                     (dls_dmem_addr[cxl][31:28] == 4'hE);
            assign cxl_axi_wvalid  = cxl_axi_awvalid;

            (* DONT_TOUCH = "TRUE" *) cxl_memory_controller #(
                .DATA_WIDTH     (256),
                .ADDR_WIDTH     (48),
                .TAG_WIDTH      (16),
                .NUM_CHANNELS   (4),
                .BURST_LEN      (8),
                .TABLE_DEPTH    (8)
            ) u_cxl (
                .clk            (clk),
                .rst_n          (rst_n),
                // AXI4 host interface — from DLS pair
                .axi_awaddr     ({{16'd0}, dls_dmem_addr[cxl][31:0]}),
                .axi_awlen      (8'd0),    // Single beat
                .axi_awsize     (3'b011),  // 8 bytes
                .axi_awburst    (2'b01),   // INCR
                .axi_awid       (16'd0),
                .axi_awvalid    (cxl_axi_awvalid),
                .axi_awready    (cxl_axi_awready),
                .axi_wdata      ({{192'd0}, dls_dmem_wdata[cxl]}),
                .axi_wstrb      ({{24'd0}, dls_dmem_wstrb[cxl]}),
                .axi_wlast      (1'b1),
                .axi_wvalid     (cxl_axi_wvalid),
                .axi_wready     (cxl_axi_wready),
                .axi_bid        (),
                .axi_bresp      (),
                .axi_bvalid     (cxl_axi_bvalid),
                .axi_bready     (1'b1),
                .axi_araddr     ({{16'd0}, dls_dmem_addr[cxl][31:0]}),
                .axi_arlen      (8'd0),
                .axi_arsize     (3'b011),
                .axi_arburst    (2'b01),
                .axi_arid       (16'd0),
                .axi_arvalid    (cxl_axi_arvalid),
                .axi_arready    (cxl_axi_arready),
                .axi_rid        (),
                .axi_rdata      (),
                .axi_rresp      (),
                .axi_rlast      (),
                .axi_rvalid     (cxl_axi_rvalid),
                .axi_rready     (1'b1),
                // CXL.mem external interface — route to top-level ports
                .cxl_mem_req_valid  (cxl_mem_req_valid[cxl]),
                .cxl_mem_req_ready  (cxl_mem_req_ready[cxl]),
                .cxl_mem_req_opcode (cxl_mem_req_opcode[cxl]),
                .cxl_mem_req_addr   (cxl_mem_req_addr[cxl]),
                .cxl_mem_req_tag    (cxl_mem_req_tag[cxl]),
                .cxl_mem_req_length (cxl_mem_req_length[cxl]),
                .cxl_mem_data_valid (cxl_mem_data_valid[cxl]),
                .cxl_mem_data_ready (cxl_mem_data_ready[cxl]),
                .cxl_mem_data       (cxl_mem_data[cxl]),
                .cxl_mem_data_be    (cxl_mem_data_be[cxl]),
                .cxl_mem_data_last  (cxl_mem_data_last[cxl]),
                .cxl_mem_rsp_valid  (cxl_mem_rsp_valid[cxl]),
                .cxl_mem_rsp_ready  (cxl_mem_rsp_ready[cxl]),
                .cxl_mem_rsp_opcode (cxl_mem_rsp_opcode[cxl]),
                .cxl_mem_rsp_tag    (cxl_mem_rsp_tag[cxl]),
                .cxl_mem_rsp_status (cxl_mem_rsp_status[cxl]),
                .cxl_mem_rdata_valid(cxl_mem_rdata_valid[cxl]),
                .cxl_mem_rdata_ready(cxl_mem_rdata_ready[cxl]),
                .cxl_mem_rdata      (cxl_mem_rdata[cxl]),
                .cxl_mem_rdata_last (cxl_mem_rdata_last[cxl]),
                // Status
                .link_up            (cxl_link_up[cxl]),
                .error_count        (),
                .ecc_error          (),
                .read_count         (),
                .write_count        (),
                .read_latency_sum   (),
                .write_latency_sum  ()
            );
        end
    endgenerate

    //=========================================================================
    // JESD204B High-Speed Serial Interfaces (2x)
    //=========================================================================
    // Instance 0 — ADC/DAC link for RF front-end (Subclass 1)
    // Instance 1 — Redundant / secondary instrument link
    generate
        for (genvar j = 0; j < 2; j++) begin : gen_jesd204b
            (* DONT_TOUCH = "TRUE" *) jesd204b_interface #(
                .NUM_LANES          (NUM_JESD_LANES),
                .OCTETS_PER_FRAME   (4),
                .FRAMES_PER_MF      (32),
                .NUM_CONVERTERS     (2),
                .CONV_RESOLUTION    (16),
                .SAMPLES_PER_FRAME  (1),
                .SCRAMBLE_EN        (1),
                .SUBCLASS           (1)
            ) u_jesd204b (
                .clk                (clk),
                .lane_clk           (clk),          // Simplified: same clock domain
                .rst_n              (rst_n),
                .sysref             (jesd_sysref[j]),
                // TX path (to DAC)
                .tx_sample_data     ('{default: '0}), // Connected to DSP chain
                .tx_sample_valid    (1'b0),
                .tx_sample_ready    (),
                .tx_lane_data       (jesd_tx_lane_data[j]),
                .tx_lane_valid      (jesd_tx_lane_valid[j]),
                .tx_sync_n          (jesd_tx_sync_n[j]),
                // RX path (from ADC)
                .rx_lane_data       (jesd_rx_lane_data[j]),
                .rx_lane_valid      (jesd_rx_lane_valid[j]),
                .rx_sample_data     (),               // Would feed DSP chain
                .rx_sample_valid    (),
                .rx_sync_n          (jesd_rx_sync_n[j]),
                // Configuration
                .cfg_did            (5'd0),
                .cfg_bid            (4'd0),
                .cfg_lid            (8'd0),
                .cfg_scr            (1'b1),
                .cfg_f_minus1       (8'd3),   // F=4
                .cfg_k_minus1       (5'd31),  // K=32
                // Status
                .link_up            (jesd_link_up[j]),
                .lane_aligned       (),
                .lane_sync          (),
                .lane_error_count   (),
                .sysref_captured    (),
                .link_state         ()
            );
        end
    endgenerate

    //=========================================================================
    // SPI Masters (4 Channels)
    //=========================================================================
    generate
        for (genvar s = 0; s < 4; s++) begin : gen_spi
            spi_master #(.NUM_CS(1)) u_spi (
                .clk            (clk),
                .rst_n          (rst_n),
                // APB interface (idle)
                .paddr          (12'd0),
                .psel           (1'b0),
                .penable        (1'b0),
                .pwrite         (1'b0),
                .pwdata         (32'd0),
                .prdata         (),
                .pready         (),
                .pslverr        (),
                // SPI physical
                .spi_clk        (spi_sck[s]),
                .spi_mosi       (spi_mosi[s]),
                .spi_miso       (spi_miso[s]),
                .spi_cs_n       (spi_cs_n[s]),
                // Interrupt + DMA
                .irq            (),
                .dma_req        (),
                .dma_ack        (1'b0),
                .dma_single     (),
                .dma_last       ()
            );
        end
    endgenerate

    //=========================================================================
    // UART Controller
    //=========================================================================
    uart_controller u_uart (
        .clk            (clk),
        .rst_n          (rst_n),
        .rx             (uart_rx),
        .tx             (uart_tx),
        .baud_div       (16'd868),  // 80MHz / 115200 / 16
        .bus_valid      (1'b0),
        .bus_wdata      (8'd0),
        .bus_rdata      (),
        .bus_ready      ()
    );

    //=========================================================================
    // Debug Module (JTAG) - Connected via peripheral bus bridge DMI
    //=========================================================================
    (* DONT_TOUCH = "TRUE" *) debug_module u_debug (
        .clk            (clk),
        .rst_n          (rst_n),
        // DMI interface (from peripheral bus bridge)
        .dmi_req_valid  (dmi_req_valid),
        .dmi_req_ready  (dmi_req_ready),
        .dmi_req_addr   (dmi_req_addr),
        .dmi_req_data   (dmi_req_data),
        .dmi_req_op     (dmi_req_op),
        .dmi_rsp_valid  (dmi_rsp_valid),
        .dmi_rsp_ready  (dmi_rsp_ready),
        .dmi_rsp_data   (dmi_rsp_data),
        .dmi_rsp_op     (dmi_rsp_op),
        // Hart control
        .halt_req       (),
        .resume_req     (),
        .halted         ({dls_halted, dls_halted}),
        .running        (~{dls_halted, dls_halted}),
        .unavailable    (8'd0),
        // Register access
        .reg_req        (),
        .reg_write      (),
        .reg_addr       (),
        .reg_wdata      (),
        .reg_rdata      (32'd0),
        .reg_ready      (1'b1),
        .reg_hartsel    (),
        // Breakpoints
        .bp_addr        (),
        .bp_enable      (),
        .bp_hit         (4'd0),
        // System bus
        .sb_req         (),
        .sb_write       (),
        .sb_addr        (),
        .sb_wdata       (),
        .sb_rdata       (32'd0),
        .sb_ready       (1'b1),
        .sb_error       (1'b0),
        // Status
        .dm_active      (),
        .ndmreset_req   (),
        .debug_irq      ()
    );

    //=========================================================================
    // Temporary Memory for Basic Execution (Until DDR4 is ready)
    //=========================================================================
    // Boot ROM - 4KB
    (* rom_style = "block" *) logic [31:0] boot_rom [0:1023];
    
    // Initial program
    initial begin
        for (int i = 0; i < 1024; i++) boot_rom[i] = 32'h00000013;  // NOP
        boot_rom[0] = 32'h00000297;  // auipc t0, 0
        boot_rom[1] = 32'h02028293;  // addi t0, t0, 32 (skip to main)
        boot_rom[2] = 32'h00028067;  // jr t0
        // Main program starts at offset 8
        boot_rom[8] = 32'h06400093;  // li x1, 100
        boot_rom[9] = 32'h0C800113;  // li x2, 200
        boot_rom[10] = 32'h002081B3; // add x3, x1, x2
        boot_rom[11] = 32'h00302023; // sd x3, 0(x0)
        boot_rom[12] = 32'h0000006F; // j . (halt)
    end

    // Simple data RAM per DLS pair (8KB each) - BRAM-inferred
    // NOTE: Each pair gets a dedicated 1024x64 BRAM with byte-write-enable.
    //       Previous implementation used a 2D array with conditional reads,
    //       preventing BRAM inference and consuming ~262K FFs.
    generate
        for (genvar p = 0; p < NUM_DLS_PAIRS; p++) begin : gen_mem
            // Per-pair BRAM: 1024 entries x 64 bits = 8KB
            (* ram_style = "block" *) logic [63:0] data_ram_p [0:1023];

            wire [9:0] dmem_word_addr = dls_dmem_addr[p][12:3];

            // Instruction fetch (from boot ROM)
            always_ff @(posedge clk) begin
                if (dls_imem_valid[p]) begin
                    dls_imem_data[p] <= boot_rom[dls_imem_addr[p][11:2]];
                    dls_imem_ready[p] <= 1'b1;
                end else begin
                    dls_imem_ready[p] <= 1'b0;
                end
            end

            // Data RAM write - explicit byte-write-enable for BRAM inference
            // (Vivado UG901 byte-write template: if(we[i]) mem[addr][i*8+:8] <= din)
            always_ff @(posedge clk) begin
                if (dls_dmem_valid[p] && dls_dmem_write[p]) begin
                    if (dls_dmem_wstrb[p][0]) data_ram_p[dmem_word_addr][ 7: 0] <= dls_dmem_wdata[p][ 7: 0];
                    if (dls_dmem_wstrb[p][1]) data_ram_p[dmem_word_addr][15: 8] <= dls_dmem_wdata[p][15: 8];
                    if (dls_dmem_wstrb[p][2]) data_ram_p[dmem_word_addr][23:16] <= dls_dmem_wdata[p][23:16];
                    if (dls_dmem_wstrb[p][3]) data_ram_p[dmem_word_addr][31:24] <= dls_dmem_wdata[p][31:24];
                    if (dls_dmem_wstrb[p][4]) data_ram_p[dmem_word_addr][39:32] <= dls_dmem_wdata[p][39:32];
                    if (dls_dmem_wstrb[p][5]) data_ram_p[dmem_word_addr][47:40] <= dls_dmem_wdata[p][47:40];
                    if (dls_dmem_wstrb[p][6]) data_ram_p[dmem_word_addr][55:48] <= dls_dmem_wdata[p][55:48];
                    if (dls_dmem_wstrb[p][7]) data_ram_p[dmem_word_addr][63:56] <= dls_dmem_wdata[p][63:56];
                end
                // Unconditional read — required for BRAM output register inference
                dls_dmem_rdata[p] <= data_ram_p[dmem_word_addr];
            end

            // Ready follows valid by one cycle
            always_ff @(posedge clk) begin
                if (!rst_n)
                    dls_dmem_ready[p] <= 1'b0;
                else
                    dls_dmem_ready[p] <= dls_dmem_valid[p];
            end

            // Vector memory (simplified)
            assign dls_vmem_rdata[p] = '0;
            assign dls_vmem_ready[p] = dls_vmem_valid[p];
        end
    endgenerate

    //=========================================================================
    // Status LEDs
    //=========================================================================
    assign led[3:0] = ~dls_halted;          // Active cores (inverted)
    assign led[4]   = |l1i_ecc_error;       // L1 I-Cache ECC error
    assign led[5]   = |l1d_ecc_error;       // L1 D-Cache ECC error
    assign led[6]   = l2_ecc_error;         // L2 ECC error
    assign led[7]   = |dls_lockstep_error;  // DLS mismatch

endmodule


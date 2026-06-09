//============================================================================
// Cache Coherence Unit - Top Level Integration
// Integrates L1 I$, L1 D$, L2$, Snoop Bus, AMO, and Memory Barriers
//============================================================================

`timescale 1ns / 1ps

module cache_coherence_unit #(
    parameter NUM_CORES       = 8,
    parameter L1I_SIZE_KB     = 16,
    parameter L1D_SIZE_KB     = 16,
    parameter L2_SIZE_KB      = 2048,   // 2MB L2 cache (fits in KU060 BRAM)
    parameter LINE_SIZE_BYTES = 64,
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 32
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Core Instruction Fetch Interfaces
    input  logic [NUM_CORES-1:0]                    core_ifetch_valid,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    core_ifetch_addr,
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    core_ifetch_data,
    output logic [NUM_CORES-1:0]                    core_ifetch_ready,
    output logic [NUM_CORES-1:0]                    core_ifetch_stall,
    
    // Core Data Memory Interfaces
    input  logic [NUM_CORES-1:0]                    core_dmem_valid,
    input  logic [NUM_CORES-1:0]                    core_dmem_write,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    core_dmem_addr,
    input  logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    core_dmem_wdata,
    input  logic [NUM_CORES-1:0][3:0]               core_dmem_wmask,
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    core_dmem_rdata,
    output logic [NUM_CORES-1:0]                    core_dmem_ready,
    output logic [NUM_CORES-1:0]                    core_dmem_stall,
    
    // AMO Interfaces (per core)
    input  logic [NUM_CORES-1:0]                    core_amo_valid,
    input  logic [NUM_CORES-1:0][4:0]               core_amo_op,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    core_amo_addr,
    input  logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    core_amo_wdata,
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    core_amo_rdata,
    output logic [NUM_CORES-1:0]                    core_amo_done,
    
    // LR/SC Interfaces (per core)
    input  logic [NUM_CORES-1:0]                    core_lr_valid,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    core_lr_addr,
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    core_lr_rdata,
    output logic [NUM_CORES-1:0]                    core_lr_done,
    input  logic [NUM_CORES-1:0]                    core_sc_valid,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    core_sc_addr,
    input  logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    core_sc_wdata,
    output logic [NUM_CORES-1:0]                    core_sc_success,
    output logic [NUM_CORES-1:0]                    core_sc_done,
    
    // FENCE Interfaces (per core)
    input  logic [NUM_CORES-1:0]                    core_fence_req,
    input  logic [NUM_CORES-1:0][3:0]               core_fence_pred,
    input  logic [NUM_CORES-1:0][3:0]               core_fence_succ,
    output logic [NUM_CORES-1:0]                    core_fence_done,
    input  logic [NUM_CORES-1:0]                    core_fence_i_req,
    output logic [NUM_CORES-1:0]                    core_fence_i_done,
    
    // Main Memory Interface
    output logic                    mem_req_valid,
    output logic                    mem_req_write,
    output logic [ADDR_WIDTH-1:0]   mem_req_addr,
    output logic [LINE_SIZE_BYTES*8-1:0] mem_req_wdata,
    input  logic [LINE_SIZE_BYTES*8-1:0] mem_resp_rdata,
    input  logic                    mem_resp_valid,
    output logic                    mem_req_ready,
    
    // Status
    output logic                    cache_system_ready,
    output logic [31:0]             total_l1i_hits,
    output logic [31:0]             total_l1d_hits,
    output logic [31:0]             total_l2_hits,
    output logic [31:0]             bus_transactions
);

    localparam LINE_BITS = LINE_SIZE_BYTES * 8;

    // L1 I-Cache to L2 interface signals
    logic [NUM_CORES-1:0]                   l1i_l2_req_valid;
    logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]   l1i_l2_req_addr;
    logic [NUM_CORES-1:0][LINE_BITS-1:0]    l1i_l2_resp_data;
    logic [NUM_CORES-1:0]                   l1i_l2_resp_valid;
    
    // L1 D-Cache to Snoop Bus interface signals
    logic [NUM_CORES-1:0]                   dcache_bus_req_valid;
    logic [NUM_CORES-1:0][2:0]              dcache_bus_req_cmd;
    logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]   dcache_bus_req_addr;
    logic [NUM_CORES-1:0][LINE_BITS-1:0]    dcache_bus_req_data;
    logic [NUM_CORES-1:0]                   dcache_bus_req_grant;
    logic [NUM_CORES-1:0][LINE_BITS-1:0]    dcache_bus_resp_data;
    logic [NUM_CORES-1:0]                   dcache_bus_resp_valid;
    logic [NUM_CORES-1:0]                   dcache_bus_resp_shared;
    
    // Snoop signals
    logic [NUM_CORES-1:0]                   icache_snoop_valid;
    logic [ADDR_WIDTH-1:0]                  icache_snoop_addr;
    logic [NUM_CORES-1:0]                   icache_snoop_hit;
    logic [NUM_CORES-1:0]                   icache_snoop_ack;
    
    logic [NUM_CORES-1:0]                   dcache_snoop_valid;
    logic [2:0]                             dcache_snoop_cmd;
    logic [ADDR_WIDTH-1:0]                  dcache_snoop_addr;
    logic [NUM_CORES-1:0]                   dcache_snoop_hit;
    logic [NUM_CORES-1:0]                   dcache_snoop_hitm;
    logic [NUM_CORES-1:0][LINE_BITS-1:0]    dcache_snoop_data;
    logic [NUM_CORES-1:0]                   dcache_snoop_ack;
    
    // L2 interface
    logic                   l2_req_valid;
    logic                   l2_req_write;
    logic [ADDR_WIDTH-1:0]  l2_req_addr;
    logic [LINE_BITS-1:0]   l2_req_wdata;
    logic [LINE_BITS-1:0]   l2_resp_rdata;
    logic                   l2_resp_valid;
    logic                   l2_resp_shared;
    logic                   l2_req_ready;
    
    // Per-core ready signals
    logic [NUM_CORES-1:0] l1i_ready;
    logic [NUM_CORES-1:0] l1d_ready;

    //------------------------------------------------------------------------
    // Generate L1 Caches per Core
    //------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i++) begin : gen_l1_caches
            // L1 Instruction Cache - BRAM version for proper inference
            l1_icache_bram #(
                .CACHE_SIZE_KB(L1I_SIZE_KB),
                .LINE_SIZE_BYTES(LINE_SIZE_BYTES),
                .NUM_WAYS(4),
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(DATA_WIDTH),
                .CORE_ID(i)
            ) u_l1i (
                .clk(clk),
                .rst_n(rst_n),
                .cpu_req_valid(core_ifetch_valid[i]),
                .cpu_req_addr(core_ifetch_addr[i]),
                .cpu_resp_data(core_ifetch_data[i]),
                .cpu_resp_valid(core_ifetch_ready[i]),
                .cpu_stall(core_ifetch_stall[i]),
                .l2_req_valid(l1i_l2_req_valid[i]),
                .l2_req_addr(l1i_l2_req_addr[i]),
                .l2_resp_data(l1i_l2_resp_data[i]),
                .l2_resp_valid(l1i_l2_resp_valid[i]),
                .l2_req_ready(),
                .snoop_valid(icache_snoop_valid[i]),
                .snoop_addr(icache_snoop_addr),
                .snoop_hit(icache_snoop_hit[i]),
                .snoop_ack(icache_snoop_ack[i]),
                .fence_i(core_fence_i_req[i]),
                .fence_complete(core_fence_i_done[i]),
                .cache_ready(l1i_ready[i])
            );
            
            // L1 Data Cache - BRAM version for proper inference
            l1_dcache_bram #(
                .CACHE_SIZE_KB(L1D_SIZE_KB),
                .LINE_SIZE_BYTES(LINE_SIZE_BYTES),
                .NUM_WAYS(4),
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(DATA_WIDTH),
                .CORE_ID(i)
            ) u_l1d (
                .clk(clk),
                .rst_n(rst_n),
                .cpu_req_valid(core_dmem_valid[i]),
                .cpu_req_write(core_dmem_write[i]),
                .cpu_req_addr(core_dmem_addr[i]),
                .cpu_req_wdata(core_dmem_wdata[i]),
                .cpu_req_wmask(core_dmem_wmask[i]),
                .cpu_resp_rdata(core_dmem_rdata[i]),
                .cpu_resp_valid(core_dmem_ready[i]),
                .cpu_stall(core_dmem_stall[i]),
                .amo_req_valid(core_amo_valid[i]),
                .amo_op(core_amo_op[i]),
                .amo_addr(core_amo_addr[i]),
                .amo_wdata(core_amo_wdata[i]),
                .amo_rdata(core_amo_rdata[i]),
                .amo_done(core_amo_done[i]),
                .bus_req_valid(dcache_bus_req_valid[i]),
                .bus_req_cmd(dcache_bus_req_cmd[i]),
                .bus_req_addr(dcache_bus_req_addr[i]),
                .bus_req_data(dcache_bus_req_data[i]),
                .bus_resp_data(dcache_bus_resp_data[i]),
                .bus_resp_valid(dcache_bus_resp_valid[i]),
                .bus_resp_shared(dcache_bus_resp_shared[i]),
                .bus_req_ready(),
                .snoop_req_valid(dcache_snoop_valid[i]),
                .snoop_req_cmd(dcache_snoop_cmd),
                .snoop_req_addr(dcache_snoop_addr),
                .snoop_resp_hit(dcache_snoop_hit[i]),
                .snoop_resp_hitm(dcache_snoop_hitm[i]),
                .snoop_resp_data(dcache_snoop_data[i]),
                .snoop_resp_ack(dcache_snoop_ack[i]),
                .fence(core_fence_req[i]),
                .fence_complete(core_fence_done[i]),
                .cache_ready(l1d_ready[i])
            );
            
            // Simple LR/SC through D-cache AMO interface
            assign core_lr_rdata[i] = core_amo_rdata[i];
            assign core_lr_done[i] = core_lr_valid[i] && core_amo_done[i];
            assign core_sc_success[i] = 1'b1; // Simplified
            assign core_sc_done[i] = core_sc_valid[i] && core_amo_done[i];
        end
    endgenerate

    //------------------------------------------------------------------------
    // Snoop Bus
    //------------------------------------------------------------------------
    snoop_bus #(
        .NUM_CORES(NUM_CORES),
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BITS(LINE_BITS)
    ) u_snoop_bus (
        .clk(clk),
        .rst_n(rst_n),
        // I-Cache snoop
        .icache_snoop_valid(icache_snoop_valid),
        .icache_snoop_addr(icache_snoop_addr),
        .icache_snoop_hit(icache_snoop_hit),
        .icache_snoop_ack(icache_snoop_ack),
        // D-Cache request
        .dcache_req_valid(dcache_bus_req_valid),
        .dcache_req_cmd(dcache_bus_req_cmd),
        .dcache_req_addr(dcache_bus_req_addr),
        .dcache_req_data(dcache_bus_req_data),
        .dcache_req_grant(dcache_bus_req_grant),
        .dcache_resp_data(dcache_bus_resp_data),
        .dcache_resp_valid(dcache_bus_resp_valid),
        .dcache_resp_shared(dcache_bus_resp_shared),
        // D-Cache snoop
        .dcache_snoop_valid(dcache_snoop_valid),
        .dcache_snoop_cmd(dcache_snoop_cmd),
        .dcache_snoop_addr(dcache_snoop_addr),
        .dcache_snoop_hit(dcache_snoop_hit),
        .dcache_snoop_hitm(dcache_snoop_hitm),
        .dcache_snoop_data(dcache_snoop_data),
        .dcache_snoop_ack(dcache_snoop_ack),
        // L2 interface
        .l2_req_valid(l2_req_valid),
        .l2_req_write(l2_req_write),
        .l2_req_addr(l2_req_addr),
        .l2_req_wdata(l2_req_wdata),
        .l2_resp_rdata(l2_resp_rdata),
        .l2_resp_valid(l2_resp_valid),
        .l2_resp_shared(l2_resp_shared),
        .l2_req_ready(l2_req_ready),
        // Status
        .bus_busy(),
        .bus_transactions(bus_transactions)
    );

    //------------------------------------------------------------------------
    // L2 Cache - BRAM version for proper inference
    // 2MB L2 cache fits comfortably in KU060's 1080 BRAM36K blocks
    //------------------------------------------------------------------------
    l2_cache_bram #(
        .CACHE_SIZE_KB(L2_SIZE_KB),  // 2MB shared L2 (configurable)
        .LINE_SIZE_BYTES(LINE_SIZE_BYTES),
        .NUM_WAYS(16),
        .ADDR_WIDTH(ADDR_WIDTH),
        .NUM_CORES(NUM_CORES)
    ) u_l2 (
        .clk(clk),
        .rst_n(rst_n),
        // Snoop Bus interface
        .bus_req_valid(l2_req_valid),
        .bus_req_write(l2_req_write),
        .bus_req_addr(l2_req_addr),
        .bus_req_wdata(l2_req_wdata),
        .bus_resp_rdata(l2_resp_rdata),
        .bus_resp_valid(l2_resp_valid),
        .bus_resp_shared(l2_resp_shared),
        .bus_req_ready(l2_req_ready),
        // Memory interface
        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_req_wdata(mem_req_wdata),
        .mem_resp_rdata(mem_resp_rdata),
        .mem_resp_valid(mem_resp_valid),
        .mem_req_ready(mem_req_ready),
        // Directory
        .dir_sharers(),
        .dir_modified(),
        // Status
        .cache_ready(),
        .hit_count(total_l2_hits),
        .miss_count()
    );

    // L1I miss handling - route through a simple arbiter to L2
    // For simplicity, L1I misses go directly to L2
    generate
        for (i = 0; i < NUM_CORES; i++) begin : gen_l1i_l2_conn
            assign l1i_l2_resp_data[i] = l2_resp_rdata;
            assign l1i_l2_resp_valid[i] = l2_resp_valid && l1i_l2_req_valid[i];
        end
    endgenerate

    // Status
    assign cache_system_ready = &l1i_ready && &l1d_ready;
    assign total_l1i_hits = '0; // Could add hit counters
    assign total_l1d_hits = '0;

endmodule









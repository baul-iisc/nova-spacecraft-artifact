//============================================================================
// Cache Subsystem Wrapper
// Simplified interface for integration with octa-core cluster
//============================================================================

`timescale 1ns / 1ps

module cache_subsystem #(
    parameter NUM_CORES       = 8,
    parameter L1I_SIZE_KB     = 16,
    parameter L1D_SIZE_KB     = 16,
    parameter L2_SIZE_KB      = 2048,   // 2MB L2 cache (fits in KU060 BRAM)
    parameter LINE_SIZE       = 64,
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 32
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Core Instruction Memory Interfaces (simple valid/ready)
    input  logic [NUM_CORES-1:0]                    imem_valid,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    imem_addr,
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    imem_rdata,
    output logic [NUM_CORES-1:0]                    imem_ready,
    
    // Core Data Memory Interfaces
    input  logic [NUM_CORES-1:0]                    dmem_valid,
    input  logic [NUM_CORES-1:0]                    dmem_write,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    dmem_addr,
    input  logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    dmem_wdata,
    input  logic [NUM_CORES-1:0][3:0]               dmem_wstrb,
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    dmem_rdata,
    output logic [NUM_CORES-1:0]                    dmem_ready,
    
    // Atomic Operations (per core)
    input  logic [NUM_CORES-1:0]                    amo_valid,
    input  logic [NUM_CORES-1:0][4:0]               amo_op,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    amo_addr,
    input  logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    amo_wdata,
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    amo_rdata,
    output logic [NUM_CORES-1:0]                    amo_ready,
    
    // FENCE signals (per core)
    input  logic [NUM_CORES-1:0]                    fence_req,
    output logic [NUM_CORES-1:0]                    fence_done,
    input  logic [NUM_CORES-1:0]                    fence_i_req,
    output logic [NUM_CORES-1:0]                    fence_i_done,
    
    // Main Memory Interface (to DDR/HBM controller)
    output logic                    mem_req_valid,
    output logic                    mem_req_write,
    output logic [ADDR_WIDTH-1:0]   mem_req_addr,
    output logic [LINE_SIZE*8-1:0]  mem_req_wdata,
    input  logic [LINE_SIZE*8-1:0]  mem_resp_rdata,
    input  logic                    mem_resp_valid,
    output logic                    mem_req_ready,
    
    // Status and Debug
    output logic                    system_ready,
    output logic [31:0]             perf_l2_hits,
    output logic [31:0]             perf_bus_trans
);

    // Internal fence signals with default pred/succ
    logic [NUM_CORES-1:0][3:0] fence_pred;
    logic [NUM_CORES-1:0][3:0] fence_succ;
    
    // Default FENCE is full memory barrier
    generate
        for (genvar g = 0; g < NUM_CORES; g++) begin : gen_fence_defaults
            assign fence_pred[g] = 4'b1111;  // All predecessor ops
            assign fence_succ[g] = 4'b1111;  // All successor ops
        end
    endgenerate

    // Unused LR/SC signals (simplified - routed through AMO)
    logic [NUM_CORES-1:0]                    lr_valid;
    logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    lr_addr;
    logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    lr_rdata;
    logic [NUM_CORES-1:0]                    lr_done;
    logic [NUM_CORES-1:0]                    sc_valid;
    logic [NUM_CORES-1:0][ADDR_WIDTH-1:0]    sc_addr;
    logic [NUM_CORES-1:0][DATA_WIDTH-1:0]    sc_wdata;
    logic [NUM_CORES-1:0]                    sc_success;
    logic [NUM_CORES-1:0]                    sc_done;
    
    assign lr_valid = '0;
    assign sc_valid = '0;
    assign lr_addr = '0;
    assign sc_addr = '0;
    assign sc_wdata = '0;

    // Instantiate cache coherence unit with BRAM-based caches
    cache_coherence_unit #(
        .NUM_CORES(NUM_CORES),
        .L1I_SIZE_KB(L1I_SIZE_KB),
        .L1D_SIZE_KB(L1D_SIZE_KB),
        .L2_SIZE_KB(L2_SIZE_KB),
        .LINE_SIZE_BYTES(LINE_SIZE),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_ccu (
        .clk(clk),
        .rst_n(rst_n),
        
        // Instruction fetch
        .core_ifetch_valid(imem_valid),
        .core_ifetch_addr(imem_addr),
        .core_ifetch_data(imem_rdata),
        .core_ifetch_ready(imem_ready),
        .core_ifetch_stall(),
        
        // Data memory
        .core_dmem_valid(dmem_valid),
        .core_dmem_write(dmem_write),
        .core_dmem_addr(dmem_addr),
        .core_dmem_wdata(dmem_wdata),
        .core_dmem_wmask(dmem_wstrb),
        .core_dmem_rdata(dmem_rdata),
        .core_dmem_ready(dmem_ready),
        .core_dmem_stall(),
        
        // AMO
        .core_amo_valid(amo_valid),
        .core_amo_op(amo_op),
        .core_amo_addr(amo_addr),
        .core_amo_wdata(amo_wdata),
        .core_amo_rdata(amo_rdata),
        .core_amo_done(amo_ready),
        
        // LR/SC
        .core_lr_valid(lr_valid),
        .core_lr_addr(lr_addr),
        .core_lr_rdata(lr_rdata),
        .core_lr_done(lr_done),
        .core_sc_valid(sc_valid),
        .core_sc_addr(sc_addr),
        .core_sc_wdata(sc_wdata),
        .core_sc_success(sc_success),
        .core_sc_done(sc_done),
        
        // FENCE
        .core_fence_req(fence_req),
        .core_fence_pred(fence_pred),
        .core_fence_succ(fence_succ),
        .core_fence_done(fence_done),
        .core_fence_i_req(fence_i_req),
        .core_fence_i_done(fence_i_done),
        
        // Memory
        .mem_req_valid(mem_req_valid),
        .mem_req_write(mem_req_write),
        .mem_req_addr(mem_req_addr),
        .mem_req_wdata(mem_req_wdata),
        .mem_resp_rdata(mem_resp_rdata),
        .mem_resp_valid(mem_resp_valid),
        .mem_req_ready(mem_req_ready),
        
        // Status
        .cache_system_ready(system_ready),
        .total_l1i_hits(),
        .total_l1d_hits(),
        .total_l2_hits(perf_l2_hits),
        .bus_transactions(perf_bus_trans)
    );

endmodule









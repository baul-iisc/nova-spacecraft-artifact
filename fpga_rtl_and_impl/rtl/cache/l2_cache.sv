//============================================================================
// L2 Unified Cache - 4MB, 16-way Set Associative, Shared
// Fixed for Xilinx BRAM inference - arrays split to stay under 1M-bit limit
// Structure: 4 set-groups × 4 way-groups × 8 data-banks = 128 RAM arrays
//============================================================================

`timescale 1ns / 1ps

module l2_cache #(
    parameter CACHE_SIZE_MB   = 4,      // 4MB for full design
    parameter CACHE_SIZE_KB   = 64,     // Only used if USE_KB_SIZE=1
    parameter USE_KB_SIZE     = 0,      // 0=use MB size (4MB), 1=use KB size for testing
    parameter LINE_SIZE_BYTES = 64,
    parameter NUM_WAYS        = 16,
    parameter ADDR_WIDTH      = 32,
    parameter NUM_CORES       = 8
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Snoop Bus Interface
    input  logic                    bus_req_valid,
    input  logic                    bus_req_write,
    input  logic [ADDR_WIDTH-1:0]   bus_req_addr,
    input  logic [LINE_SIZE_BYTES*8-1:0] bus_req_wdata,
    output logic [LINE_SIZE_BYTES*8-1:0] bus_resp_rdata,
    output logic                    bus_resp_valid,
    output logic                    bus_resp_shared,
    input  logic                    bus_req_ready,
    
    // Memory Interface
    output logic                    mem_req_valid,
    output logic                    mem_req_write,
    output logic [ADDR_WIDTH-1:0]   mem_req_addr,
    output logic [LINE_SIZE_BYTES*8-1:0] mem_req_wdata,
    input  logic [LINE_SIZE_BYTES*8-1:0] mem_resp_rdata,
    input  logic                    mem_resp_valid,
    output logic                    mem_req_ready,
    
    // Directory for tracking sharers
    output logic [NUM_CORES-1:0]    dir_sharers,
    output logic                    dir_modified,
    
    // Status
    output logic                    cache_ready,
    output logic [31:0]             hit_count,
    output logic [31:0]             miss_count
);

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;  // 512 bits
    localparam CACHE_BYTES = USE_KB_SIZE ? (CACHE_SIZE_KB * 1024) : (CACHE_SIZE_MB * 1024 * 1024);
    localparam NUM_SETS    = CACHE_BYTES / (LINE_SIZE_BYTES * NUM_WAYS);
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);
    localparam INDEX_BITS  = $clog2(NUM_SETS);
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;

    // Partitioning to stay under Vivado's 1M-bit limit per variable:
    // 4MB = 33M bits → need ~34 arrays minimum
    // Strategy: 4 set-groups × 4 way-groups × 8 data-banks = 128 arrays
    localparam NUM_SET_GROUPS = 4;
    localparam NUM_WAY_GROUPS = 4;
    localparam NUM_DATA_BANKS = 8;
    localparam SETS_PER_GROUP = NUM_SETS / NUM_SET_GROUPS;  // 1024 sets
    localparam WAYS_PER_GROUP = NUM_WAYS / NUM_WAY_GROUPS;  // 4 ways
    localparam BANK_WIDTH = LINE_BITS / NUM_DATA_BANKS;     // 64 bits
    localparam SET_GROUP_BITS = $clog2(NUM_SET_GROUPS);     // 2 bits
    localparam LOCAL_INDEX_BITS = INDEX_BITS - SET_GROUP_BITS;  // 10 bits
    
    // Each RAM array: 64 bits × 1024 sets × 4 ways = 262,144 bits (< 1M) ✓
    
    // Tag RAMs - split by set-group and way-group
    // Each: TAG_BITS × 1024 sets × 4 ways ≈ 56K bits
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg0_wg0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg0_wg1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg0_wg2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg0_wg3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg1_wg0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg1_wg1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg1_wg2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg1_wg3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg2_wg0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg2_wg1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg2_wg2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg2_wg3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg3_wg0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg3_wg1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg3_wg2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags_sg3_wg3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    
    // Data RAMs - split by set-group, way-group, and data-bank
    // Declaring for set-group 0, way-group 0 (repeat pattern for all 16 combinations)
    // SG0 WG0
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg0_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG0 WG1
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg1_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG0 WG2
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg2_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG0 WG3
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg0_wg3_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG1 WG0
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg0_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG1 WG1
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg1_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG1 WG2
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg2_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG1 WG3
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg1_wg3_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG2 WG0
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg0_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG2 WG1
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg1_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG2 WG2
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg2_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG2 WG3
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg2_wg3_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG3 WG0
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg0_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG3 WG1
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg1_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG3 WG2
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg2_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    // SG3 WG3
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b0 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b1 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b2 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b3 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b4 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b5 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b6 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_sg3_wg3_b7 [SETS_PER_GROUP-1:0][WAYS_PER_GROUP-1:0];
    
    // Control arrays - small, use distributed RAM with sync reset
    logic [NUM_WAYS-1:0]   valid [NUM_SETS-1:0];
    logic [NUM_WAYS-1:0]   dirty [NUM_SETS-1:0];
    logic [NUM_CORES-1:0]  sharers [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [14:0]           plru  [NUM_SETS-1:0];

    // Address decode
    wire [TAG_BITS-1:0]   tag   = bus_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] index = bus_req_addr[OFFSET_BITS +: INDEX_BITS];
    wire [SET_GROUP_BITS-1:0] set_group = index[INDEX_BITS-1 -: SET_GROUP_BITS];
    wire [LOCAL_INDEX_BITS-1:0] local_index = index[LOCAL_INDEX_BITS-1:0];

    // Tag read from appropriate set-group
    logic [TAG_BITS-1:0] tags_read [NUM_WAYS-1:0];
    
    always_comb begin
        case (set_group)
            2'd0: begin
                for (int w = 0; w < WAYS_PER_GROUP; w++) begin
                    tags_read[w+0]  = tags_sg0_wg0[local_index][w];
                    tags_read[w+4]  = tags_sg0_wg1[local_index][w];
                    tags_read[w+8]  = tags_sg0_wg2[local_index][w];
                    tags_read[w+12] = tags_sg0_wg3[local_index][w];
                end
            end
            2'd1: begin
                for (int w = 0; w < WAYS_PER_GROUP; w++) begin
                    tags_read[w+0]  = tags_sg1_wg0[local_index][w];
                    tags_read[w+4]  = tags_sg1_wg1[local_index][w];
                    tags_read[w+8]  = tags_sg1_wg2[local_index][w];
                    tags_read[w+12] = tags_sg1_wg3[local_index][w];
                end
            end
            2'd2: begin
                for (int w = 0; w < WAYS_PER_GROUP; w++) begin
                    tags_read[w+0]  = tags_sg2_wg0[local_index][w];
                    tags_read[w+4]  = tags_sg2_wg1[local_index][w];
                    tags_read[w+8]  = tags_sg2_wg2[local_index][w];
                    tags_read[w+12] = tags_sg2_wg3[local_index][w];
                end
            end
            default: begin
                for (int w = 0; w < WAYS_PER_GROUP; w++) begin
                    tags_read[w+0]  = tags_sg3_wg0[local_index][w];
                    tags_read[w+4]  = tags_sg3_wg1[local_index][w];
                    tags_read[w+8]  = tags_sg3_wg2[local_index][w];
                    tags_read[w+12] = tags_sg3_wg3[local_index][w];
                end
            end
        endcase
    end

    // Hit detection
    logic [NUM_WAYS-1:0] way_hit;
    logic [3:0] hit_way;
    logic cache_hit;
    
    always_comb begin
        way_hit = '0;
        hit_way = '0;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (valid[index][w] && tags_read[w] == tag) begin
                way_hit[w] = 1'b1;
                hit_way = w[3:0];
            end
        end
        cache_hit = |way_hit;
    end

    // PLRU victim selection for 16-way
    function automatic logic [3:0] get_victim_16way(input logic [14:0] p);
        logic [3:0] victim;
        if (!p[0]) begin
            if (!p[1]) victim = !p[3] ? 4'd0 : 4'd1;
            else       victim = !p[4] ? 4'd2 : 4'd3;
        end else if (!p[2]) begin
            if (!p[5]) victim = !p[7] ? 4'd4 : 4'd5;
            else       victim = !p[8] ? 4'd6 : 4'd7;
        end else if (!p[6]) begin
            if (!p[9])  victim = !p[11] ? 4'd8  : 4'd9;
            else        victim = !p[12] ? 4'd10 : 4'd11;
        end else begin
            if (!p[10]) victim = !p[13] ? 4'd12 : 4'd13;
            else        victim = !p[14] ? 4'd14 : 4'd15;
        end
        return victim;
    endfunction

    // State machine
    typedef enum logic [2:0] {
        IDLE, LOOKUP, WB_REQ, WB_WAIT, FILL_REQ, FILL_WAIT, FILL_DONE
    } state_t;
    
    state_t state, next_state;
    
    logic [ADDR_WIDTH-1:0]  req_addr_r;
    logic [LINE_BITS-1:0]   req_wdata_r;
    logic                   req_write_r;
    logic [3:0]             victim_way;
    
    // Registered set-group and local-index for writes
    logic [SET_GROUP_BITS-1:0] req_set_group;
    logic [LOCAL_INDEX_BITS-1:0] req_local_index;
    logic [INDEX_BITS-1:0] req_index;
    
    // Victim info
    wire victim_valid = valid[req_index][victim_way];
    wire victim_dirty = dirty[req_index][victim_way];
    wire [TAG_BITS-1:0] victim_tag = tags_read[victim_way];

    // State machine - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            req_wdata_r <= '0;
            req_write_r <= '0;
            victim_way <= '0;
            req_set_group <= '0;
            req_local_index <= '0;
            req_index <= '0;
        end else begin
            state <= next_state;
            
            if (state == IDLE && bus_req_valid) begin
                req_addr_r <= bus_req_addr;
                req_wdata_r <= bus_req_wdata;
                req_write_r <= bus_req_write;
                req_set_group <= set_group;
                req_local_index <= local_index;
                req_index <= index;
            end
            
            if (state == LOOKUP && !cache_hit) begin
                victim_way <= get_victim_16way(plru[index]);
            end
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE:      next_state = bus_req_valid ? LOOKUP : IDLE;
            LOOKUP:    next_state = cache_hit ? IDLE : (victim_valid && victim_dirty ? WB_REQ : FILL_REQ);
            WB_REQ:    next_state = WB_WAIT;
            WB_WAIT:   next_state = mem_resp_valid ? FILL_REQ : WB_WAIT;
            FILL_REQ:  next_state = FILL_WAIT;
            FILL_WAIT: next_state = mem_resp_valid ? FILL_DONE : FILL_WAIT;
            FILL_DONE: next_state = IDLE;
            default:   next_state = IDLE;
        endcase
    end

    // Refill address decode
    wire [TAG_BITS-1:0]   refill_tag   = req_addr_r[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] refill_index = req_addr_r[OFFSET_BITS +: INDEX_BITS];

    // Valid/Dirty/PLRU update - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                valid[s] <= '0;
                dirty[s] <= '0;
                plru[s] <= '0;
            end
            hit_count <= '0;
            miss_count <= '0;
        end else begin
            if (state == LOOKUP && cache_hit) begin
                hit_count <= hit_count + 1;
                if (bus_req_write)
                    dirty[index][hit_way] <= 1'b1;
            end
            if (state == LOOKUP && !cache_hit)
                miss_count <= miss_count + 1;
            if (state == FILL_DONE) begin
                valid[refill_index][victim_way] <= 1'b1;
                dirty[refill_index][victim_way] <= req_write_r;
            end
            if (state == WB_WAIT && mem_resp_valid)
                dirty[refill_index][victim_way] <= 1'b0;
        end
    end

    // Simplified data read - returns zeros for now (full implementation would mux all banks)
    logic [LINE_BITS-1:0] hit_line;
    assign hit_line = '0;  // Placeholder - full implementation would read from appropriate banks

    // Memory interface
    assign mem_req_valid = (state == FILL_REQ) || (state == WB_REQ);
    assign mem_req_write = (state == WB_REQ);
    assign mem_req_addr  = (state == WB_REQ) ? 
                           {victim_tag, refill_index, {OFFSET_BITS{1'b0}}} :
                           {refill_tag, refill_index, {OFFSET_BITS{1'b0}}};
    assign mem_req_wdata = '0;  // Placeholder
    assign mem_req_ready = (state == FILL_WAIT) || (state == WB_WAIT);

    // Bus response
    assign bus_resp_rdata  = (state == LOOKUP && cache_hit) ? hit_line :
                             (state == FILL_DONE) ? mem_resp_rdata : '0;
    assign bus_resp_valid  = (state == LOOKUP && cache_hit) || (state == FILL_DONE);
    assign bus_resp_shared = cache_hit && (|sharers[index][hit_way]);

    // Directory outputs
    assign dir_sharers  = cache_hit ? sharers[index][hit_way] : '0;
    assign dir_modified = cache_hit && dirty[index][hit_way];
    
    assign cache_ready = (state == IDLE);

endmodule

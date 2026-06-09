//============================================================================
// 1MB L2 Cache - Properly structured for BRAM inference
// Reduced size to fit within synthesis limits and demonstrate BRAM usage
//
// Structure:
//   - 1MB total, 16-way set associative
//   - 64B cache line
//   - 1024 sets (1MB / 16 ways / 64B)
//   - Each way stored in separate BRAM
//
// BRAM Usage:
//   - Data: 16 ways × 8 banks × 64-bit × 1024 depth = 128 BRAM18 (64 BRAM36)
//   - Tags: 16 ways × 51-bit × 1024 depth = 16 BRAM18 (8 BRAM36)
//   - Total: ~72 BRAM36 for 1MB L2 cache
//============================================================================

`timescale 1ns / 1ps

module l2_cache_1mb_bram #(
    parameter CACHE_SIZE_KB   = 1024,     // 1MB
    parameter LINE_SIZE       = 64,       // 64 bytes per line
    parameter NUM_WAYS        = 16,
    parameter ADDR_WIDTH      = 64,
    parameter NUM_CORES       = 8,
    parameter DATA_WIDTH      = 512       // 64 bytes = 512 bits
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Single-port interface (arbiter handles core selection)
    input  logic                    req_valid,
    input  logic                    req_write,
    input  logic [ADDR_WIDTH-1:0]   req_addr,
    input  logic [DATA_WIDTH-1:0]   req_wdata,
    output logic [DATA_WIDTH-1:0]   req_rdata,
    output logic                    req_ready,
    
    // DDR interface
    output logic                    ddr_valid,
    output logic                    ddr_write,
    output logic [ADDR_WIDTH-1:0]   ddr_addr,
    output logic [DATA_WIDTH-1:0]   ddr_wdata,
    input  logic [DATA_WIDTH-1:0]   ddr_rdata,
    input  logic                    ddr_ready,
    
    // ECC status
    output logic                    ecc_error,
    output logic                    uncorrectable
);

    // Cache geometry
    localparam CACHE_BYTES = CACHE_SIZE_KB * 1024;    // 1,048,576 bytes
    localparam NUM_SETS    = CACHE_BYTES / (LINE_SIZE * NUM_WAYS);  // 1024 sets
    localparam OFFSET_BITS = $clog2(LINE_SIZE);       // 6 bits
    localparam INDEX_BITS  = $clog2(NUM_SETS);        // 10 bits
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;  // 48 bits
    
    // Data banking for BRAM efficiency
    localparam NUM_BANKS = 8;
    localparam BANK_WIDTH = DATA_WIDTH / NUM_BANKS;  // 64 bits per bank

    // Address decode
    wire [TAG_BITS-1:0]   req_tag   = req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] req_index = req_addr[OFFSET_BITS +: INDEX_BITS];

    //=========================================================================
    // Tag Storage - One BRAM per way using generate
    //=========================================================================
    localparam TAG_ENTRY_BITS = TAG_BITS + 3;  // tag + valid + dirty + moesi_msb
    
    logic [TAG_ENTRY_BITS-1:0] tag_rd_data [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0] tag_addr;
    logic [NUM_WAYS-1:0] tag_wr_en;
    logic [TAG_ENTRY_BITS-1:0] tag_wr_data;
    
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_tag_way
            // Each way gets its own BRAM for tags
            (* ram_style = "block" *) reg [TAG_ENTRY_BITS-1:0] tag_bram [0:NUM_SETS-1];
            
            always_ff @(posedge clk) begin
                tag_rd_data[w] <= tag_bram[tag_addr];
                if (tag_wr_en[w]) begin
                    tag_bram[tag_addr] <= tag_wr_data;
                end
            end
        end
    endgenerate

    //=========================================================================
    // Data Storage - Banked BRAMs per way
    //=========================================================================
    logic [BANK_WIDTH-1:0] data_rd_data [NUM_WAYS-1:0][NUM_BANKS-1:0];
    logic [INDEX_BITS-1:0] data_addr;
    logic [NUM_WAYS-1:0] data_way_wr_en;
    logic [BANK_WIDTH-1:0] data_wr_data [NUM_BANKS-1:0];
    
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_data_way
            for (genvar b = 0; b < NUM_BANKS; b++) begin : gen_data_bank
                // Each way/bank combination gets its own BRAM
                (* ram_style = "block" *) reg [BANK_WIDTH-1:0] data_bram [0:NUM_SETS-1];
                
                always_ff @(posedge clk) begin
                    data_rd_data[w][b] <= data_bram[data_addr];
                    if (data_way_wr_en[w]) begin
                        data_bram[data_addr] <= data_wr_data[b];
                    end
                end
            end
        end
    endgenerate

    //=========================================================================
    // Cache Controller FSM
    //=========================================================================
    typedef enum logic [2:0] {
        IDLE,
        TAG_CHECK,
        HIT_RESP,
        MISS_FETCH,
        MISS_FILL
    } state_t;
    
    state_t state, next_state;
    
    // Hit detection
    logic [NUM_WAYS-1:0] way_hit;
    logic hit;
    logic [3:0] hit_way;
    
    // PLRU replacement (simplified as random for this example)
    logic [3:0] replace_way;
    
    always_comb begin
        hit = 1'b0;
        hit_way = 4'd0;
        for (int w = 0; w < NUM_WAYS; w++) begin
            logic valid;
            logic [TAG_BITS-1:0] stored_tag;
            valid = tag_rd_data[w][0];
            stored_tag = tag_rd_data[w][TAG_ENTRY_BITS-1 -: TAG_BITS];
            way_hit[w] = valid && (stored_tag == req_tag);
            if (way_hit[w]) begin
                hit = 1'b1;
                hit_way = w[3:0];
            end
        end
    end
    
    // Simple counter for replacement
    logic [3:0] replace_counter;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            replace_counter <= 4'd0;
        else if (state == MISS_FILL)
            replace_counter <= replace_counter + 1;
    end
    assign replace_way = replace_counter;
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end
    
    always_comb begin
        next_state = state;
        case (state)
            IDLE:       if (req_valid) next_state = TAG_CHECK;
            TAG_CHECK:  next_state = hit ? HIT_RESP : MISS_FETCH;
            HIT_RESP:   next_state = IDLE;
            MISS_FETCH: if (ddr_ready) next_state = MISS_FILL;
            MISS_FILL:  next_state = IDLE;
            default:    next_state = IDLE;
        endcase
    end
    
    // Address for memories
    assign tag_addr = req_index;
    assign data_addr = req_index;
    
    // Tag write
    assign tag_wr_data = {req_tag, 1'b0, 1'b1};  // tag, dirty=0, valid=1
    always_comb begin
        tag_wr_en = '0;
        if (state == MISS_FILL)
            tag_wr_en[replace_way] = 1'b1;
        else if (state == HIT_RESP && req_write)
            tag_wr_en[hit_way] = 1'b1;
    end
    
    // Data write
    always_comb begin
        for (int b = 0; b < NUM_BANKS; b++)
            data_wr_data[b] = state == MISS_FILL ? ddr_rdata[b*BANK_WIDTH +: BANK_WIDTH] 
                                                 : req_wdata[b*BANK_WIDTH +: BANK_WIDTH];
    end
    
    always_comb begin
        data_way_wr_en = '0;
        if (state == MISS_FILL)
            data_way_wr_en[replace_way] = 1'b1;
        else if (state == HIT_RESP && req_write)
            data_way_wr_en[hit_way] = 1'b1;
    end
    
    // Assemble read data from selected way
    logic [DATA_WIDTH-1:0] selected_data;
    always_comb begin
        selected_data = '0;
        for (int b = 0; b < NUM_BANKS; b++)
            selected_data[b*BANK_WIDTH +: BANK_WIDTH] = data_rd_data[hit_way][b];
    end
    
    // Output logic
    assign req_rdata = selected_data;
    assign req_ready = (state == HIT_RESP) || (state == MISS_FILL);
    
    assign ddr_valid = (state == MISS_FETCH);
    assign ddr_write = 1'b0;  // Simplified - no writeback
    assign ddr_addr = {req_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    assign ddr_wdata = '0;
    
    assign ecc_error = 1'b0;       // Simplified
    assign uncorrectable = 1'b0;

endmodule





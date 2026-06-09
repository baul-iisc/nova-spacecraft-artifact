//============================================================================
// L1 Instruction Cache - 16KB, 4-way Set Associative
// Fixed for Xilinx BRAM inference - uses synchronous reset for control,
// no reset for data/tag arrays
//============================================================================

`timescale 1ns / 1ps

module l1_icache #(
    parameter CACHE_SIZE_KB   = 16,
    parameter LINE_SIZE_BYTES = 64,
    parameter NUM_WAYS        = 4,
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 32,
    parameter CORE_ID         = 0
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // CPU Interface
    input  logic                    cpu_req_valid,
    input  logic [ADDR_WIDTH-1:0]   cpu_req_addr,
    output logic [DATA_WIDTH-1:0]   cpu_resp_data,
    output logic                    cpu_resp_valid,
    output logic                    cpu_stall,
    
    // L2 Interface
    output logic                    l2_req_valid,
    output logic [ADDR_WIDTH-1:0]   l2_req_addr,
    input  logic [LINE_SIZE_BYTES*8-1:0] l2_resp_data,
    input  logic                    l2_resp_valid,
    output logic                    l2_req_ready,
    
    // Snoop Interface (invalidation only for I$)
    input  logic                    snoop_valid,
    input  logic [ADDR_WIDTH-1:0]   snoop_addr,
    output logic                    snoop_hit,
    output logic                    snoop_ack,
    
    // FENCE.I
    input  logic                    fence_i,
    output logic                    fence_complete,
    
    // Status
    output logic                    cache_ready
);

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;
    localparam NUM_SETS    = (CACHE_SIZE_KB * 1024) / (LINE_SIZE_BYTES * NUM_WAYS);
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);
    localparam INDEX_BITS  = $clog2(NUM_SETS);
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    localparam WORD_BITS   = $clog2(LINE_SIZE_BYTES / 4);

    // Banked data storage for BRAM inference (8 banks of 64 bits each)
    localparam NUM_BANKS = 8;
    localparam BANK_WIDTH = LINE_BITS / NUM_BANKS;  // 64 bits per bank
    
    // Tag and Data RAMs - NO RESET for BRAM inference
    (* ram_style = "block" *) logic [TAG_BITS-1:0]   tags  [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank0 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank1 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank2 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank3 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank4 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank5 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank6 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank7 [NUM_SETS-1:0][NUM_WAYS-1:0];
    
    // Valid bits and PLRU - small, can use distributed RAM or registers
    logic [NUM_WAYS-1:0]    valid  [NUM_SETS-1:0];
    logic [2:0]             plru   [NUM_SETS-1:0];

    // Helper function to read full line from banks
    function automatic logic [LINE_BITS-1:0] read_data_line(
        input logic [INDEX_BITS-1:0] idx,
        input logic [1:0] way
    );
        return {data_bank7[idx][way], data_bank6[idx][way], data_bank5[idx][way], data_bank4[idx][way],
                data_bank3[idx][way], data_bank2[idx][way], data_bank1[idx][way], data_bank0[idx][way]};
    endfunction

    // Address decode
    wire [TAG_BITS-1:0]   tag   = cpu_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] index = cpu_req_addr[OFFSET_BITS +: INDEX_BITS];
    wire [WORD_BITS-1:0]  word  = cpu_req_addr[OFFSET_BITS-1:2];

    // Hit detection
    logic [NUM_WAYS-1:0] way_hit;
    logic [1:0] hit_way;
    logic cache_hit;
    
    always_comb begin
        way_hit = '0;
        hit_way = '0;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (valid[index][w] && tags[index][w] == tag) begin
                way_hit[w] = 1'b1;
                hit_way = w[1:0];
            end
        end
        cache_hit = |way_hit;
    end

    // State machine
    typedef enum logic [2:0] {IDLE, LOOKUP, REFILL_REQ, REFILL_WAIT, REFILL_DONE, FENCE_INV} state_t;
    state_t state, next_state;
    
    logic [ADDR_WIDTH-1:0] req_addr_r;
    logic [1:0] victim_way;
    logic [INDEX_BITS-1:0] fence_idx;

    // PLRU victim selection
    function automatic logic [1:0] get_victim(input logic [2:0] p);
        if (!p[2]) return p[1] ? 2'd1 : 2'd0;
        else       return p[0] ? 2'd3 : 2'd2;
    endfunction

    // State machine and control registers - SYNCHRONOUS reset only
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            victim_way <= '0;
            fence_idx <= '0;
        end else begin
            state <= next_state;
            if (state == IDLE && cpu_req_valid && !cache_hit)
                req_addr_r <= cpu_req_addr;
            if (state == LOOKUP && !cache_hit)
                victim_way <= get_victim(plru[index]);
            if (state == FENCE_INV)
                fence_idx <= fence_idx + 1;
            else if (fence_i && state == IDLE)
                fence_idx <= '0;
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE:        next_state = fence_i ? FENCE_INV : (cpu_req_valid ? LOOKUP : IDLE);
            LOOKUP:      next_state = cache_hit ? IDLE : REFILL_REQ;
            REFILL_REQ:  next_state = REFILL_WAIT;
            REFILL_WAIT: next_state = l2_resp_valid ? REFILL_DONE : REFILL_WAIT;
            REFILL_DONE: next_state = IDLE;
            FENCE_INV:   next_state = (fence_idx == NUM_SETS-1) ? IDLE : FENCE_INV;
            default:     next_state = IDLE;
        endcase
    end

    // Refill address decode
    wire [TAG_BITS-1:0]   refill_tag   = req_addr_r[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] refill_index = req_addr_r[OFFSET_BITS +: INDEX_BITS];

    // Valid bits update - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                valid[s] <= '0;
                plru[s] <= '0;
            end
        end else begin
            // Refill - set valid
            if (state == REFILL_DONE) begin
                valid[refill_index][victim_way] <= 1'b1;
            end
            // PLRU update on hit
            if (state == LOOKUP && cache_hit) begin
                case (hit_way)
                    2'd0: begin plru[index][2] <= 1'b1; plru[index][1] <= 1'b1; end
                    2'd1: begin plru[index][2] <= 1'b1; plru[index][1] <= 1'b0; end
                    2'd2: begin plru[index][2] <= 1'b0; plru[index][0] <= 1'b1; end
                    2'd3: begin plru[index][2] <= 1'b0; plru[index][0] <= 1'b0; end
                endcase
            end
            // FENCE.I invalidate
            if (state == FENCE_INV)
                valid[fence_idx] <= '0;
            // Snoop invalidate
            if (snoop_valid) begin
                for (int w = 0; w < NUM_WAYS; w++) begin
                    if (valid[snoop_addr[OFFSET_BITS +: INDEX_BITS]][w] && 
                        tags[snoop_addr[OFFSET_BITS +: INDEX_BITS]][w] == snoop_addr[ADDR_WIDTH-1 -: TAG_BITS])
                        valid[snoop_addr[OFFSET_BITS +: INDEX_BITS]][w] <= 1'b0;
                end
            end
        end
    end
    
    // Tag RAM update - NO RESET for BRAM inference
    always_ff @(posedge clk) begin
        if (state == REFILL_DONE) begin
            tags[refill_index][victim_way] <= refill_tag;
        end
    end
    
    // Data RAM update - NO RESET, banked writes for BRAM inference
    always_ff @(posedge clk) begin
        if (state == REFILL_DONE) begin
            data_bank0[refill_index][victim_way] <= l2_resp_data[0*BANK_WIDTH +: BANK_WIDTH];
            data_bank1[refill_index][victim_way] <= l2_resp_data[1*BANK_WIDTH +: BANK_WIDTH];
            data_bank2[refill_index][victim_way] <= l2_resp_data[2*BANK_WIDTH +: BANK_WIDTH];
            data_bank3[refill_index][victim_way] <= l2_resp_data[3*BANK_WIDTH +: BANK_WIDTH];
            data_bank4[refill_index][victim_way] <= l2_resp_data[4*BANK_WIDTH +: BANK_WIDTH];
            data_bank5[refill_index][victim_way] <= l2_resp_data[5*BANK_WIDTH +: BANK_WIDTH];
            data_bank6[refill_index][victim_way] <= l2_resp_data[6*BANK_WIDTH +: BANK_WIDTH];
            data_bank7[refill_index][victim_way] <= l2_resp_data[7*BANK_WIDTH +: BANK_WIDTH];
        end
    end

    // Snoop hit detection
    always_comb begin
        snoop_hit = 1'b0;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (valid[snoop_addr[OFFSET_BITS +: INDEX_BITS]][w] &&
                tags[snoop_addr[OFFSET_BITS +: INDEX_BITS]][w] == snoop_addr[ADDR_WIDTH-1 -: TAG_BITS])
                snoop_hit = 1'b1;
        end
    end

    // Outputs
    wire [LINE_BITS-1:0] hit_line = read_data_line(index, hit_way);
    assign cpu_resp_data  = (state == LOOKUP && cache_hit) ? hit_line[word*32 +: 32] :
                            (state == REFILL_DONE) ? l2_resp_data[req_addr_r[OFFSET_BITS-1:2]*32 +: 32] : '0;
    assign cpu_resp_valid = (state == LOOKUP && cache_hit) || (state == REFILL_DONE);
    assign cpu_stall      = cpu_req_valid && !cpu_resp_valid;
    assign l2_req_valid   = (state == REFILL_REQ);
    assign l2_req_addr    = {req_addr_r[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    assign l2_req_ready   = (state == REFILL_WAIT);
    assign snoop_ack      = snoop_valid;
    assign fence_complete = (state == FENCE_INV && fence_idx == NUM_SETS-1);
    assign cache_ready    = (state == IDLE);

endmodule

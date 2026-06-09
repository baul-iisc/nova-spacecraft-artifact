//============================================================================
// L1 Instruction Cache - 16KB, 4-way Set Associative
// Redesigned for guaranteed BRAM inference using explicit BRAM modules
// Each way has its own BRAM instance for tags and data
//============================================================================

`timescale 1ns / 1ps

module l1_icache_bram #(
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

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;  // 512 bits
    localparam NUM_SETS    = (CACHE_SIZE_KB * 1024) / (LINE_SIZE_BYTES * NUM_WAYS);  // 64 sets
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);  // 6 bits
    localparam INDEX_BITS  = $clog2(NUM_SETS);         // 6 bits
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;  // 20 bits
    localparam WORD_BITS   = $clog2(LINE_SIZE_BYTES / 4);  // 4 bits

    // Address decode
    wire [TAG_BITS-1:0]   req_tag   = cpu_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] req_index = cpu_req_addr[OFFSET_BITS +: INDEX_BITS];
    wire [WORD_BITS-1:0]  req_word  = cpu_req_addr[OFFSET_BITS-1:2];

    // Valid bits - small array, can be registers
    logic [NUM_WAYS-1:0] valid [NUM_SETS-1:0];
    logic [2:0]          plru  [NUM_SETS-1:0];

    // Tag BRAM interfaces - one per way
    logic                    tag_wr_en   [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   tag_wr_addr [NUM_WAYS-1:0];
    logic [TAG_BITS-1:0]     tag_wr_data [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   tag_rd_addr [NUM_WAYS-1:0];
    logic [TAG_BITS-1:0]     tag_rd_data [NUM_WAYS-1:0];

    // Data BRAM interfaces - one per way (512-bit wide)
    logic                    data_wr_en   [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   data_wr_addr [NUM_WAYS-1:0];
    logic [LINE_BITS-1:0]    data_wr_data [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   data_rd_addr [NUM_WAYS-1:0];
    logic [LINE_BITS-1:0]    data_rd_data [NUM_WAYS-1:0];

    // Generate BRAM instances for each way
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_way_bram
            // Tag BRAM - 20 bits x 64 entries = 1280 bits (tiny, but use BRAM for consistency)
            bram_sdp #(
                .DATA_WIDTH(TAG_BITS),
                .ADDR_WIDTH(INDEX_BITS),
                .DEPTH(NUM_SETS)
            ) u_tag_ram (
                .clk(clk),
                .wr_en(tag_wr_en[w]),
                .wr_addr(tag_wr_addr[w]),
                .wr_data(tag_wr_data[w]),
                .rd_en(1'b1),
                .rd_addr(tag_rd_addr[w]),
                .rd_data(tag_rd_data[w])
            );
            
            // Data BRAM - 512 bits x 64 entries = 32768 bits = 4KB per way
            bram_sdp #(
                .DATA_WIDTH(LINE_BITS),
                .ADDR_WIDTH(INDEX_BITS),
                .DEPTH(NUM_SETS)
            ) u_data_ram (
                .clk(clk),
                .wr_en(data_wr_en[w]),
                .wr_addr(data_wr_addr[w]),
                .wr_data(data_wr_data[w]),
                .rd_en(1'b1),
                .rd_addr(data_rd_addr[w]),
                .rd_data(data_rd_data[w])
            );
        end
    endgenerate

    // State machine
    typedef enum logic [2:0] {
        IDLE, LOOKUP, REFILL_REQ, REFILL_WAIT, REFILL_DONE, FENCE_INV
    } state_t;
    
    state_t state, next_state;
    
    logic [ADDR_WIDTH-1:0]  req_addr_r;
    logic [TAG_BITS-1:0]    req_tag_r;
    logic [INDEX_BITS-1:0]  req_index_r;
    logic [WORD_BITS-1:0]   req_word_r;
    logic [1:0]             victim_way;
    logic [INDEX_BITS-1:0]  fence_idx;

    // Pipeline registers for lookup
    logic [INDEX_BITS-1:0]  lookup_index_r;
    logic [TAG_BITS-1:0]    lookup_tag_r;
    logic [WORD_BITS-1:0]   lookup_word_r;
    logic                   lookup_valid_r;

    // Hit detection (registered - 1 cycle after address presented)
    logic [NUM_WAYS-1:0] way_hit_r;
    logic [1:0] hit_way_r;
    logic cache_hit_r;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            way_hit_r <= '0;
            hit_way_r <= '0;
            cache_hit_r <= '0;
        end else begin
            way_hit_r <= '0;
            hit_way_r <= '0;
            for (int w = 0; w < NUM_WAYS; w++) begin
                if (valid[lookup_index_r][w] && tag_rd_data[w] == lookup_tag_r) begin
                    way_hit_r[w] <= 1'b1;
                    hit_way_r <= w[1:0];
                end
            end
            cache_hit_r <= |way_hit_r;
        end
    end

    // PLRU victim selection
    function automatic logic [1:0] get_victim(input logic [2:0] p);
        if (!p[2]) return p[1] ? 2'd1 : 2'd0;
        else       return p[0] ? 2'd3 : 2'd2;
    endfunction

    // State machine control
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            req_tag_r <= '0;
            req_index_r <= '0;
            req_word_r <= '0;
            victim_way <= '0;
            fence_idx <= '0;
            lookup_index_r <= '0;
            lookup_tag_r <= '0;
            lookup_word_r <= '0;
            lookup_valid_r <= '0;
        end else begin
            state <= next_state;
            
            // Capture request on IDLE->LOOKUP
            if (state == IDLE && cpu_req_valid) begin
                req_addr_r <= cpu_req_addr;
                req_tag_r <= req_tag;
                req_index_r <= req_index;
                req_word_r <= req_word;
                lookup_index_r <= req_index;
                lookup_tag_r <= req_tag;
                lookup_word_r <= req_word;
                lookup_valid_r <= 1'b1;
            end else begin
                lookup_valid_r <= 1'b0;
            end
            
            // Select victim on miss
            if (state == LOOKUP && !cache_hit_r) begin
                victim_way <= get_victim(plru[req_index_r]);
            end
            
            // FENCE.I index counter
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
            LOOKUP:      next_state = cache_hit_r ? IDLE : REFILL_REQ;
            REFILL_REQ:  next_state = REFILL_WAIT;
            REFILL_WAIT: next_state = l2_resp_valid ? REFILL_DONE : REFILL_WAIT;
            REFILL_DONE: next_state = IDLE;
            FENCE_INV:   next_state = (fence_idx == NUM_SETS-1) ? IDLE : FENCE_INV;
            default:     next_state = IDLE;
        endcase
    end

    // Tag RAM control - read during IDLE/lookup, write during refill
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            tag_rd_addr[w] = (state == IDLE) ? req_index : req_index_r;
            tag_wr_en[w] = (state == REFILL_DONE) && (w[1:0] == victim_way);
            tag_wr_addr[w] = req_index_r;
            tag_wr_data[w] = req_tag_r;
        end
    end

    // Data RAM control - read during IDLE/lookup, write during refill
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            data_rd_addr[w] = (state == IDLE) ? req_index : req_index_r;
            data_wr_en[w] = (state == REFILL_DONE) && (w[1:0] == victim_way);
            data_wr_addr[w] = req_index_r;
            data_wr_data[w] = l2_resp_data;
        end
    end

    // Valid bits and PLRU update
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                valid[s] <= '0;
                plru[s] <= '0;
            end
        end else begin
            // Refill - set valid
            if (state == REFILL_DONE) begin
                valid[req_index_r][victim_way] <= 1'b1;
            end
            
            // PLRU update on hit
            if (state == LOOKUP && cache_hit_r) begin
                case (hit_way_r)
                    2'd0: begin plru[req_index_r][2] <= 1'b1; plru[req_index_r][1] <= 1'b1; end
                    2'd1: begin plru[req_index_r][2] <= 1'b1; plru[req_index_r][1] <= 1'b0; end
                    2'd2: begin plru[req_index_r][2] <= 1'b0; plru[req_index_r][0] <= 1'b1; end
                    2'd3: begin plru[req_index_r][2] <= 1'b0; plru[req_index_r][0] <= 1'b0; end
                endcase
            end
            
            // FENCE.I invalidate
            if (state == FENCE_INV)
                valid[fence_idx] <= '0;
                
            // Snoop invalidate
            if (snoop_valid) begin
                logic [INDEX_BITS-1:0] snoop_idx;
                logic [TAG_BITS-1:0] snoop_tag_val;
                snoop_idx = snoop_addr[OFFSET_BITS +: INDEX_BITS];
                snoop_tag_val = snoop_addr[ADDR_WIDTH-1 -: TAG_BITS];
                for (int w = 0; w < NUM_WAYS; w++) begin
                    if (valid[snoop_idx][w] && tag_rd_data[w] == snoop_tag_val)
                        valid[snoop_idx][w] <= 1'b0;
                end
            end
        end
    end

    // Snoop hit detection
    always_comb begin
        snoop_hit = 1'b0;
        if (snoop_valid) begin
            for (int w = 0; w < NUM_WAYS; w++) begin
                if (valid[snoop_addr[OFFSET_BITS +: INDEX_BITS]][w] &&
                    tag_rd_data[w] == snoop_addr[ADDR_WIDTH-1 -: TAG_BITS])
                    snoop_hit = 1'b1;
            end
        end
    end

    // Output mux - select data from hit way
    logic [LINE_BITS-1:0] hit_line;
    always_comb begin
        hit_line = data_rd_data[hit_way_r];
    end

    // CPU outputs
    assign cpu_resp_data  = (state == LOOKUP && cache_hit_r) ? hit_line[req_word_r*32 +: 32] :
                            (state == REFILL_DONE) ? l2_resp_data[req_word_r*32 +: 32] : '0;
    assign cpu_resp_valid = (state == LOOKUP && cache_hit_r) || (state == REFILL_DONE);
    assign cpu_stall      = cpu_req_valid && !cpu_resp_valid;
    
    // L2 interface
    assign l2_req_valid   = (state == REFILL_REQ);
    assign l2_req_addr    = {req_addr_r[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    assign l2_req_ready   = (state == REFILL_WAIT);
    
    // Snoop/fence outputs
    assign snoop_ack      = snoop_valid;
    assign fence_complete = (state == FENCE_INV && fence_idx == NUM_SETS-1);
    assign cache_ready    = (state == IDLE);

endmodule


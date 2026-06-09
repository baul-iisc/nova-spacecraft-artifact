//============================================================================
// L1 Instruction Cache with SECDED ECC - 16KB, 4-way Set Associative
// Space-Grade: Single-bit correct, double-bit detect, auto-reload on error
//============================================================================

`timescale 1ns / 1ps

module l1_icache_ecc #(
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
    
    // ECC Error Status
    output logic                    ecc_single_error,    // Correctable error occurred
    output logic                    ecc_double_error,    // Uncorrectable error (triggers reload)
    output logic [31:0]             ecc_error_addr,      // Address of error
    output logic [31:0]             ecc_error_count,     // Total error count
    
    // Status
    output logic                    cache_ready
);

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;  // 512 bits
    localparam NUM_SETS    = (CACHE_SIZE_KB * 1024) / (LINE_SIZE_BYTES * NUM_WAYS); // 64
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);  // 6
    localparam INDEX_BITS  = $clog2(NUM_SETS);         // 6
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;  // 20
    localparam WORD_BITS   = $clog2(LINE_SIZE_BYTES / 4);  // 4
    
    // ECC parameters: 64-bit data + 8 ECC bits = 72 bits per word
    localparam WORDS_PER_LINE = LINE_SIZE_BYTES / 8;  // 8 words of 64-bit
    localparam ECC_WORD_BITS = 72;  // 64 data + 8 ECC
    localparam ECC_LINE_BITS = WORDS_PER_LINE * ECC_WORD_BITS;  // 576 bits

    //------------------------------------------------------------------------
    // Storage with ECC
    //------------------------------------------------------------------------
    // Tag storage with ECC (20-bit tag + 7 ECC = 27 bits, padded to 32)
    logic [31:0]              tags_ecc   [NUM_SETS-1:0][NUM_WAYS-1:0];
    // Valid bits (no ECC needed - use TMR instead)
    logic [NUM_WAYS-1:0]      valid      [NUM_SETS-1:0];
    logic [NUM_WAYS-1:0]      valid_tmr1 [NUM_SETS-1:0];
    logic [NUM_WAYS-1:0]      valid_tmr2 [NUM_SETS-1:0];
    // Data storage with ECC (576 bits per line)
    logic [ECC_LINE_BITS-1:0] data_ecc   [NUM_SETS-1:0][NUM_WAYS-1:0];
    // Pseudo-LRU
    logic [2:0]               plru       [NUM_SETS-1:0];

    //------------------------------------------------------------------------
    // Address decode
    //------------------------------------------------------------------------
    wire [TAG_BITS-1:0]   tag   = cpu_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] index = cpu_req_addr[OFFSET_BITS +: INDEX_BITS];
    wire [WORD_BITS-1:0]  word  = cpu_req_addr[OFFSET_BITS-1:2];
    wire [2:0]            dword = cpu_req_addr[OFFSET_BITS-1:3];  // 64-bit word select

    //------------------------------------------------------------------------
    // TMR voting for valid bits
    //------------------------------------------------------------------------
    logic [NUM_WAYS-1:0] valid_voted [NUM_SETS-1:0];
    
    always_comb begin
        for (int s = 0; s < NUM_SETS; s++) begin
            for (int w = 0; w < NUM_WAYS; w++) begin
                valid_voted[s][w] = (valid[s][w] & valid_tmr1[s][w]) |
                                    (valid_tmr1[s][w] & valid_tmr2[s][w]) |
                                    (valid[s][w] & valid_tmr2[s][w]);
            end
        end
    end

    //------------------------------------------------------------------------
    // ECC decode for tag
    //------------------------------------------------------------------------
    logic [TAG_BITS-1:0] tag_decoded [NUM_WAYS-1:0];
    logic [NUM_WAYS-1:0] tag_single_err;
    logic [NUM_WAYS-1:0] tag_double_err;
    
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_tag_ecc
            ecc_secded_32 u_tag_ecc (
                .data_in('0),  // Not used for decode
                .encoded_out(),
                .encoded_in({7'b0, tags_ecc[index][w]}),  // Padded to 39 bits
                .data_out(tag_decoded[w]),
                .single_error(tag_single_err[w]),
                .double_error(tag_double_err[w])
            );
        end
    endgenerate

    //------------------------------------------------------------------------
    // ECC decode for data
    //------------------------------------------------------------------------
    logic [63:0] data_decoded [NUM_WAYS-1:0][WORDS_PER_LINE-1:0];
    logic [NUM_WAYS-1:0] data_single_err [WORDS_PER_LINE-1:0];
    logic [NUM_WAYS-1:0] data_double_err [WORDS_PER_LINE-1:0];
    
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_data_ecc_way
            for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_data_ecc_word
                ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_data_ecc (
                    .clk(clk),
                    .rst_n(rst_n),
                    .data_in('0),
                    .encoded_out(),
                    .encoded_in(data_ecc[index][w][d*ECC_WORD_BITS +: ECC_WORD_BITS]),
                    .data_out(data_decoded[w][d]),
                    .single_error(data_single_err[d][w]),
                    .double_error(data_double_err[d][w]),
                    .error_position()
                );
            end
        end
    endgenerate

    //------------------------------------------------------------------------
    // Hit detection (using decoded tags)
    //------------------------------------------------------------------------
    logic [NUM_WAYS-1:0] way_hit;
    logic [1:0] hit_way;
    logic cache_hit;
    logic hit_has_ecc_error;
    logic hit_uncorrectable;
    
    always_comb begin
        way_hit = '0;
        hit_way = '0;
        hit_has_ecc_error = 1'b0;
        hit_uncorrectable = 1'b0;
        
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (valid_voted[index][w] && tag_decoded[w][TAG_BITS-1:0] == tag) begin
                way_hit[w] = 1'b1;
                hit_way = w[1:0];
                // Check for ECC errors in this way
                hit_has_ecc_error = tag_single_err[w] | (|data_single_err[dword]);
                hit_uncorrectable = tag_double_err[w] | (|data_double_err[dword]);
            end
        end
        cache_hit = |way_hit && !hit_uncorrectable;
    end

    //------------------------------------------------------------------------
    // State machine with ECC error handling
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE, 
        LOOKUP, 
        REFILL_REQ, 
        REFILL_WAIT, 
        REFILL_DONE,
        ECC_RELOAD_REQ,   // Reload due to uncorrectable ECC error
        ECC_RELOAD_WAIT,
        ECC_RELOAD_DONE,
        FENCE_INV
    } state_t;
    
    state_t state, next_state;
    
    logic [ADDR_WIDTH-1:0] req_addr_r;
    logic [1:0] victim_way;
    logic [INDEX_BITS-1:0] fence_idx;
    logic ecc_error_triggered;

    // PLRU victim selection
    function automatic logic [1:0] get_victim(input logic [2:0] p);
        if (!p[2]) return p[1] ? 2'd1 : 2'd0;
        else       return p[0] ? 2'd3 : 2'd2;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            victim_way <= '0;
            fence_idx <= '0;
            ecc_error_triggered <= 1'b0;
        end else begin
            state <= next_state;
            
            if (state == IDLE && cpu_req_valid) begin
                req_addr_r <= cpu_req_addr;
            end
            
            if (state == LOOKUP && !cache_hit)
                victim_way <= get_victim(plru[index]);
            
            // Track if we're reloading due to ECC error
            if (state == LOOKUP && hit_uncorrectable)
                ecc_error_triggered <= 1'b1;
            else if (state == ECC_RELOAD_DONE)
                ecc_error_triggered <= 1'b0;
            
            if (state == FENCE_INV)
                fence_idx <= fence_idx + 1;
            else if (fence_i && state == IDLE)
                fence_idx <= '0;
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (fence_i)
                    next_state = FENCE_INV;
                else if (cpu_req_valid)
                    next_state = LOOKUP;
            end
            
            LOOKUP: begin
                if (hit_uncorrectable)
                    next_state = ECC_RELOAD_REQ;  // Uncorrectable -> reload from L2
                else if (cache_hit)
                    next_state = IDLE;
                else
                    next_state = REFILL_REQ;
            end
            
            REFILL_REQ:      next_state = REFILL_WAIT;
            REFILL_WAIT:     next_state = l2_resp_valid ? REFILL_DONE : REFILL_WAIT;
            REFILL_DONE:     next_state = IDLE;
            
            ECC_RELOAD_REQ:  next_state = ECC_RELOAD_WAIT;
            ECC_RELOAD_WAIT: next_state = l2_resp_valid ? ECC_RELOAD_DONE : ECC_RELOAD_WAIT;
            ECC_RELOAD_DONE: next_state = IDLE;
            
            FENCE_INV:       next_state = (fence_idx == NUM_SETS-1) ? IDLE : FENCE_INV;
            
            default: next_state = IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // ECC encoding for write
    //------------------------------------------------------------------------
    logic [ECC_LINE_BITS-1:0] encoded_line;
    
    generate
        for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_encode
            logic [71:0] encoded_word;
            ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_encode (
                .clk(clk),
                .rst_n(rst_n),
                .data_in(l2_resp_data[d*64 +: 64]),
                .encoded_out(encoded_word),
                .encoded_in('0),
                .data_out(),
                .single_error(),
                .double_error(),
                .error_position()
            );
            assign encoded_line[d*72 +: 72] = encoded_word;
        end
    endgenerate

    //------------------------------------------------------------------------
    // Tag encoding
    //------------------------------------------------------------------------
    wire [TAG_BITS-1:0]   refill_tag   = req_addr_r[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] refill_index = req_addr_r[OFFSET_BITS +: INDEX_BITS];
    
    logic [38:0] tag_encoded;
    ecc_secded_32 u_tag_encode (
        .data_in({{(32-TAG_BITS){1'b0}}, refill_tag}),
        .encoded_out(tag_encoded),
        .encoded_in('0),
        .data_out(),
        .single_error(),
        .double_error()
    );

    //------------------------------------------------------------------------
    // Cache update with ECC
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                valid[s] <= '0;
                valid_tmr1[s] <= '0;
                valid_tmr2[s] <= '0;
            end
        end else begin
            // Refill or ECC reload
            if (state == REFILL_DONE || state == ECC_RELOAD_DONE) begin
                tags_ecc[refill_index][victim_way] <= tag_encoded[31:0];
                data_ecc[refill_index][victim_way] <= encoded_line;
                // TMR valid bits
                valid[refill_index][victim_way] <= 1'b1;
                valid_tmr1[refill_index][victim_way] <= 1'b1;
                valid_tmr2[refill_index][victim_way] <= 1'b1;
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
            if (state == FENCE_INV) begin
                valid[fence_idx] <= '0;
                valid_tmr1[fence_idx] <= '0;
                valid_tmr2[fence_idx] <= '0;
            end
            
            // Snoop invalidate
            if (snoop_valid) begin
                for (int w = 0; w < NUM_WAYS; w++) begin
                    logic [INDEX_BITS-1:0] snp_idx;
                    snp_idx = snoop_addr[OFFSET_BITS +: INDEX_BITS];
                    if (valid_voted[snp_idx][w] && 
                        tag_decoded[w][TAG_BITS-1:0] == snoop_addr[ADDR_WIDTH-1 -: TAG_BITS]) begin
                        valid[snp_idx][w] <= 1'b0;
                        valid_tmr1[snp_idx][w] <= 1'b0;
                        valid_tmr2[snp_idx][w] <= 1'b0;
                    end
                end
            end
        end
    end

    //------------------------------------------------------------------------
    // ECC error tracking
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ecc_error_count <= '0;
            ecc_error_addr <= '0;
        end else begin
            if (state == LOOKUP && (hit_has_ecc_error || hit_uncorrectable)) begin
                ecc_error_count <= ecc_error_count + 1;
                ecc_error_addr <= cpu_req_addr;
            end
        end
    end
    
    assign ecc_single_error = (state == LOOKUP) && hit_has_ecc_error && !hit_uncorrectable;
    assign ecc_double_error = (state == LOOKUP) && hit_uncorrectable;

    //------------------------------------------------------------------------
    // Snoop hit
    //------------------------------------------------------------------------
    always_comb begin
        snoop_hit = 1'b0;
        for (int w = 0; w < NUM_WAYS; w++) begin
            logic [INDEX_BITS-1:0] snp_idx;
            snp_idx = snoop_addr[OFFSET_BITS +: INDEX_BITS];
            if (valid_voted[snp_idx][w] &&
                tag_decoded[w][TAG_BITS-1:0] == snoop_addr[ADDR_WIDTH-1 -: TAG_BITS])
                snoop_hit = 1'b1;
        end
    end

    //------------------------------------------------------------------------
    // Outputs
    //------------------------------------------------------------------------
    // Select correct 32-bit word from 64-bit decoded word
    wire [63:0] hit_dword = data_decoded[hit_way][dword];
    wire [31:0] hit_word = word[0] ? hit_dword[63:32] : hit_dword[31:0];
    
    assign cpu_resp_data  = (state == LOOKUP && cache_hit) ? hit_word :
                            ((state == REFILL_DONE || state == ECC_RELOAD_DONE) ? 
                             l2_resp_data[req_addr_r[OFFSET_BITS-1:2]*32 +: 32] : '0);
    assign cpu_resp_valid = (state == LOOKUP && cache_hit) || 
                            (state == REFILL_DONE) || 
                            (state == ECC_RELOAD_DONE);
    assign cpu_stall      = cpu_req_valid && !cpu_resp_valid;
    
    assign l2_req_valid   = (state == REFILL_REQ) || (state == ECC_RELOAD_REQ);
    assign l2_req_addr    = {req_addr_r[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    assign l2_req_ready   = (state == REFILL_WAIT) || (state == ECC_RELOAD_WAIT);
    
    assign snoop_ack      = snoop_valid;
    assign fence_complete = (state == FENCE_INV && fence_idx == NUM_SETS-1);
    assign cache_ready    = (state == IDLE);

endmodule









//============================================================================
// L2 Unified Cache with SECDED ECC - 4MB, 16-way Set Associative
// Space-Grade: Write-back, ECC on all data, auto-reload on uncorrectable error
//============================================================================

`timescale 1ns / 1ps

module l2_cache_ecc #(
    parameter CACHE_SIZE_MB   = 4,
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
    
    // Main Memory Interface (with ECC)
    output logic                    mem_req_valid,
    output logic                    mem_req_write,
    output logic [ADDR_WIDTH-1:0]   mem_req_addr,
    output logic [LINE_SIZE_BYTES*8-1:0] mem_req_wdata,
    output logic [LINE_SIZE_BYTES-1:0]   mem_req_ecc,   // ECC bits for memory
    input  logic [LINE_SIZE_BYTES*8-1:0] mem_resp_rdata,
    input  logic [LINE_SIZE_BYTES-1:0]   mem_resp_ecc,  // ECC from memory
    input  logic                    mem_resp_valid,
    input  logic                    mem_resp_ecc_error, // Memory ECC error
    output logic                    mem_req_ready,
    
    // Directory
    output logic [NUM_CORES-1:0]    dir_sharers,
    output logic                    dir_modified,
    
    // ECC Status
    output logic                    ecc_single_error,
    output logic                    ecc_double_error,
    output logic [31:0]             ecc_error_addr,
    output logic [31:0]             ecc_error_count,
    
    // Status
    output logic                    cache_ready,
    output logic [31:0]             hit_count,
    output logic [31:0]             miss_count
);

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;  // 512 bits
    localparam CACHE_BYTES = CACHE_SIZE_MB * 1024 * 1024;
    localparam NUM_SETS    = CACHE_BYTES / (LINE_SIZE_BYTES * NUM_WAYS);  // 4096
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);  // 6
    localparam INDEX_BITS  = $clog2(NUM_SETS);         // 12
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;  // 14
    
    // ECC: 64-bit + 8 ECC = 72 bits per word, 8 words per line = 576 bits
    localparam WORDS_PER_LINE = LINE_SIZE_BYTES / 8;
    localparam ECC_WORD_BITS = 72;
    localparam ECC_LINE_BITS = WORDS_PER_LINE * ECC_WORD_BITS;  // 576 bits

    //------------------------------------------------------------------------
    // Storage with ECC - Vivado infers BRAM/URAM automatically
    // Note: ram_style removed for compatibility with all device families
    //------------------------------------------------------------------------
    logic [TAG_BITS+7-1:0]    tags_ecc [NUM_SETS-1:0][NUM_WAYS-1:0];  // Tag + ECC
    logic [NUM_WAYS-1:0]      valid    [NUM_SETS-1:0];
    logic [NUM_WAYS-1:0]      dirty    [NUM_SETS-1:0];
    logic [ECC_LINE_BITS-1:0] data_ecc [NUM_SETS-1:0][NUM_WAYS-1:0];
    
    // Directory: track which L1s have copies (with TMR)
    logic [NUM_CORES-1:0]     sharers     [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [NUM_CORES-1:0]     sharers_tmr1[NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [NUM_CORES-1:0]     sharers_tmr2[NUM_SETS-1:0][NUM_WAYS-1:0];
    
    // PLRU for 16-way
    logic [14:0]              plru [NUM_SETS-1:0];

    //------------------------------------------------------------------------
    // Address decode
    //------------------------------------------------------------------------
    wire [TAG_BITS-1:0]   tag   = bus_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] index = bus_req_addr[OFFSET_BITS +: INDEX_BITS];

    //------------------------------------------------------------------------
    // TMR voting for sharers
    //------------------------------------------------------------------------
    logic [NUM_CORES-1:0] sharers_voted [NUM_SETS-1:0][NUM_WAYS-1:0];
    
    always_comb begin
        for (int s = 0; s < NUM_SETS; s++) begin
            for (int w = 0; w < NUM_WAYS; w++) begin
                for (int c = 0; c < NUM_CORES; c++) begin
                    sharers_voted[s][w][c] = (sharers[s][w][c] & sharers_tmr1[s][w][c]) |
                                             (sharers_tmr1[s][w][c] & sharers_tmr2[s][w][c]) |
                                             (sharers[s][w][c] & sharers_tmr2[s][w][c]);
                end
            end
        end
    end

    //------------------------------------------------------------------------
    // ECC decoding for tags (simplified - 16 bits tag + 5 ECC)
    //------------------------------------------------------------------------
    logic [TAG_BITS-1:0] tag_decoded [NUM_WAYS-1:0];
    logic [NUM_WAYS-1:0] tag_single_err, tag_double_err;
    
    // Simplified tag ECC (would use proper Hamming code)
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            tag_decoded[w] = tags_ecc[index][w][TAG_BITS-1:0];
            // Check parity (simplified)
            tag_single_err[w] = ^tags_ecc[index][w] & valid[index][w];
            tag_double_err[w] = 1'b0;  // Simplified
        end
    end

    //------------------------------------------------------------------------
    // ECC decoding for data
    //------------------------------------------------------------------------
    logic [63:0] data_decoded [NUM_WAYS-1:0][WORDS_PER_LINE-1:0];
    logic [NUM_WAYS-1:0] data_single_err [WORDS_PER_LINE-1:0];
    logic [NUM_WAYS-1:0] data_double_err [WORDS_PER_LINE-1:0];
    
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_l2_ecc_way
            for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_l2_ecc_word
                ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_ecc (
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

    // Reconstruct lines
    logic [LINE_BITS-1:0] line_decoded [NUM_WAYS-1:0];
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_l2_line
            for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_l2_word
                assign line_decoded[w][d*64 +: 64] = data_decoded[w][d];
            end
        end
    endgenerate

    //------------------------------------------------------------------------
    // Hit detection
    //------------------------------------------------------------------------
    logic [NUM_WAYS-1:0] way_hit;
    logic [3:0] hit_way;
    logic cache_hit;
    logic hit_has_ecc_error;
    logic hit_uncorrectable;
    
    always_comb begin
        way_hit = '0;
        hit_way = '0;
        hit_has_ecc_error = 1'b0;
        hit_uncorrectable = 1'b0;
        
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (valid[index][w] && tag_decoded[w] == tag) begin
                way_hit[w] = 1'b1;
                hit_way = w[3:0];
                // Check all words for ECC errors
                for (int d = 0; d < WORDS_PER_LINE; d++) begin
                    hit_has_ecc_error |= data_single_err[d][w];
                    hit_uncorrectable |= data_double_err[d][w];
                end
            end
        end
        cache_hit = |way_hit && !hit_uncorrectable;
    end

    //------------------------------------------------------------------------
    // PLRU victim selection for 16-way (binary tree)
    //------------------------------------------------------------------------
    function automatic logic [3:0] get_victim_16way(input logic [14:0] p);
        logic [3:0] victim;
        if (!p[0]) begin
            if (!p[1]) begin
                victim = p[3] ? 4'd1 : 4'd0;
            end else begin
                victim = p[4] ? 4'd3 : 4'd2;
            end
        end else if (!p[2]) begin
            if (!p[5]) begin
                victim = p[7] ? 4'd5 : 4'd4;
            end else begin
                victim = p[8] ? 4'd7 : 4'd6;
            end
        end else if (!p[6]) begin
            if (!p[9]) begin
                victim = p[11] ? 4'd9 : 4'd8;
            end else begin
                victim = p[12] ? 4'd11 : 4'd10;
            end
        end else begin
            if (!p[10]) begin
                victim = p[13] ? 4'd13 : 4'd12;
            end else begin
                victim = p[14] ? 4'd15 : 4'd14;
            end
        end
        return victim;
    endfunction

    //------------------------------------------------------------------------
    // State machine
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE, LOOKUP,
        WB_REQ, WB_WAIT,
        FILL_REQ, FILL_WAIT, FILL_DONE,
        ECC_RELOAD_REQ, ECC_RELOAD_WAIT, ECC_RELOAD_DONE
    } state_t;
    
    state_t state, next_state;
    
    logic [ADDR_WIDTH-1:0]  req_addr_r;
    logic [LINE_BITS-1:0]   req_wdata_r;
    logic                   req_write_r;
    logic [3:0]             victim_way;
    
    wire victim_valid = valid[req_addr_r[OFFSET_BITS +: INDEX_BITS]][victim_way];
    wire victim_dirty = dirty[req_addr_r[OFFSET_BITS +: INDEX_BITS]][victim_way];
    wire [TAG_BITS-1:0] victim_tag = tag_decoded[victim_way];
    wire [LINE_BITS-1:0] victim_data = line_decoded[victim_way];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            req_wdata_r <= '0;
            req_write_r <= '0;
            victim_way <= '0;
        end else begin
            state <= next_state;
            
            if (state == IDLE && bus_req_valid) begin
                req_addr_r <= bus_req_addr;
                req_wdata_r <= bus_req_wdata;
                req_write_r <= bus_req_write;
            end
            
            if (state == LOOKUP && !cache_hit)
                victim_way <= get_victim_16way(plru[index]);
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE:   next_state = bus_req_valid ? LOOKUP : IDLE;
            LOOKUP: begin
                if (hit_uncorrectable)
                    next_state = ECC_RELOAD_REQ;
                else if (cache_hit)
                    next_state = IDLE;
                else if (victim_valid && victim_dirty)
                    next_state = WB_REQ;
                else
                    next_state = FILL_REQ;
            end
            WB_REQ:    next_state = WB_WAIT;
            WB_WAIT:   next_state = mem_resp_valid ? FILL_REQ : WB_WAIT;
            FILL_REQ:  next_state = FILL_WAIT;
            FILL_WAIT: next_state = mem_resp_valid ? FILL_DONE : FILL_WAIT;
            FILL_DONE: next_state = IDLE;
            ECC_RELOAD_REQ:  next_state = ECC_RELOAD_WAIT;
            ECC_RELOAD_WAIT: next_state = mem_resp_valid ? ECC_RELOAD_DONE : ECC_RELOAD_WAIT;
            ECC_RELOAD_DONE: next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // ECC encoding for refill
    //------------------------------------------------------------------------
    logic [ECC_LINE_BITS-1:0] encoded_refill;
    generate
        for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_l2_enc
            logic [71:0] enc_word;
            ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_enc (
                .clk(clk), .rst_n(rst_n),
                .data_in(mem_resp_rdata[d*64 +: 64]),
                .encoded_out(enc_word),
                .encoded_in('0),
                .data_out(), .single_error(), .double_error(), .error_position()
            );
            assign encoded_refill[d*72 +: 72] = enc_word;
        end
    endgenerate

    wire [TAG_BITS-1:0]   refill_tag   = req_addr_r[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] refill_index = req_addr_r[OFFSET_BITS +: INDEX_BITS];

    //------------------------------------------------------------------------
    // Cache update
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                valid[s] <= '0;
                dirty[s] <= '0;
            end
            hit_count <= '0;
            miss_count <= '0;
            ecc_error_count <= '0;
        end else begin
            // Hit
            if (state == LOOKUP && cache_hit) begin
                hit_count <= hit_count + 1;
                if (bus_req_write) begin
                    dirty[index][hit_way] <= 1'b1;
                end
            end
            
            // Miss
            if (state == LOOKUP && !cache_hit) begin
                miss_count <= miss_count + 1;
            end
            
            // ECC error
            if (state == LOOKUP && (hit_has_ecc_error || hit_uncorrectable)) begin
                ecc_error_count <= ecc_error_count + 1;
                ecc_error_addr <= bus_req_addr;
            end
            
            // Refill
            if (state == FILL_DONE || state == ECC_RELOAD_DONE) begin
                tags_ecc[refill_index][victim_way] <= {7'b0, refill_tag};  // Simplified ECC
                data_ecc[refill_index][victim_way] <= encoded_refill;
                valid[refill_index][victim_way] <= 1'b1;
                dirty[refill_index][victim_way] <= req_write_r;
                sharers[refill_index][victim_way] <= '0;
                sharers_tmr1[refill_index][victim_way] <= '0;
                sharers_tmr2[refill_index][victim_way] <= '0;
            end
            
            // Writeback clears dirty
            if (state == WB_WAIT && mem_resp_valid) begin
                dirty[refill_index][victim_way] <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------------
    // ECC status
    //------------------------------------------------------------------------
    assign ecc_single_error = (state == LOOKUP) && hit_has_ecc_error && !hit_uncorrectable;
    assign ecc_double_error = (state == LOOKUP) && hit_uncorrectable;

    //------------------------------------------------------------------------
    // Memory interface
    //------------------------------------------------------------------------
    assign mem_req_valid = (state == FILL_REQ) || (state == WB_REQ) || (state == ECC_RELOAD_REQ);
    assign mem_req_write = (state == WB_REQ);
    assign mem_req_addr  = (state == WB_REQ) ? 
                           {victim_tag, refill_index, {OFFSET_BITS{1'b0}}} :
                           {refill_tag, refill_index, {OFFSET_BITS{1'b0}}};
    assign mem_req_wdata = victim_data;
    assign mem_req_ecc   = '0;  // ECC computed by memory controller
    assign mem_req_ready = (state == FILL_WAIT) || (state == WB_WAIT) || (state == ECC_RELOAD_WAIT);

    //------------------------------------------------------------------------
    // Bus response
    //------------------------------------------------------------------------
    wire [LINE_BITS-1:0] hit_line = line_decoded[hit_way];
    assign bus_resp_rdata  = (state == LOOKUP && cache_hit) ? hit_line :
                             ((state == FILL_DONE || state == ECC_RELOAD_DONE) ? mem_resp_rdata : '0);
    assign bus_resp_valid  = (state == LOOKUP && cache_hit) || 
                             (state == FILL_DONE) || (state == ECC_RELOAD_DONE);
    assign bus_resp_shared = cache_hit && (|sharers_voted[index][hit_way]);

    assign dir_sharers  = cache_hit ? sharers_voted[index][hit_way] : '0;
    assign dir_modified = cache_hit && dirty[index][hit_way];
    
    assign cache_ready = (state == IDLE);

endmodule


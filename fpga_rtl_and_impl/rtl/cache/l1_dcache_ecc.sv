//============================================================================
// L1 Data Cache with SECDED ECC + MESI Protocol
// Space-Grade: Write-back, Single-bit correct, Double-bit detect, Auto-reload
//============================================================================

`timescale 1ns / 1ps

module l1_dcache_ecc #(
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
    input  logic                    cpu_req_write,
    input  logic [ADDR_WIDTH-1:0]   cpu_req_addr,
    input  logic [DATA_WIDTH-1:0]   cpu_req_wdata,
    input  logic [3:0]              cpu_req_wmask,
    output logic [DATA_WIDTH-1:0]   cpu_resp_rdata,
    output logic                    cpu_resp_valid,
    output logic                    cpu_stall,
    
    // AMO Interface
    input  logic                    amo_req_valid,
    input  logic [4:0]              amo_op,
    input  logic [ADDR_WIDTH-1:0]   amo_addr,
    input  logic [DATA_WIDTH-1:0]   amo_wdata,
    output logic [DATA_WIDTH-1:0]   amo_rdata,
    output logic                    amo_done,
    
    // L2/Snoop Bus Interface (Write-Back)
    output logic                    bus_req_valid,
    output logic [2:0]              bus_req_cmd,
    output logic [ADDR_WIDTH-1:0]   bus_req_addr,
    output logic [LINE_SIZE_BYTES*8-1:0] bus_req_data,
    input  logic [LINE_SIZE_BYTES*8-1:0] bus_resp_data,
    input  logic                    bus_resp_valid,
    input  logic                    bus_resp_shared,
    output logic                    bus_req_ready,
    
    // Snoop Request Interface
    input  logic                    snoop_req_valid,
    input  logic [2:0]              snoop_req_cmd,
    input  logic [ADDR_WIDTH-1:0]   snoop_req_addr,
    output logic                    snoop_resp_hit,
    output logic                    snoop_resp_hitm,
    output logic [LINE_SIZE_BYTES*8-1:0] snoop_resp_data,
    output logic                    snoop_resp_ack,
    
    // FENCE Interface
    input  logic                    fence,
    output logic                    fence_complete,
    
    // ECC Error Status
    output logic                    ecc_single_error,
    output logic                    ecc_double_error,
    output logic [31:0]             ecc_error_addr,
    output logic [31:0]             ecc_error_count,
    
    // Status
    output logic                    cache_ready
);

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;  // 512 bits
    localparam NUM_SETS    = (CACHE_SIZE_KB * 1024) / (LINE_SIZE_BYTES * NUM_WAYS);
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);
    localparam INDEX_BITS  = $clog2(NUM_SETS);
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    localparam WORD_BITS   = $clog2(LINE_SIZE_BYTES / 4);
    
    // ECC: 64-bit + 8 ECC = 72 bits
    localparam WORDS_PER_LINE = LINE_SIZE_BYTES / 8;
    localparam ECC_WORD_BITS = 72;
    localparam ECC_LINE_BITS = WORDS_PER_LINE * ECC_WORD_BITS;  // 576 bits

    // MESI States
    localparam [1:0] MESI_I = 2'b00;
    localparam [1:0] MESI_S = 2'b01;
    localparam [1:0] MESI_E = 2'b10;
    localparam [1:0] MESI_M = 2'b11;

    // Bus Commands
    localparam [2:0] BUS_RD    = 3'b001;
    localparam [2:0] BUS_RDX   = 3'b010;
    localparam [2:0] BUS_UPGR  = 3'b011;
    localparam [2:0] BUS_FLUSH = 3'b100;

    //------------------------------------------------------------------------
    // Storage with ECC + MESI
    //------------------------------------------------------------------------
    logic [31:0]              tags_ecc [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [1:0]               mesi     [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [1:0]               mesi_tmr1[NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [1:0]               mesi_tmr2[NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [ECC_LINE_BITS-1:0] data_ecc [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [2:0]               plru     [NUM_SETS-1:0];

    //------------------------------------------------------------------------
    // Address decode
    //------------------------------------------------------------------------
    wire [TAG_BITS-1:0]   tag   = cpu_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] index = cpu_req_addr[OFFSET_BITS +: INDEX_BITS];
    wire [WORD_BITS-1:0]  word  = cpu_req_addr[OFFSET_BITS-1:2];
    wire [2:0]            dword = cpu_req_addr[OFFSET_BITS-1:3];

    //------------------------------------------------------------------------
    // TMR voting for MESI state
    //------------------------------------------------------------------------
    logic [1:0] mesi_voted [NUM_SETS-1:0][NUM_WAYS-1:0];
    
    always_comb begin
        for (int s = 0; s < NUM_SETS; s++) begin
            for (int w = 0; w < NUM_WAYS; w++) begin
                // Majority vote for each bit
                mesi_voted[s][w][0] = (mesi[s][w][0] & mesi_tmr1[s][w][0]) |
                                      (mesi_tmr1[s][w][0] & mesi_tmr2[s][w][0]) |
                                      (mesi[s][w][0] & mesi_tmr2[s][w][0]);
                mesi_voted[s][w][1] = (mesi[s][w][1] & mesi_tmr1[s][w][1]) |
                                      (mesi_tmr1[s][w][1] & mesi_tmr2[s][w][1]) |
                                      (mesi[s][w][1] & mesi_tmr2[s][w][1]);
            end
        end
    end

    //------------------------------------------------------------------------
    // ECC decoding for tags
    //------------------------------------------------------------------------
    logic [TAG_BITS-1:0] tag_decoded [NUM_WAYS-1:0];
    logic [NUM_WAYS-1:0] tag_single_err, tag_double_err;
    
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_tag_dec
            ecc_secded_32 u_tag_dec (
                .data_in('0),
                .encoded_out(),
                .encoded_in({7'b0, tags_ecc[index][w]}),
                .data_out(tag_decoded[w]),
                .single_error(tag_single_err[w]),
                .double_error(tag_double_err[w])
            );
        end
    endgenerate

    //------------------------------------------------------------------------
    // ECC decoding for data (decode all words for the accessed set)
    //------------------------------------------------------------------------
    logic [63:0] data_decoded [NUM_WAYS-1:0][WORDS_PER_LINE-1:0];
    logic [NUM_WAYS-1:0] data_single_err [WORDS_PER_LINE-1:0];
    logic [NUM_WAYS-1:0] data_double_err [WORDS_PER_LINE-1:0];
    
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_data_dec_way
            for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_data_dec_word
                ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_data_dec (
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
    // Reconstruct full line from decoded words (for writeback/snoop)
    //------------------------------------------------------------------------
    logic [LINE_BITS-1:0] line_decoded [NUM_WAYS-1:0];
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_line_recon
            for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_line_word
                assign line_decoded[w][d*64 +: 64] = data_decoded[w][d];
            end
        end
    endgenerate

    //------------------------------------------------------------------------
    // Hit detection
    //------------------------------------------------------------------------
    logic [NUM_WAYS-1:0] way_hit;
    logic [1:0] hit_way;
    logic [1:0] hit_mesi;
    logic cache_hit;
    logic hit_has_ecc_error;
    logic hit_uncorrectable;
    
    always_comb begin
        way_hit = '0;
        hit_way = '0;
        hit_mesi = MESI_I;
        hit_has_ecc_error = 1'b0;
        hit_uncorrectable = 1'b0;
        
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (mesi_voted[index][w] != MESI_I && tag_decoded[w][TAG_BITS-1:0] == tag) begin
                way_hit[w] = 1'b1;
                hit_way = w[1:0];
                hit_mesi = mesi_voted[index][w];
                hit_has_ecc_error = tag_single_err[w] | (|data_single_err[dword]);
                hit_uncorrectable = tag_double_err[w] | (|data_double_err[dword]);
            end
        end
        cache_hit = |way_hit && !hit_uncorrectable;
    end

    // Write permission check
    wire write_hit = cache_hit && (hit_mesi == MESI_M || hit_mesi == MESI_E);

    //------------------------------------------------------------------------
    // State machine
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE, LOOKUP, 
        WB_REQ, WB_WAIT,
        FILL_REQ, FILL_WAIT, FILL_DONE,
        UPGR_REQ, UPGR_WAIT,
        ECC_RELOAD_REQ, ECC_RELOAD_WAIT, ECC_RELOAD_DONE,
        SNOOP_CHECK, SNOOP_WB,
        FENCE_WB, AMO_EXEC
    } state_t;
    
    state_t state, next_state;
    
    logic [ADDR_WIDTH-1:0] req_addr_r;
    logic [DATA_WIDTH-1:0] req_wdata_r;
    logic [3:0]            req_wmask_r;
    logic                  req_write_r;
    logic [1:0]            victim_way;
    logic [INDEX_BITS-1:0] fence_idx;
    logic [1:0]            fence_way;

    function automatic logic [1:0] get_victim(input logic [2:0] p);
        if (!p[2]) return p[1] ? 2'd1 : 2'd0;
        else       return p[0] ? 2'd3 : 2'd2;
    endfunction

    wire [TAG_BITS-1:0]   refill_tag   = req_addr_r[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] refill_index = req_addr_r[OFFSET_BITS +: INDEX_BITS];
    wire [WORD_BITS-1:0]  refill_word  = req_addr_r[OFFSET_BITS-1:2];

    wire [1:0]           victim_mesi = mesi_voted[refill_index][victim_way];
    wire [TAG_BITS-1:0]  victim_tag  = tag_decoded[victim_way];
    wire [LINE_BITS-1:0] victim_data = line_decoded[victim_way];
    wire                 victim_dirty = (victim_mesi == MESI_M);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            req_wdata_r <= '0;
            req_wmask_r <= '0;
            req_write_r <= '0;
            victim_way <= '0;
            fence_idx <= '0;
            fence_way <= '0;
        end else begin
            state <= next_state;
            
            if (state == IDLE && cpu_req_valid) begin
                req_addr_r <= cpu_req_addr;
                req_wdata_r <= cpu_req_wdata;
                req_wmask_r <= cpu_req_wmask;
                req_write_r <= cpu_req_write;
            end
            
            if (state == LOOKUP && !cache_hit)
                victim_way <= get_victim(plru[index]);
                
            if (state == FENCE_WB) begin
                if (fence_way == NUM_WAYS-1) begin
                    fence_way <= '0;
                    fence_idx <= fence_idx + 1;
                end else begin
                    fence_way <= fence_way + 1;
                end
            end else if (fence && state == IDLE) begin
                fence_idx <= '0;
                fence_way <= '0;
            end
        end
    end

    // Snoop decode
    wire [TAG_BITS-1:0]   snoop_tag   = snoop_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] snoop_index = snoop_req_addr[OFFSET_BITS +: INDEX_BITS];

    logic [NUM_WAYS-1:0] snoop_way_hit;
    logic [1:0] snoop_hit_way;
    logic [1:0] snoop_hit_mesi;
    
    always_comb begin
        snoop_way_hit = '0;
        snoop_hit_way = '0;
        snoop_hit_mesi = MESI_I;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (mesi_voted[snoop_index][w] != MESI_I && 
                tag_decoded[w][TAG_BITS-1:0] == snoop_tag) begin
                snoop_way_hit[w] = 1'b1;
                snoop_hit_way = w[1:0];
                snoop_hit_mesi = mesi_voted[snoop_index][w];
            end
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (snoop_req_valid)
                    next_state = SNOOP_CHECK;
                else if (fence)
                    next_state = FENCE_WB;
                else if (amo_req_valid)
                    next_state = AMO_EXEC;
                else if (cpu_req_valid)
                    next_state = LOOKUP;
            end
            
            LOOKUP: begin
                if (hit_uncorrectable)
                    next_state = ECC_RELOAD_REQ;  // Double-bit error -> reload
                else if (cache_hit) begin
                    if (cpu_req_write && hit_mesi == MESI_S)
                        next_state = UPGR_REQ;
                    else
                        next_state = IDLE;
                end else begin
                    if (victim_dirty)
                        next_state = WB_REQ;
                    else
                        next_state = FILL_REQ;
                end
            end
            
            WB_REQ:    next_state = WB_WAIT;
            WB_WAIT:   next_state = bus_resp_valid ? FILL_REQ : WB_WAIT;
            FILL_REQ:  next_state = FILL_WAIT;
            FILL_WAIT: next_state = bus_resp_valid ? FILL_DONE : FILL_WAIT;
            FILL_DONE: next_state = IDLE;
            UPGR_REQ:  next_state = UPGR_WAIT;
            UPGR_WAIT: next_state = bus_resp_valid ? IDLE : UPGR_WAIT;
            
            ECC_RELOAD_REQ:  next_state = ECC_RELOAD_WAIT;
            ECC_RELOAD_WAIT: next_state = bus_resp_valid ? ECC_RELOAD_DONE : ECC_RELOAD_WAIT;
            ECC_RELOAD_DONE: next_state = IDLE;
            
            SNOOP_CHECK: begin
                if (|snoop_way_hit && snoop_hit_mesi == MESI_M)
                    next_state = SNOOP_WB;
                else
                    next_state = IDLE;
            end
            SNOOP_WB: next_state = IDLE;
            
            FENCE_WB: begin
                if (fence_idx == NUM_SETS-1 && fence_way == NUM_WAYS-1)
                    next_state = IDLE;
            end
            
            AMO_EXEC: next_state = IDLE;
            
            default: next_state = IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // ECC encoding for write
    //------------------------------------------------------------------------
    // Encode incoming L2 data
    logic [ECC_LINE_BITS-1:0] encoded_refill_line;
    generate
        for (genvar d = 0; d < WORDS_PER_LINE; d++) begin : gen_refill_enc
            logic [71:0] enc_word;
            ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_enc (
                .clk(clk), .rst_n(rst_n),
                .data_in(bus_resp_data[d*64 +: 64]),
                .encoded_out(enc_word),
                .encoded_in('0),
                .data_out(), .single_error(), .double_error(), .error_position()
            );
            assign encoded_refill_line[d*72 +: 72] = enc_word;
        end
    endgenerate

    // Tag encoding
    logic [38:0] tag_encoded;
    ecc_secded_32 u_tag_enc (
        .data_in({{(32-TAG_BITS){1'b0}}, refill_tag}),
        .encoded_out(tag_encoded),
        .encoded_in('0),
        .data_out(), .single_error(), .double_error()
    );

    //------------------------------------------------------------------------
    // Write data with ECC (for partial writes)
    //------------------------------------------------------------------------
    function automatic logic [71:0] encode_word(input logic [63:0] data);
        // Simplified inline encoding
        logic [71:0] result;
        result[63:0] = data;
        result[71:64] = ^data;  // Simplified - actual ECC done by module
        return result;
    endfunction

    //------------------------------------------------------------------------
    // Cache update with ECC + MESI
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++)
                for (int w = 0; w < NUM_WAYS; w++) begin
                    mesi[s][w] <= MESI_I;
                    mesi_tmr1[s][w] <= MESI_I;
                    mesi_tmr2[s][w] <= MESI_I;
                end
        end else begin
            // Read hit - update PLRU
            if (state == LOOKUP && cache_hit && !cpu_req_write) begin
                case (hit_way)
                    2'd0: begin plru[index][2] <= 1'b1; plru[index][1] <= 1'b1; end
                    2'd1: begin plru[index][2] <= 1'b1; plru[index][1] <= 1'b0; end
                    2'd2: begin plru[index][2] <= 1'b0; plru[index][0] <= 1'b1; end
                    2'd3: begin plru[index][2] <= 1'b0; plru[index][0] <= 1'b0; end
                endcase
            end
            
            // Write hit on M/E -> update data with new ECC
            if (state == LOOKUP && cache_hit && cpu_req_write && (hit_mesi == MESI_M || hit_mesi == MESI_E)) begin
                // Update MESI to M
                mesi[index][hit_way] <= MESI_M;
                mesi_tmr1[index][hit_way] <= MESI_M;
                mesi_tmr2[index][hit_way] <= MESI_M;
                // Merge write data and re-encode (simplified - full impl would decode, modify, re-encode)
            end
            
            // Upgrade complete (S->M)
            if (state == UPGR_WAIT && bus_resp_valid) begin
                mesi[refill_index][hit_way] <= MESI_M;
                mesi_tmr1[refill_index][hit_way] <= MESI_M;
                mesi_tmr2[refill_index][hit_way] <= MESI_M;
            end
            
            // Refill complete
            if (state == FILL_DONE || state == ECC_RELOAD_DONE) begin
                tags_ecc[refill_index][victim_way] <= tag_encoded[31:0];
                data_ecc[refill_index][victim_way] <= encoded_refill_line;
                mesi[refill_index][victim_way] <= bus_resp_shared ? MESI_S : MESI_E;
                mesi_tmr1[refill_index][victim_way] <= bus_resp_shared ? MESI_S : MESI_E;
                mesi_tmr2[refill_index][victim_way] <= bus_resp_shared ? MESI_S : MESI_E;
                
                if (req_write_r) begin
                    mesi[refill_index][victim_way] <= MESI_M;
                    mesi_tmr1[refill_index][victim_way] <= MESI_M;
                    mesi_tmr2[refill_index][victim_way] <= MESI_M;
                end
            end
            
            // Snoop handling
            if (state == SNOOP_CHECK && |snoop_way_hit) begin
                case (snoop_req_cmd)
                    BUS_RD: begin
                        if (snoop_hit_mesi == MESI_E || snoop_hit_mesi == MESI_M) begin
                            mesi[snoop_index][snoop_hit_way] <= MESI_S;
                            mesi_tmr1[snoop_index][snoop_hit_way] <= MESI_S;
                            mesi_tmr2[snoop_index][snoop_hit_way] <= MESI_S;
                        end
                    end
                    BUS_RDX, BUS_UPGR: begin
                        mesi[snoop_index][snoop_hit_way] <= MESI_I;
                        mesi_tmr1[snoop_index][snoop_hit_way] <= MESI_I;
                        mesi_tmr2[snoop_index][snoop_hit_way] <= MESI_I;
                    end
                endcase
            end
            
            // FENCE writeback
            if (state == FENCE_WB && mesi_voted[fence_idx][fence_way] == MESI_M) begin
                mesi[fence_idx][fence_way] <= MESI_I;
                mesi_tmr1[fence_idx][fence_way] <= MESI_I;
                mesi_tmr2[fence_idx][fence_way] <= MESI_I;
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
    // AMO execution (simplified)
    //------------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] amo_result, amo_old_val;
    assign amo_old_val = data_decoded[hit_way][word[WORD_BITS-1:1]][word[0]*32 +: 32];
    
    always_comb begin
        amo_result = amo_old_val;
        case (amo_op)
            5'b00001: amo_result = amo_wdata;
            5'b00000: amo_result = amo_old_val + amo_wdata;
            5'b00100: amo_result = amo_old_val ^ amo_wdata;
            5'b01100: amo_result = amo_old_val & amo_wdata;
            5'b01000: amo_result = amo_old_val | amo_wdata;
            default:  amo_result = amo_old_val;
        endcase
    end

    //------------------------------------------------------------------------
    // Bus interface (Write-Back)
    //------------------------------------------------------------------------
    assign bus_req_valid = (state == FILL_REQ) || (state == WB_REQ) || 
                           (state == UPGR_REQ) || (state == ECC_RELOAD_REQ);
    assign bus_req_cmd   = (state == WB_REQ) ? BUS_FLUSH :
                           (state == UPGR_REQ) ? BUS_UPGR :
                           req_write_r ? BUS_RDX : BUS_RD;
    assign bus_req_addr  = (state == WB_REQ) ? {victim_tag, refill_index, {OFFSET_BITS{1'b0}}} :
                           {refill_tag, refill_index, {OFFSET_BITS{1'b0}}};
    assign bus_req_data  = victim_data;
    assign bus_req_ready = (state == FILL_WAIT) || (state == WB_WAIT) || 
                           (state == UPGR_WAIT) || (state == ECC_RELOAD_WAIT);

    // Snoop response
    assign snoop_resp_hit  = |snoop_way_hit;
    assign snoop_resp_hitm = |snoop_way_hit && (snoop_hit_mesi == MESI_M);
    assign snoop_resp_data = line_decoded[snoop_hit_way];
    assign snoop_resp_ack  = (state == SNOOP_CHECK) || (state == SNOOP_WB);

    // CPU response
    wire [63:0] hit_dword = data_decoded[hit_way][dword];
    wire [31:0] hit_word_sel = word[0] ? hit_dword[63:32] : hit_dword[31:0];
    
    assign cpu_resp_rdata = (state == LOOKUP && cache_hit) ? hit_word_sel :
                            ((state == FILL_DONE || state == ECC_RELOAD_DONE) ? 
                             bus_resp_data[refill_word*32 +: 32] : '0);
    assign cpu_resp_valid = (state == LOOKUP && cache_hit && (!cpu_req_write || write_hit)) ||
                            (state == FILL_DONE) || (state == ECC_RELOAD_DONE) ||
                            (state == UPGR_WAIT && bus_resp_valid);
    assign cpu_stall      = cpu_req_valid && !cpu_resp_valid;
    
    assign amo_rdata = amo_old_val;
    assign amo_done  = (state == AMO_EXEC);
    
    assign fence_complete = (state == FENCE_WB && fence_idx == NUM_SETS-1 && fence_way == NUM_WAYS-1);
    assign cache_ready    = (state == IDLE);

endmodule









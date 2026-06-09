//============================================================================
// L2 Unified Cache - Configurable Size, 16-way Set Associative
// Redesigned for guaranteed BRAM inference
// Structure: Separate BRAMs per way, banked data storage
//============================================================================

`timescale 1ns / 1ps

module l2_cache_bram #(
    parameter CACHE_SIZE_KB   = 2048,   // 2MB L2 cache for full design
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
    localparam CACHE_BYTES = CACHE_SIZE_KB * 1024;
    localparam NUM_SETS    = CACHE_BYTES / (LINE_SIZE_BYTES * NUM_WAYS);
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);  // 6 bits
    localparam INDEX_BITS  = $clog2(NUM_SETS);         // e.g., 11 bits for 2048 sets
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;

    // Data banking - 8 banks of 64 bits each
    localparam NUM_DATA_BANKS = 8;
    localparam BANK_WIDTH = LINE_BITS / NUM_DATA_BANKS;  // 64 bits

    // Address decode
    wire [TAG_BITS-1:0]   req_tag   = bus_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] req_index = bus_req_addr[OFFSET_BITS +: INDEX_BITS];

    // Control arrays - Use distributed RAM (LUTRAM) for small arrays
    (* ram_style = "distributed" *) logic [NUM_WAYS-1:0]   valid [NUM_SETS-1:0];
    (* ram_style = "distributed" *) logic [NUM_WAYS-1:0]   dirty [NUM_SETS-1:0];
    (* ram_style = "distributed" *) logic [14:0]           plru  [NUM_SETS-1:0];
    
    // Sharers directory - use simplified per-set tracking instead of per-line
    // This reduces from NUM_SETS*NUM_WAYS*NUM_CORES to NUM_SETS*NUM_CORES bits
    // Each bit indicates if that core has ANY line from this set
    (* ram_style = "distributed" *) logic [NUM_CORES-1:0]  set_sharers [NUM_SETS-1:0];
    
    // For backwards compatibility, create sharers output based on set_sharers
    logic [NUM_CORES-1:0]  sharers_local;

    // Tag BRAM interfaces - one per way
    logic                    tag_wr_en   [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   tag_wr_addr [NUM_WAYS-1:0];
    logic [TAG_BITS-1:0]     tag_wr_data [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   tag_rd_addr [NUM_WAYS-1:0];
    logic [TAG_BITS-1:0]     tag_rd_data [NUM_WAYS-1:0];

    // Data BRAM interfaces - one per way per bank
    logic                     data_wr_en   [NUM_WAYS-1:0][NUM_DATA_BANKS-1:0];
    logic [INDEX_BITS-1:0]    data_wr_addr [NUM_WAYS-1:0][NUM_DATA_BANKS-1:0];
    logic [BANK_WIDTH-1:0]    data_wr_data [NUM_WAYS-1:0][NUM_DATA_BANKS-1:0];
    logic [INDEX_BITS-1:0]    data_rd_addr [NUM_WAYS-1:0][NUM_DATA_BANKS-1:0];
    logic [BANK_WIDTH-1:0]    data_rd_data [NUM_WAYS-1:0][NUM_DATA_BANKS-1:0];

    // Generate BRAM instances
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_way
            // Tag BRAM per way
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
            
            // Data BRAMs - one per bank per way
            for (genvar b = 0; b < NUM_DATA_BANKS; b++) begin : gen_bank
                bram_sdp #(
                    .DATA_WIDTH(BANK_WIDTH),
                    .ADDR_WIDTH(INDEX_BITS),
                    .DEPTH(NUM_SETS)
                ) u_data_ram (
                    .clk(clk),
                    .wr_en(data_wr_en[w][b]),
                    .wr_addr(data_wr_addr[w][b]),
                    .wr_data(data_wr_data[w][b]),
                    .rd_en(1'b1),
                    .rd_addr(data_rd_addr[w][b]),
                    .rd_data(data_rd_data[w][b])
                );
            end
        end
    endgenerate

    // State machine
    typedef enum logic [2:0] {
        IDLE, LOOKUP, WB_REQ, WB_WAIT, FILL_REQ, FILL_WAIT, FILL_DONE
    } state_t;
    
    state_t state, next_state;
    
    // Request registers
    logic [ADDR_WIDTH-1:0]  req_addr_r;
    logic [LINE_BITS-1:0]   req_wdata_r;
    logic                   req_write_r;
    logic [TAG_BITS-1:0]    req_tag_r;
    logic [INDEX_BITS-1:0]  req_index_r;
    logic [3:0]             victim_way;

    // Hit detection (registered - 1 cycle after address presented)
    logic [NUM_WAYS-1:0] way_hit_r;
    logic [3:0] hit_way_r;
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
                if (valid[req_index_r][w] && tag_rd_data[w] == req_tag_r) begin
                    way_hit_r[w] <= 1'b1;
                    hit_way_r <= w[3:0];
                end
            end
            cache_hit_r <= |way_hit_r;
        end
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

    // Victim info
    wire victim_valid = valid[req_index_r][victim_way];
    wire victim_dirty = dirty[req_index_r][victim_way];
    wire [TAG_BITS-1:0] victim_tag = tag_rd_data[victim_way];

    // Assemble victim data line from banks
    logic [LINE_BITS-1:0] victim_line;
    always_comb begin
        for (int b = 0; b < NUM_DATA_BANKS; b++) begin
            victim_line[b*BANK_WIDTH +: BANK_WIDTH] = data_rd_data[victim_way][b];
        end
    end

    // State machine control
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            req_wdata_r <= '0;
            req_write_r <= '0;
            req_tag_r <= '0;
            req_index_r <= '0;
            victim_way <= '0;
        end else begin
            state <= next_state;
            
            if (state == IDLE && bus_req_valid) begin
                req_addr_r <= bus_req_addr;
                req_wdata_r <= bus_req_wdata;
                req_write_r <= bus_req_write;
                req_tag_r <= req_tag;
                req_index_r <= req_index;
            end
            
            if (state == LOOKUP && !cache_hit_r) begin
                victim_way <= get_victim_16way(plru[req_index_r]);
            end
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE:      next_state = bus_req_valid ? LOOKUP : IDLE;
            LOOKUP:    next_state = cache_hit_r ? IDLE : (victim_valid && victim_dirty ? WB_REQ : FILL_REQ);
            WB_REQ:    next_state = WB_WAIT;
            WB_WAIT:   next_state = mem_resp_valid ? FILL_REQ : WB_WAIT;
            FILL_REQ:  next_state = FILL_WAIT;
            FILL_WAIT: next_state = mem_resp_valid ? FILL_DONE : FILL_WAIT;
            FILL_DONE: next_state = IDLE;
            default:   next_state = IDLE;
        endcase
    end

    // Tag RAM control
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            tag_rd_addr[w] = (state == IDLE) ? req_index : req_index_r;
            tag_wr_en[w] = (state == FILL_DONE) && (w[3:0] == victim_way);
            tag_wr_addr[w] = req_index_r;
            tag_wr_data[w] = req_tag_r;
        end
    end

    // Data RAM control
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            for (int b = 0; b < NUM_DATA_BANKS; b++) begin
                data_rd_addr[w][b] = (state == IDLE) ? req_index : req_index_r;
                data_wr_en[w][b] = (state == FILL_DONE) && (w[3:0] == victim_way);
                data_wr_addr[w][b] = req_index_r;
                data_wr_data[w][b] = req_write_r ? 
                    req_wdata_r[b*BANK_WIDTH +: BANK_WIDTH] : 
                    mem_resp_rdata[b*BANK_WIDTH +: BANK_WIDTH];
            end
        end
    end

    // Valid/Dirty/PLRU/Sharers update - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                valid[s] <= '0;
                dirty[s] <= '0;
                plru[s] <= '0;
                set_sharers[s] <= '0;
            end
            hit_count <= '0;
            miss_count <= '0;
            sharers_local <= '0;
        end else begin
            if (state == LOOKUP && cache_hit_r) begin
                hit_count <= hit_count + 1;
                if (req_write_r)
                    dirty[req_index_r][hit_way_r] <= 1'b1;
                // Capture sharers for output
                sharers_local <= set_sharers[req_index_r];
            end
            if (state == LOOKUP && !cache_hit_r)
                miss_count <= miss_count + 1;
            if (state == FILL_DONE) begin
                valid[req_index_r][victim_way] <= 1'b1;
                dirty[req_index_r][victim_way] <= req_write_r;
                // Clear sharers when we fill a new line
                set_sharers[req_index_r] <= '0;
            end
            if (state == WB_WAIT && mem_resp_valid)
                dirty[req_index_r][victim_way] <= 1'b0;
        end
    end

    // Assemble hit data line from banks
    logic [LINE_BITS-1:0] hit_line;
    always_comb begin
        for (int b = 0; b < NUM_DATA_BANKS; b++) begin
            hit_line[b*BANK_WIDTH +: BANK_WIDTH] = data_rd_data[hit_way_r][b];
        end
    end

    // Memory interface
    assign mem_req_valid = (state == FILL_REQ) || (state == WB_REQ);
    assign mem_req_write = (state == WB_REQ);
    assign mem_req_addr  = (state == WB_REQ) ? 
                           {victim_tag, req_index_r, {OFFSET_BITS{1'b0}}} :
                           {req_tag_r, req_index_r, {OFFSET_BITS{1'b0}}};
    assign mem_req_wdata = victim_line;
    assign mem_req_ready = (state == FILL_WAIT) || (state == WB_WAIT);

    // Bus response
    assign bus_resp_rdata  = (state == LOOKUP && cache_hit_r) ? hit_line :
                             (state == FILL_DONE) ? mem_resp_rdata : '0;
    assign bus_resp_valid  = (state == LOOKUP && cache_hit_r) || (state == FILL_DONE);
    assign bus_resp_shared = cache_hit_r && (|set_sharers[req_index_r]);

    // Directory outputs - use registered sharers for timing
    assign dir_sharers  = sharers_local;
    assign dir_modified = cache_hit_r && dirty[req_index_r][hit_way_r];
    
    assign cache_ready = (state == IDLE);

endmodule


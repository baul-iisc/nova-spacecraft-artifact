//============================================================================
// L1 Data Cache - 16KB, 4-way Set Associative with MESI Protocol
// Redesigned for guaranteed BRAM inference using explicit BRAM modules
//============================================================================

`timescale 1ns / 1ps

module l1_dcache_bram #(
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
    
    // L2/Snoop Bus Interface
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
    
    // Status
    output logic                    cache_ready
);

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;  // 512 bits
    localparam NUM_SETS    = (CACHE_SIZE_KB * 1024) / (LINE_SIZE_BYTES * NUM_WAYS);  // 64 sets
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);  // 6 bits
    localparam INDEX_BITS  = $clog2(NUM_SETS);         // 6 bits
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;  // 20 bits
    localparam WORD_BITS   = $clog2(LINE_SIZE_BYTES / 4);  // 4 bits

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

    // Address decode
    wire [TAG_BITS-1:0]   req_tag   = cpu_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] req_index = cpu_req_addr[OFFSET_BITS +: INDEX_BITS];
    wire [WORD_BITS-1:0]  req_word  = cpu_req_addr[OFFSET_BITS-1:2];

    // Control arrays - small, use registers
    logic [1:0]          mesi  [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [2:0]          plru  [NUM_SETS-1:0];

    // Tag BRAM interfaces
    logic                    tag_wr_en   [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   tag_wr_addr [NUM_WAYS-1:0];
    logic [TAG_BITS-1:0]     tag_wr_data [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   tag_rd_addr [NUM_WAYS-1:0];
    logic [TAG_BITS-1:0]     tag_rd_data [NUM_WAYS-1:0];

    // Data BRAM interfaces with byte enables
    logic                    data_wr_en   [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   data_wr_addr [NUM_WAYS-1:0];
    logic [LINE_BITS-1:0]    data_wr_data [NUM_WAYS-1:0];
    logic [LINE_BITS/8-1:0]  data_wr_be   [NUM_WAYS-1:0];
    logic [INDEX_BITS-1:0]   data_rd_addr [NUM_WAYS-1:0];
    logic [LINE_BITS-1:0]    data_rd_data [NUM_WAYS-1:0];

    // Generate BRAM instances for each way
    generate
        for (genvar w = 0; w < NUM_WAYS; w++) begin : gen_way_bram
            // Tag BRAM
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
            
            // Data BRAM with byte enables
            bram_byte_wr #(
                .DATA_WIDTH(LINE_BITS),
                .ADDR_WIDTH(INDEX_BITS),
                .DEPTH(NUM_SETS)
            ) u_data_ram (
                .clk(clk),
                .wr_en(data_wr_en[w]),
                .wr_addr(data_wr_addr[w]),
                .wr_data(data_wr_data[w]),
                .wr_be(data_wr_be[w]),
                .rd_en(1'b1),
                .rd_addr(data_rd_addr[w]),
                .rd_data(data_rd_data[w])
            );
        end
    endgenerate

    // State machine
    typedef enum logic [3:0] {
        IDLE, LOOKUP, 
        WB_REQ, WB_WAIT,
        FILL_REQ, FILL_WAIT, FILL_DONE,
        UPGR_REQ, UPGR_WAIT,
        SNOOP_CHECK, SNOOP_WB,
        FENCE_WB, AMO_EXEC
    } state_t;
    
    state_t state, next_state;
    
    // Request registers
    logic [ADDR_WIDTH-1:0] req_addr_r;
    logic [DATA_WIDTH-1:0] req_wdata_r;
    logic [3:0]            req_wmask_r;
    logic                  req_write_r;
    logic [TAG_BITS-1:0]   req_tag_r;
    logic [INDEX_BITS-1:0] req_index_r;
    logic [WORD_BITS-1:0]  req_word_r;
    logic [1:0]            victim_way;
    logic [INDEX_BITS-1:0] fence_idx;
    logic [1:0]            fence_way;

    // Hit detection (registered from BRAM reads)
    logic [NUM_WAYS-1:0] way_hit_r;
    logic [1:0] hit_way_r;
    logic [1:0] hit_mesi_r;
    logic cache_hit_r;
    logic write_hit_r;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            way_hit_r <= '0;
            hit_way_r <= '0;
            hit_mesi_r <= MESI_I;
            cache_hit_r <= '0;
            write_hit_r <= '0;
        end else begin
            way_hit_r <= '0;
            hit_way_r <= '0;
            hit_mesi_r <= MESI_I;
            for (int w = 0; w < NUM_WAYS; w++) begin
                if (mesi[req_index_r][w] != MESI_I && tag_rd_data[w] == req_tag_r) begin
                    way_hit_r[w] <= 1'b1;
                    hit_way_r <= w[1:0];
                    hit_mesi_r <= mesi[req_index_r][w];
                end
            end
            cache_hit_r <= |way_hit_r;
            write_hit_r <= cache_hit_r && (hit_mesi_r == MESI_M || hit_mesi_r == MESI_E);
        end
    end

    // PLRU victim selection
    function automatic logic [1:0] get_victim(input logic [2:0] p);
        if (!p[2]) return p[1] ? 2'd1 : 2'd0;
        else       return p[0] ? 2'd3 : 2'd2;
    endfunction

    // Victim info
    wire [1:0]           victim_mesi = mesi[req_index_r][victim_way];
    wire [TAG_BITS-1:0]  victim_tag  = tag_rd_data[victim_way];
    wire [LINE_BITS-1:0] victim_data = data_rd_data[victim_way];
    wire                 victim_dirty = (victim_mesi == MESI_M);

    // State machine control
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            req_wdata_r <= '0;
            req_wmask_r <= '0;
            req_write_r <= '0;
            req_tag_r <= '0;
            req_index_r <= '0;
            req_word_r <= '0;
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
                req_tag_r <= req_tag;
                req_index_r <= req_index;
                req_word_r <= req_word;
            end
            
            if (state == LOOKUP && !cache_hit_r)
                victim_way <= get_victim(plru[req_index_r]);
                
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

    // Snoop address decode
    wire [TAG_BITS-1:0]   snoop_tag   = snoop_req_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] snoop_index = snoop_req_addr[OFFSET_BITS +: INDEX_BITS];

    // Snoop hit detection
    logic [NUM_WAYS-1:0] snoop_way_hit;
    logic [1:0] snoop_hit_way;
    logic [1:0] snoop_hit_mesi;
    
    always_comb begin
        snoop_way_hit = '0;
        snoop_hit_way = '0;
        snoop_hit_mesi = MESI_I;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (mesi[snoop_index][w] != MESI_I && tag_rd_data[w] == snoop_tag) begin
                snoop_way_hit[w] = 1'b1;
                snoop_hit_way = w[1:0];
                snoop_hit_mesi = mesi[snoop_index][w];
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
                if (cache_hit_r) begin
                    if (req_write_r && hit_mesi_r == MESI_S)
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

    // Tag RAM control
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            tag_rd_addr[w] = (state == IDLE) ? req_index : 
                             (snoop_req_valid) ? snoop_index : req_index_r;
            tag_wr_en[w] = (state == FILL_DONE) && (w[1:0] == victim_way);
            tag_wr_addr[w] = req_index_r;
            tag_wr_data[w] = req_tag_r;
        end
    end

    // Data RAM control - with byte enable for writes
    logic [LINE_BITS-1:0] write_line;
    logic [LINE_BITS/8-1:0] write_be;
    
    always_comb begin
        // Default write line from bus response
        write_line = bus_resp_data;
        write_be = '1;  // Full line write
        
        // For write hits, merge CPU data
        if (state == LOOKUP && cache_hit_r && req_write_r && write_hit_r) begin
            write_line = data_rd_data[hit_way_r];
            write_be = '0;
            // Set byte enables for the specific word
            for (int b = 0; b < 4; b++) begin
                if (req_wmask_r[b])
                    write_be[req_word_r*4 + b] = 1'b1;
            end
            // Merge write data
            for (int b = 0; b < 4; b++) begin
                if (req_wmask_r[b])
                    write_line[(req_word_r*32) + b*8 +: 8] = req_wdata_r[b*8 +: 8];
            end
        end
        
        // For refill with write, merge CPU data
        if (state == FILL_DONE && req_write_r) begin
            for (int b = 0; b < 4; b++) begin
                if (req_wmask_r[b])
                    write_line[(req_word_r*32) + b*8 +: 8] = req_wdata_r[b*8 +: 8];
            end
        end
    end
    
    always_comb begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            data_rd_addr[w] = (state == IDLE) ? req_index : 
                              (snoop_req_valid) ? snoop_index : req_index_r;
            data_wr_en[w] = ((state == FILL_DONE) && (w[1:0] == victim_way)) ||
                            ((state == LOOKUP) && cache_hit_r && req_write_r && write_hit_r && (w[1:0] == hit_way_r));
            data_wr_addr[w] = req_index_r;
            data_wr_data[w] = write_line;
            data_wr_be[w] = write_be;
        end
    end

    // MESI and PLRU update
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                for (int w = 0; w < NUM_WAYS; w++)
                    mesi[s][w] <= MESI_I;
                plru[s] <= '0;
            end
        end else begin
            // Read hit - update PLRU
            if (state == LOOKUP && cache_hit_r && !req_write_r) begin
                case (hit_way_r)
                    2'd0: begin plru[req_index_r][2] <= 1'b1; plru[req_index_r][1] <= 1'b1; end
                    2'd1: begin plru[req_index_r][2] <= 1'b1; plru[req_index_r][1] <= 1'b0; end
                    2'd2: begin plru[req_index_r][2] <= 1'b0; plru[req_index_r][0] <= 1'b1; end
                    2'd3: begin plru[req_index_r][2] <= 1'b0; plru[req_index_r][0] <= 1'b0; end
                endcase
            end
            
            // Write hit on M/E - update MESI to M
            if (state == LOOKUP && cache_hit_r && req_write_r && write_hit_r) begin
                mesi[req_index_r][hit_way_r] <= MESI_M;
            end
            
            // Upgrade complete (S->M)
            if (state == UPGR_WAIT && bus_resp_valid) begin
                mesi[req_index_r][hit_way_r] <= MESI_M;
            end
            
            // Refill complete
            if (state == FILL_DONE) begin
                mesi[req_index_r][victim_way] <= req_write_r ? MESI_M : (bus_resp_shared ? MESI_S : MESI_E);
            end
            
            // Snoop handling
            if (state == SNOOP_CHECK && |snoop_way_hit) begin
                case (snoop_req_cmd)
                    BUS_RD: begin
                        if (snoop_hit_mesi == MESI_E || snoop_hit_mesi == MESI_M)
                            mesi[snoop_index][snoop_hit_way] <= MESI_S;
                    end
                    BUS_RDX, BUS_UPGR: begin
                        mesi[snoop_index][snoop_hit_way] <= MESI_I;
                    end
                    default: ;
                endcase
            end
            
            // FENCE writeback
            if (state == FENCE_WB && mesi[fence_idx][fence_way] == MESI_M) begin
                mesi[fence_idx][fence_way] <= MESI_I;
            end
        end
    end

    // AMO execution (simplified)
    assign amo_rdata = data_rd_data[hit_way_r][req_word_r*32 +: 32];
    assign amo_done  = (state == AMO_EXEC);

    // Bus interface
    assign bus_req_valid = (state == FILL_REQ) || (state == WB_REQ) || (state == UPGR_REQ);
    assign bus_req_cmd   = (state == WB_REQ) ? BUS_FLUSH :
                           (state == UPGR_REQ) ? BUS_UPGR :
                           req_write_r ? BUS_RDX : BUS_RD;
    assign bus_req_addr  = (state == WB_REQ) ? {victim_tag, req_index_r, {OFFSET_BITS{1'b0}}} :
                           {req_tag_r, req_index_r, {OFFSET_BITS{1'b0}}};
    assign bus_req_data  = victim_data;
    assign bus_req_ready = (state == FILL_WAIT) || (state == WB_WAIT) || (state == UPGR_WAIT);

    // Snoop response
    assign snoop_resp_hit  = |snoop_way_hit;
    assign snoop_resp_hitm = |snoop_way_hit && (snoop_hit_mesi == MESI_M);
    assign snoop_resp_data = data_rd_data[snoop_hit_way];
    assign snoop_resp_ack  = (state == SNOOP_CHECK) || (state == SNOOP_WB);

    // CPU response
    wire [LINE_BITS-1:0] hit_line = data_rd_data[hit_way_r];
    assign cpu_resp_rdata = (state == LOOKUP && cache_hit_r) ? hit_line[req_word_r*32 +: 32] :
                            (state == FILL_DONE) ? bus_resp_data[req_word_r*32 +: 32] : '0;
    assign cpu_resp_valid = (state == LOOKUP && cache_hit_r && (!req_write_r || write_hit_r)) ||
                            (state == FILL_DONE) ||
                            (state == UPGR_WAIT && bus_resp_valid);
    assign cpu_stall      = cpu_req_valid && !cpu_resp_valid;
    
    // FENCE
    assign fence_complete = (state == FENCE_WB && fence_idx == NUM_SETS-1 && fence_way == NUM_WAYS-1);
    assign cache_ready    = (state == IDLE);

endmodule


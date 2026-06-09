//============================================================================
// L1 Data Cache - 16KB, 4-way Set Associative with MESI Protocol
// Fixed for Xilinx BRAM inference - uses synchronous reset for control,
// no reset for data/tag arrays
//============================================================================

`timescale 1ns / 1ps

module l1_dcache #(
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

    localparam LINE_BITS   = LINE_SIZE_BYTES * 8;
    localparam NUM_SETS    = (CACHE_SIZE_KB * 1024) / (LINE_SIZE_BYTES * NUM_WAYS);
    localparam OFFSET_BITS = $clog2(LINE_SIZE_BYTES);
    localparam INDEX_BITS  = $clog2(NUM_SETS);
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    localparam WORD_BITS   = $clog2(LINE_SIZE_BYTES / 4);

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

    // Banked data storage for BRAM inference
    localparam NUM_BANKS = 8;
    localparam BANK_WIDTH = LINE_BITS / NUM_BANKS;  // 64 bits per bank
    
    // Tag RAM - NO RESET for BRAM inference
    (* ram_style = "block" *) logic [TAG_BITS-1:0] tags [NUM_SETS-1:0][NUM_WAYS-1:0];
    
    // Data RAMs - NO RESET, banked for BRAM inference
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank0 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank1 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank2 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank3 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank4 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank5 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank6 [NUM_SETS-1:0][NUM_WAYS-1:0];
    (* ram_style = "block" *) logic [BANK_WIDTH-1:0] data_bank7 [NUM_SETS-1:0][NUM_WAYS-1:0];
    
    // MESI state and PLRU - small, can use distributed RAM
    logic [1:0]  mesi [NUM_SETS-1:0][NUM_WAYS-1:0];
    logic [2:0]  plru [NUM_SETS-1:0];

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
    wire [1:0]            byte_off = cpu_req_addr[1:0];

    // Hit detection
    logic [NUM_WAYS-1:0] way_hit;
    logic [1:0] hit_way;
    logic [1:0] hit_mesi;
    logic cache_hit;
    
    always_comb begin
        way_hit = '0;
        hit_way = '0;
        hit_mesi = MESI_I;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (mesi[index][w] != MESI_I && tags[index][w] == tag) begin
                way_hit[w] = 1'b1;
                hit_way = w[1:0];
                hit_mesi = mesi[index][w];
            end
        end
        cache_hit = |way_hit;
    end

    wire write_hit = cache_hit && (hit_mesi == MESI_M || hit_mesi == MESI_E);

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
    
    logic [ADDR_WIDTH-1:0] req_addr_r;
    logic [DATA_WIDTH-1:0] req_wdata_r;
    logic [3:0]            req_wmask_r;
    logic                  req_write_r;
    logic [1:0]            victim_way;
    logic [INDEX_BITS-1:0] fence_idx;
    logic [1:0]            fence_way;

    // PLRU victim selection
    function automatic logic [1:0] get_victim(input logic [2:0] p);
        if (!p[2]) return p[1] ? 2'd1 : 2'd0;
        else       return p[0] ? 2'd3 : 2'd2;
    endfunction

    // Refill address decode
    wire [TAG_BITS-1:0]   refill_tag   = req_addr_r[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] refill_index = req_addr_r[OFFSET_BITS +: INDEX_BITS];
    wire [WORD_BITS-1:0]  refill_word  = req_addr_r[OFFSET_BITS-1:2];

    // Victim info
    wire [1:0]           victim_mesi = mesi[refill_index][victim_way];
    wire [TAG_BITS-1:0]  victim_tag  = tags[refill_index][victim_way];
    wire [LINE_BITS-1:0] victim_data = read_data_line(refill_index, victim_way);
    wire                 victim_dirty = (victim_mesi == MESI_M);

    // State machine and control registers - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
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
            if (mesi[snoop_index][w] != MESI_I && tags[snoop_index][w] == snoop_tag) begin
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
                if (cache_hit) begin
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

    // MESI state and PLRU update - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int s = 0; s < NUM_SETS; s++) begin
                for (int w = 0; w < NUM_WAYS; w++)
                    mesi[s][w] <= MESI_I;
                plru[s] <= '0;
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
            
            // Write hit on M/E - update MESI to M
            if (state == LOOKUP && cache_hit && cpu_req_write && (hit_mesi == MESI_M || hit_mesi == MESI_E)) begin
                mesi[index][hit_way] <= MESI_M;
            end
            
            // Upgrade complete (S->M)
            if (state == UPGR_WAIT && bus_resp_valid) begin
                mesi[refill_index][hit_way] <= MESI_M;
            end
            
            // Refill complete
            if (state == FILL_DONE) begin
                mesi[refill_index][victim_way] <= req_write_r ? MESI_M : (bus_resp_shared ? MESI_S : MESI_E);
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
            
            // FENCE writeback - invalidate all dirty lines
            if (state == FENCE_WB && mesi[fence_idx][fence_way] == MESI_M) begin
                mesi[fence_idx][fence_way] <= MESI_I;
            end
        end
    end
    
    // Tag RAM update - NO RESET for BRAM inference
    always_ff @(posedge clk) begin
        if (state == FILL_DONE) begin
            tags[refill_index][victim_way] <= refill_tag;
        end
    end
    
    // Data RAM update - NO RESET, banked writes for BRAM inference
    // Separate always blocks for each bank to help BRAM inference
    always_ff @(posedge clk) begin
        // Write hit on M/E
        if (state == LOOKUP && cache_hit && cpu_req_write && (hit_mesi == MESI_M || hit_mesi == MESI_E)) begin
            // Byte-enable write to specific word
            if (word == 0) begin
                if (cpu_req_wmask[0]) data_bank0[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank0[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank0[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank0[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 1) begin
                if (cpu_req_wmask[0]) data_bank0[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank0[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank0[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank0[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 2) begin
                if (cpu_req_wmask[0]) data_bank1[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank1[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank1[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank1[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 3) begin
                if (cpu_req_wmask[0]) data_bank1[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank1[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank1[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank1[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 4) begin
                if (cpu_req_wmask[0]) data_bank2[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank2[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank2[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank2[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 5) begin
                if (cpu_req_wmask[0]) data_bank2[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank2[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank2[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank2[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 6) begin
                if (cpu_req_wmask[0]) data_bank3[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank3[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank3[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank3[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 7) begin
                if (cpu_req_wmask[0]) data_bank3[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank3[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank3[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank3[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 8) begin
                if (cpu_req_wmask[0]) data_bank4[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank4[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank4[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank4[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 9) begin
                if (cpu_req_wmask[0]) data_bank4[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank4[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank4[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank4[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 10) begin
                if (cpu_req_wmask[0]) data_bank5[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank5[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank5[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank5[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 11) begin
                if (cpu_req_wmask[0]) data_bank5[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank5[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank5[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank5[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 12) begin
                if (cpu_req_wmask[0]) data_bank6[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank6[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank6[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank6[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 13) begin
                if (cpu_req_wmask[0]) data_bank6[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank6[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank6[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank6[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end else if (word == 14) begin
                if (cpu_req_wmask[0]) data_bank7[index][hit_way][0 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank7[index][hit_way][8 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank7[index][hit_way][16 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank7[index][hit_way][24 +: 8] <= cpu_req_wdata[24 +: 8];
            end else begin // word == 15
                if (cpu_req_wmask[0]) data_bank7[index][hit_way][32 +: 8] <= cpu_req_wdata[0 +: 8];
                if (cpu_req_wmask[1]) data_bank7[index][hit_way][40 +: 8] <= cpu_req_wdata[8 +: 8];
                if (cpu_req_wmask[2]) data_bank7[index][hit_way][48 +: 8] <= cpu_req_wdata[16 +: 8];
                if (cpu_req_wmask[3]) data_bank7[index][hit_way][56 +: 8] <= cpu_req_wdata[24 +: 8];
            end
        end
        
        // Upgrade complete - write the pending data
        if (state == UPGR_WAIT && bus_resp_valid) begin
            // Similar word-based write for upgrade
            if (refill_word < 2) begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank0[refill_index][hit_way][(refill_word[0]*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end else if (refill_word < 4) begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank1[refill_index][hit_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end else if (refill_word < 6) begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank2[refill_index][hit_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end else if (refill_word < 8) begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank3[refill_index][hit_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end else if (refill_word < 10) begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank4[refill_index][hit_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end else if (refill_word < 12) begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank5[refill_index][hit_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end else if (refill_word < 14) begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank6[refill_index][hit_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end else begin
                for (int b = 0; b < 4; b++)
                    if (req_wmask_r[b]) data_bank7[refill_index][hit_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
            end
        end
        
        // Refill complete - write entire line
        if (state == FILL_DONE) begin
            if (req_write_r) begin
                // Merge write data with fetched line
                data_bank0[refill_index][victim_way] <= bus_resp_data[0*BANK_WIDTH +: BANK_WIDTH];
                data_bank1[refill_index][victim_way] <= bus_resp_data[1*BANK_WIDTH +: BANK_WIDTH];
                data_bank2[refill_index][victim_way] <= bus_resp_data[2*BANK_WIDTH +: BANK_WIDTH];
                data_bank3[refill_index][victim_way] <= bus_resp_data[3*BANK_WIDTH +: BANK_WIDTH];
                data_bank4[refill_index][victim_way] <= bus_resp_data[4*BANK_WIDTH +: BANK_WIDTH];
                data_bank5[refill_index][victim_way] <= bus_resp_data[5*BANK_WIDTH +: BANK_WIDTH];
                data_bank6[refill_index][victim_way] <= bus_resp_data[6*BANK_WIDTH +: BANK_WIDTH];
                data_bank7[refill_index][victim_way] <= bus_resp_data[7*BANK_WIDTH +: BANK_WIDTH];
                // Apply byte masks to the correct word
                if (refill_word < 2) begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank0[refill_index][victim_way][(refill_word[0]*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end else if (refill_word < 4) begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank1[refill_index][victim_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end else if (refill_word < 6) begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank2[refill_index][victim_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end else if (refill_word < 8) begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank3[refill_index][victim_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end else if (refill_word < 10) begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank4[refill_index][victim_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end else if (refill_word < 12) begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank5[refill_index][victim_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end else if (refill_word < 14) begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank6[refill_index][victim_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end else begin
                    for (int b = 0; b < 4; b++)
                        if (req_wmask_r[b]) data_bank7[refill_index][victim_way][((refill_word[0])*32) + b*8 +: 8] <= req_wdata_r[b*8 +: 8];
                end
            end else begin
                data_bank0[refill_index][victim_way] <= bus_resp_data[0*BANK_WIDTH +: BANK_WIDTH];
                data_bank1[refill_index][victim_way] <= bus_resp_data[1*BANK_WIDTH +: BANK_WIDTH];
                data_bank2[refill_index][victim_way] <= bus_resp_data[2*BANK_WIDTH +: BANK_WIDTH];
                data_bank3[refill_index][victim_way] <= bus_resp_data[3*BANK_WIDTH +: BANK_WIDTH];
                data_bank4[refill_index][victim_way] <= bus_resp_data[4*BANK_WIDTH +: BANK_WIDTH];
                data_bank5[refill_index][victim_way] <= bus_resp_data[5*BANK_WIDTH +: BANK_WIDTH];
                data_bank6[refill_index][victim_way] <= bus_resp_data[6*BANK_WIDTH +: BANK_WIDTH];
                data_bank7[refill_index][victim_way] <= bus_resp_data[7*BANK_WIDTH +: BANK_WIDTH];
            end
        end
    end

    // AMO execution
    logic [DATA_WIDTH-1:0] amo_result;
    logic [DATA_WIDTH-1:0] amo_old_val;
    wire [LINE_BITS-1:0] amo_line = read_data_line(index, hit_way);
    
    always_comb begin
        amo_old_val = amo_line[word*32 +: 32];
        amo_result = amo_old_val;
        case (amo_op)
            5'b00001: amo_result = amo_wdata;
            5'b00000: amo_result = amo_old_val + amo_wdata;
            5'b00100: amo_result = amo_old_val ^ amo_wdata;
            5'b01100: amo_result = amo_old_val & amo_wdata;
            5'b01000: amo_result = amo_old_val | amo_wdata;
            5'b10000: amo_result = ($signed(amo_old_val) < $signed(amo_wdata)) ? amo_old_val : amo_wdata;
            5'b10100: amo_result = ($signed(amo_old_val) > $signed(amo_wdata)) ? amo_old_val : amo_wdata;
            5'b11000: amo_result = (amo_old_val < amo_wdata) ? amo_old_val : amo_wdata;
            5'b11100: amo_result = (amo_old_val > amo_wdata) ? amo_old_val : amo_wdata;
            default:  amo_result = amo_old_val;
        endcase
    end

    // Bus interface
    assign bus_req_valid = (state == FILL_REQ) || (state == WB_REQ) || (state == UPGR_REQ);
    assign bus_req_cmd   = (state == WB_REQ) ? BUS_FLUSH :
                           (state == UPGR_REQ) ? BUS_UPGR :
                           req_write_r ? BUS_RDX : BUS_RD;
    assign bus_req_addr  = (state == WB_REQ) ? {victim_tag, refill_index, {OFFSET_BITS{1'b0}}} :
                           {refill_tag, refill_index, {OFFSET_BITS{1'b0}}};
    assign bus_req_data  = victim_data;
    assign bus_req_ready = (state == FILL_WAIT) || (state == WB_WAIT) || (state == UPGR_WAIT);

    // Snoop response
    assign snoop_resp_hit  = |snoop_way_hit;
    assign snoop_resp_hitm = |snoop_way_hit && (snoop_hit_mesi == MESI_M);
    assign snoop_resp_data = read_data_line(snoop_index, snoop_hit_way);
    assign snoop_resp_ack  = (state == SNOOP_CHECK) || (state == SNOOP_WB);

    // CPU response
    wire [LINE_BITS-1:0] hit_line = read_data_line(index, hit_way);
    assign cpu_resp_rdata = (state == LOOKUP && cache_hit) ? hit_line[word*32 +: 32] :
                            (state == FILL_DONE) ? bus_resp_data[refill_word*32 +: 32] : '0;
    assign cpu_resp_valid = (state == LOOKUP && cache_hit && (!cpu_req_write || write_hit)) ||
                            (state == FILL_DONE) ||
                            (state == UPGR_WAIT && bus_resp_valid);
    assign cpu_stall      = cpu_req_valid && !cpu_resp_valid;
    
    // AMO response
    assign amo_rdata = amo_old_val;
    assign amo_done  = (state == AMO_EXEC);
    
    // FENCE
    assign fence_complete = (state == FENCE_WB && fence_idx == NUM_SETS-1 && fence_way == NUM_WAYS-1);
    assign cache_ready    = (state == IDLE);

endmodule

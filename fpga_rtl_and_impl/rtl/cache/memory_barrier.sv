//============================================================================
// Memory Barrier Unit
// Implements RISC-V FENCE instructions for memory ordering
//============================================================================

`timescale 1ns / 1ps

module memory_barrier #(
    parameter NUM_CORES = 8
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // FENCE request from core
    input  logic                    fence_req,
    input  logic [3:0]              fence_pred,     // Predecessor set (I/O/R/W)
    input  logic [3:0]              fence_succ,     // Successor set (I/O/R/W)
    input  logic                    fence_tso,      // TSO fence (lighter weight)
    output logic                    fence_complete,
    
    // FENCE.I request (instruction fence)
    input  logic                    fence_i_req,
    output logic                    fence_i_complete,
    
    // Store buffer interface
    input  logic                    store_buffer_empty,
    output logic                    store_buffer_flush,
    
    // Load queue interface
    input  logic                    load_queue_empty,
    output logic                    load_queue_drain,
    
    // I-Cache interface
    output logic                    icache_invalidate,
    input  logic                    icache_inv_complete,
    
    // D-Cache interface
    output logic                    dcache_writeback,
    input  logic                    dcache_wb_complete,
    
    // Pipeline interface
    output logic                    pipeline_stall,
    
    // Core ID
    input  logic [2:0]              core_id
);

    // FENCE field bits
    localparam FENCE_I = 3;  // Device input
    localparam FENCE_O = 2;  // Device output
    localparam FENCE_R = 1;  // Memory reads
    localparam FENCE_W = 0;  // Memory writes

    // State machine
    typedef enum logic [3:0] {
        IDLE,
        FENCE_DRAIN_STORES,
        FENCE_DRAIN_LOADS,
        FENCE_WAIT_STORE_BUFFER,
        FENCE_WAIT_LOADS,
        FENCE_WRITEBACK,
        FENCE_WAIT_WB,
        FENCE_I_START,
        FENCE_I_INVALIDATE,
        FENCE_I_WAIT,
        COMPLETE
    } state_t;
    
    state_t state, next_state;

    // Registered fence parameters
    logic [3:0] pred_r, succ_r;
    logic       is_fence_i_r;
    logic       is_tso_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            pred_r <= '0;
            succ_r <= '0;
            is_fence_i_r <= 1'b0;
            is_tso_r <= 1'b0;
        end else begin
            state <= next_state;
            
            if (state == IDLE) begin
                if (fence_i_req) begin
                    is_fence_i_r <= 1'b1;
                    is_tso_r <= 1'b0;
                end else if (fence_req) begin
                    pred_r <= fence_pred;
                    succ_r <= fence_succ;
                    is_fence_i_r <= 1'b0;
                    is_tso_r <= fence_tso;
                end
            end
        end
    end

    // Determine what actions are needed based on fence type
    wire need_store_drain = pred_r[FENCE_W] && (succ_r[FENCE_R] || succ_r[FENCE_W]);
    wire need_load_drain  = pred_r[FENCE_R] && succ_r[FENCE_W];
    wire need_writeback   = pred_r[FENCE_W] && succ_r[FENCE_O];

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (fence_i_req)
                    next_state = FENCE_I_START;
                else if (fence_req) begin
                    if (fence_tso)
                        next_state = FENCE_DRAIN_STORES;
                    else if (need_store_drain)
                        next_state = FENCE_DRAIN_STORES;
                    else if (need_load_drain)
                        next_state = FENCE_DRAIN_LOADS;
                    else
                        next_state = COMPLETE;
                end
            end
            
            FENCE_DRAIN_STORES: next_state = FENCE_WAIT_STORE_BUFFER;
            FENCE_WAIT_STORE_BUFFER: begin
                if (store_buffer_empty) begin
                    if (need_load_drain && !is_tso_r)
                        next_state = FENCE_DRAIN_LOADS;
                    else if (need_writeback)
                        next_state = FENCE_WRITEBACK;
                    else
                        next_state = COMPLETE;
                end
            end
            
            FENCE_DRAIN_LOADS: next_state = FENCE_WAIT_LOADS;
            FENCE_WAIT_LOADS: begin
                if (load_queue_empty) begin
                    if (need_writeback)
                        next_state = FENCE_WRITEBACK;
                    else
                        next_state = COMPLETE;
                end
            end
            
            FENCE_WRITEBACK: next_state = FENCE_WAIT_WB;
            FENCE_WAIT_WB: begin
                if (dcache_wb_complete)
                    next_state = COMPLETE;
            end
            
            FENCE_I_START: next_state = FENCE_DRAIN_STORES;
            
            // For FENCE.I, after draining stores, invalidate I$
            FENCE_I_INVALIDATE: next_state = FENCE_I_WAIT;
            FENCE_I_WAIT: begin
                if (icache_inv_complete)
                    next_state = COMPLETE;
            end
            
            COMPLETE: next_state = IDLE;
            
            default: next_state = IDLE;
        endcase
        
        // Special handling: after store drain in FENCE.I path
        if (state == FENCE_WAIT_STORE_BUFFER && store_buffer_empty && is_fence_i_r)
            next_state = FENCE_I_INVALIDATE;
    end

    // Output control signals
    assign store_buffer_flush = (state == FENCE_DRAIN_STORES);
    assign load_queue_drain   = (state == FENCE_DRAIN_LOADS);
    assign icache_invalidate  = (state == FENCE_I_INVALIDATE);
    assign dcache_writeback   = (state == FENCE_WRITEBACK);
    assign pipeline_stall     = (state != IDLE) && (state != COMPLETE);
    assign fence_complete     = (state == COMPLETE) && !is_fence_i_r;
    assign fence_i_complete   = (state == COMPLETE) && is_fence_i_r;

endmodule









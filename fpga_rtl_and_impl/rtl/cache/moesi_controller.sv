//============================================================================
// MOESI Protocol Controller
// Manages 5-state cache coherence for multicore system
//
// MOESI States:
//   M - Modified:  Line is dirty, exclusive to this cache
//   O - Owned:     Line may be shared but this cache is responsible for writeback
//   E - Exclusive: Line is clean, exclusive to this cache
//   S - Shared:    Line is clean, may exist in other caches
//   I - Invalid:   Line is not valid
//
// Key difference from MESI:
//   Owned state allows sharing modified data without writeback to memory
//   Reduces memory bandwidth for read-sharing patterns
//============================================================================

`timescale 1ns / 1ps

module moesi_controller #(
    parameter NUM_CORES    = 8,
    parameter ADDR_WIDTH   = 32,
    parameter LINE_BITS    = 512
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Core Request Interface (from each core's L1 D$)
    input  logic [NUM_CORES-1:0]    core_req_valid,
    input  logic [NUM_CORES-1:0][2:0] core_req_cmd,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0] core_req_addr,
    input  logic [NUM_CORES-1:0][LINE_BITS-1:0]  core_req_data,
    output logic [NUM_CORES-1:0]    core_req_grant,
    output logic [NUM_CORES-1:0][LINE_BITS-1:0]  core_resp_data,
    output logic [NUM_CORES-1:0]    core_resp_valid,
    output logic [NUM_CORES-1:0][2:0] core_resp_state,  // New MOESI state
    
    // Snoop Interface (to each core's L1 D$)
    output logic [NUM_CORES-1:0]    snoop_req_valid,
    output logic [2:0]              snoop_req_cmd,
    output logic [ADDR_WIDTH-1:0]   snoop_req_addr,
    input  logic [NUM_CORES-1:0]    snoop_resp_hit,
    input  logic [NUM_CORES-1:0]    snoop_resp_hitm,  // Hit in M or O state
    input  logic [NUM_CORES-1:0]    snoop_resp_hito,  // Hit in O state specifically
    input  logic [NUM_CORES-1:0][LINE_BITS-1:0] snoop_resp_data,
    input  logic [NUM_CORES-1:0][2:0] snoop_resp_state,
    input  logic [NUM_CORES-1:0]    snoop_resp_ack,
    
    // L2 Cache Interface
    output logic                    l2_req_valid,
    output logic                    l2_req_write,
    output logic [ADDR_WIDTH-1:0]   l2_req_addr,
    output logic [LINE_BITS-1:0]    l2_req_data,
    input  logic [LINE_BITS-1:0]    l2_resp_data,
    input  logic                    l2_resp_valid,
    input  logic                    l2_resp_shared,
    output logic                    l2_req_ready
);

    // MOESI States
    localparam [2:0] MOESI_I = 3'b000;  // Invalid
    localparam [2:0] MOESI_S = 3'b001;  // Shared
    localparam [2:0] MOESI_E = 3'b010;  // Exclusive
    localparam [2:0] MOESI_O = 3'b011;  // Owned (new in MOESI)
    localparam [2:0] MOESI_M = 3'b100;  // Modified

    // Bus Commands
    localparam [2:0] BUS_RD    = 3'b001;  // Read (for shared)
    localparam [2:0] BUS_RDX   = 3'b010;  // Read exclusive (for write)
    localparam [2:0] BUS_UPGR  = 3'b011;  // Upgrade (S->M)
    localparam [2:0] BUS_FLUSH = 3'b100;  // Flush/writeback
    localparam [2:0] BUS_FLUSHOPT = 3'b101; // Flush optimized (O->S transition)

    // State machine
    typedef enum logic [3:0] {
        IDLE,
        ARBITRATE,
        SNOOP_SEND,
        SNOOP_WAIT,
        SNOOP_COLLECT,
        CACHE_TO_CACHE,  // Direct cache-to-cache transfer
        L2_REQ,
        L2_WAIT,
        RESPOND,
        WRITEBACK,
        UPDATE_OWNER     // Update ownership for O state
    } state_t;
    
    state_t state, next_state;

    // Arbiter - round robin
    logic [$clog2(NUM_CORES)-1:0] current_core;
    logic [$clog2(NUM_CORES)-1:0] last_served;
    logic [NUM_CORES-1:0]         pending_req;
    
    // Registered request
    logic [2:0]              req_cmd_r;
    logic [ADDR_WIDTH-1:0]   req_addr_r;
    logic [LINE_BITS-1:0]    req_data_r;
    logic [$clog2(NUM_CORES)-1:0] req_core_r;
    
    // Snoop results
    logic                    any_hit;
    logic                    any_hitm;    // Hit in M state
    logic                    any_hito;    // Hit in O state (new)
    logic [LINE_BITS-1:0]    owner_data;  // Data from M or O state holder
    logic [$clog2(NUM_CORES)-1:0] owner_core;
    logic [NUM_CORES-1:0]    snoop_ack_received;
    logic [2:0]              new_state;
    
    // L2 response
    logic [LINE_BITS-1:0]    l2_data_r;
    logic                    l2_shared_r;

    // Round-robin arbiter
    always_comb begin
        current_core = last_served;
        pending_req = core_req_valid;
        
        for (int i = 0; i < NUM_CORES; i++) begin
            logic [$clog2(NUM_CORES)-1:0] check_core;
            check_core = (last_served + 1 + i) % NUM_CORES;
            if (pending_req[check_core]) begin
                current_core = check_core;
                break;
            end
        end
    end

    // SYNCHRONOUS reset for FPGA optimization
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            last_served <= '0;
            req_cmd_r <= '0;
            req_addr_r <= '0;
            req_data_r <= '0;
            req_core_r <= '0;
            snoop_ack_received <= '0;
            owner_data <= '0;
            owner_core <= '0;
            l2_data_r <= '0;
            l2_shared_r <= 1'b0;
            any_hit <= 1'b0;
            any_hitm <= 1'b0;
            any_hito <= 1'b0;
        end else begin
            state <= next_state;
            
            case (state)
                ARBITRATE: begin
                    req_cmd_r <= core_req_cmd[current_core];
                    req_addr_r <= core_req_addr[current_core];
                    req_data_r <= core_req_data[current_core];
                    req_core_r <= current_core;
                    snoop_ack_received <= '0;
                    any_hit <= 1'b0;
                    any_hitm <= 1'b0;
                    any_hito <= 1'b0;
                end
                
                SNOOP_WAIT: begin
                    for (int i = 0; i < NUM_CORES; i++) begin
                        if (snoop_resp_ack[i]) begin
                            snoop_ack_received[i] <= 1'b1;
                            if (snoop_resp_hit[i])
                                any_hit <= 1'b1;
                            if (snoop_resp_hitm[i]) begin
                                any_hitm <= 1'b1;
                                owner_data <= snoop_resp_data[i];
                                owner_core <= i[$clog2(NUM_CORES)-1:0];
                            end
                            if (snoop_resp_hito[i]) begin
                                any_hito <= 1'b1;
                                if (!any_hitm) begin  // O has lower priority than M
                                    owner_data <= snoop_resp_data[i];
                                    owner_core <= i[$clog2(NUM_CORES)-1:0];
                                end
                            end
                        end
                    end
                end
                
                L2_WAIT: begin
                    if (l2_resp_valid) begin
                        l2_data_r <= l2_resp_data;
                        l2_shared_r <= l2_resp_shared || any_hit;
                    end
                end
                
                RESPOND: begin
                    last_served <= req_core_r;
                end
                default: ; // Other states: registers hold values
            endcase
        end
    end

    // Check if all snoops received (excluding requesting core)
    logic all_snoops_done;
    always_comb begin
        all_snoops_done = 1'b1;
        for (int i = 0; i < NUM_CORES; i++) begin
            if (i != req_core_r && !snoop_ack_received[i])
                all_snoops_done = 1'b0;
        end
    end

    // Determine new MOESI state for requesting cache
    always_comb begin
        new_state = MOESI_I;
        case (req_cmd_r)
            BUS_RD: begin
                // Read request
                if (any_hitm || any_hito) begin
                    // Data from another cache - Shared state
                    new_state = MOESI_S;
                end else if (l2_shared_r) begin
                    // L2 reports shared
                    new_state = MOESI_S;
                end else begin
                    // Exclusive access
                    new_state = MOESI_E;
                end
            end
            BUS_RDX: begin
                // Read exclusive - always Modified
                new_state = MOESI_M;
            end
            BUS_UPGR: begin
                // Upgrade from S to M
                new_state = MOESI_M;
            end
            default: new_state = MOESI_I;
        endcase
    end

    // Next state logic
    always_comb begin
        next_state = state;
        case (state)
            IDLE:         next_state = |core_req_valid ? ARBITRATE : IDLE;
            ARBITRATE:    next_state = SNOOP_SEND;
            SNOOP_SEND:   next_state = SNOOP_WAIT;
            SNOOP_WAIT:   next_state = all_snoops_done ? SNOOP_COLLECT : SNOOP_WAIT;
            SNOOP_COLLECT: begin
                if (req_cmd_r == BUS_FLUSH)
                    next_state = WRITEBACK;
                else if (any_hitm || any_hito)
                    next_state = CACHE_TO_CACHE;  // Get data from owner cache
                else
                    next_state = L2_REQ;
            end
            CACHE_TO_CACHE: next_state = UPDATE_OWNER;  // Update owner's state
            UPDATE_OWNER:   next_state = RESPOND;
            L2_REQ:       next_state = L2_WAIT;
            L2_WAIT:      next_state = l2_resp_valid ? RESPOND : L2_WAIT;
            RESPOND:      next_state = IDLE;
            WRITEBACK:    next_state = L2_WAIT;
            default:      next_state = IDLE;
        endcase
    end

    // Snoop broadcast
    always_comb begin
        snoop_req_valid = '0;
        if (state == SNOOP_SEND || state == SNOOP_WAIT) begin
            for (int i = 0; i < NUM_CORES; i++) begin
                if (i != req_core_r)
                    snoop_req_valid[i] = 1'b1;
            end
        end
    end
    assign snoop_req_cmd  = req_cmd_r;
    assign snoop_req_addr = req_addr_r;

    // Core request grant
    always_comb begin
        core_req_grant = '0;
        if (state == ARBITRATE)
            core_req_grant[current_core] = 1'b1;
    end

    // Core response
    always_comb begin
        core_resp_valid = '0;
        core_resp_data = '{default: '0};
        core_resp_state = '{default: MOESI_I};
        
        if (state == RESPOND) begin
            core_resp_valid[req_core_r] = 1'b1;
            core_resp_state[req_core_r] = new_state;
            
            // Data source selection
            if (any_hitm || any_hito) begin
                // Cache-to-cache transfer (data from M or O state holder)
                core_resp_data[req_core_r] = owner_data;
            end else begin
                // Data from L2
                core_resp_data[req_core_r] = l2_data_r;
            end
        end
    end

    // L2 request
    assign l2_req_valid = (state == L2_REQ) || (state == WRITEBACK);
    assign l2_req_write = (state == WRITEBACK) || (req_cmd_r == BUS_FLUSH);
    assign l2_req_addr  = req_addr_r;
    assign l2_req_data  = (any_hitm || any_hito) ? owner_data : req_data_r;
    assign l2_req_ready = (state == IDLE);

    //=========================================================================
    // MOESI State Transition Rules (for snooping caches)
    //=========================================================================
    // When a cache receives a snoop request, it transitions according to:
    //
    // On BUS_RD (read by another cache):
    //   M -> O  (share modified data, become owner)
    //   O -> O  (stay owner, share data)
    //   E -> S  (no longer exclusive)
    //   S -> S  (stay shared)
    //
    // On BUS_RDX (read exclusive by another cache):
    //   M -> I  (writeback data, invalidate)
    //   O -> I  (supply data, invalidate)
    //   E -> I  (invalidate)
    //   S -> I  (invalidate)
    //
    // On BUS_UPGR (upgrade by another cache):
    //   S -> I  (invalidate - other cache going to M)
    //=========================================================================

endmodule



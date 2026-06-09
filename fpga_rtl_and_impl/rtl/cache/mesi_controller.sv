//============================================================================
// MESI Protocol Controller
// Manages cache coherence state transitions for multicore system
//============================================================================

`timescale 1ns / 1ps

module mesi_controller #(
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
    output logic [NUM_CORES-1:0]    core_resp_shared,
    
    // Snoop Interface (to each core's L1 D$)
    output logic [NUM_CORES-1:0]    snoop_req_valid,
    output logic [2:0]              snoop_req_cmd,
    output logic [ADDR_WIDTH-1:0]   snoop_req_addr,
    input  logic [NUM_CORES-1:0]    snoop_resp_hit,
    input  logic [NUM_CORES-1:0]    snoop_resp_hitm,
    input  logic [NUM_CORES-1:0][LINE_BITS-1:0] snoop_resp_data,
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

    // Bus Commands
    localparam [2:0] BUS_RD    = 3'b001;
    localparam [2:0] BUS_RDX   = 3'b010;
    localparam [2:0] BUS_UPGR  = 3'b011;
    localparam [2:0] BUS_FLUSH = 3'b100;

    // State machine
    typedef enum logic [3:0] {
        IDLE,
        ARBITRATE,
        SNOOP_SEND,
        SNOOP_WAIT,
        SNOOP_COLLECT,
        L2_REQ,
        L2_WAIT,
        RESPOND,
        WRITEBACK
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
    logic                    any_hitm;
    logic [LINE_BITS-1:0]    hitm_data;
    logic [$clog2(NUM_CORES)-1:0] hitm_core;
    logic [NUM_CORES-1:0]    snoop_ack_received;
    
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
            any_hit <= 1'b0;
            any_hitm <= 1'b0;
            hitm_data <= '0;
            hitm_core <= '0;
            l2_data_r <= '0;
            l2_shared_r <= 1'b0;
        end else begin
            state <= next_state;
            
            case (state)
                IDLE: begin
                    if (|core_req_valid) begin
                        req_core_r <= current_core;
                        req_cmd_r  <= core_req_cmd[current_core];
                        req_addr_r <= core_req_addr[current_core];
                        req_data_r <= core_req_data[current_core];
                        snoop_ack_received <= '0;
                        any_hit <= 1'b0;
                        any_hitm <= 1'b0;
                    end
                end
                
                SNOOP_SEND: begin
                    snoop_ack_received <= '0;
                end
                
                SNOOP_WAIT: begin
                    // Collect snoop responses
                    for (int i = 0; i < NUM_CORES; i++) begin
                        if (snoop_resp_ack[i] && i != req_core_r) begin
                            snoop_ack_received[i] <= 1'b1;
                            if (snoop_resp_hit[i])
                                any_hit <= 1'b1;
                            if (snoop_resp_hitm[i]) begin
                                any_hitm <= 1'b1;
                                hitm_data <= snoop_resp_data[i];
                                hitm_core <= i[$clog2(NUM_CORES)-1:0];
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
                else if (any_hitm)
                    next_state = RESPOND;  // Got data from another cache
                else
                    next_state = L2_REQ;
            end
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

    // L2 interface
    assign l2_req_valid = (state == L2_REQ) || (state == WRITEBACK);
    assign l2_req_write = (state == WRITEBACK);
    assign l2_req_addr  = req_addr_r;
    assign l2_req_data  = (state == WRITEBACK) ? req_data_r : '0;
    assign l2_req_ready = (state == L2_WAIT);

    // Core responses
    always_comb begin
        core_req_grant = '0;
        core_resp_valid = '0;
        core_resp_data = '0;
        core_resp_shared = '0;
        
        if (state == RESPOND) begin
            core_req_grant[req_core_r] = 1'b1;
            core_resp_valid[req_core_r] = 1'b1;
            core_resp_data[req_core_r] = any_hitm ? hitm_data : l2_data_r;
            core_resp_shared[req_core_r] = any_hit || l2_shared_r;
        end
    end

endmodule









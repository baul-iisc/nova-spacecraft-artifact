//============================================================================
// Atomic Memory Operations (AMO) Unit
// Supports RISC-V A extension atomic operations
//============================================================================

`timescale 1ns / 1ps

module amo_unit #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // CPU Interface
    input  logic                    amo_req_valid,
    input  logic [4:0]              amo_op,         // AMO operation code
    input  logic [ADDR_WIDTH-1:0]   amo_addr,
    input  logic [DATA_WIDTH-1:0]   amo_wdata,
    input  logic                    amo_aq,         // Acquire semantics
    input  logic                    amo_rl,         // Release semantics
    output logic [DATA_WIDTH-1:0]   amo_rdata,
    output logic                    amo_done,
    output logic                    amo_error,
    
    // Load Reserved / Store Conditional
    input  logic                    lr_req_valid,
    input  logic [ADDR_WIDTH-1:0]   lr_addr,
    output logic [DATA_WIDTH-1:0]   lr_rdata,
    output logic                    lr_done,
    
    input  logic                    sc_req_valid,
    input  logic [ADDR_WIDTH-1:0]   sc_addr,
    input  logic [DATA_WIDTH-1:0]   sc_wdata,
    output logic                    sc_success,
    output logic                    sc_done,
    
    // Memory Interface
    output logic                    mem_req_valid,
    output logic                    mem_req_write,
    output logic [ADDR_WIDTH-1:0]   mem_req_addr,
    output logic [DATA_WIDTH-1:0]   mem_req_wdata,
    input  logic [DATA_WIDTH-1:0]   mem_resp_rdata,
    input  logic                    mem_resp_valid,
    
    // Snoop Interface for reservation invalidation
    input  logic                    snoop_valid,
    input  logic [ADDR_WIDTH-1:0]   snoop_addr,
    input  logic                    snoop_invalidate,
    
    // Core ID for reservation tracking
    input  logic [2:0]              core_id
);

    // AMO operation codes (RISC-V A extension)
    localparam [4:0] AMO_LR    = 5'b00010;  // Load Reserved
    localparam [4:0] AMO_SC    = 5'b00011;  // Store Conditional
    localparam [4:0] AMO_SWAP  = 5'b00001;
    localparam [4:0] AMO_ADD   = 5'b00000;
    localparam [4:0] AMO_XOR   = 5'b00100;
    localparam [4:0] AMO_AND   = 5'b01100;
    localparam [4:0] AMO_OR    = 5'b01000;
    localparam [4:0] AMO_MIN   = 5'b10000;
    localparam [4:0] AMO_MAX   = 5'b10100;
    localparam [4:0] AMO_MINU  = 5'b11000;
    localparam [4:0] AMO_MAXU  = 5'b11100;

    // State machine
    typedef enum logic [2:0] {
        IDLE,
        READ_REQ,
        READ_WAIT,
        COMPUTE,
        WRITE_REQ,
        WRITE_WAIT,
        COMPLETE
    } state_t;
    
    state_t state, next_state;

    // Reservation register for LR/SC
    logic                    reservation_valid;
    logic [ADDR_WIDTH-1:0]   reservation_addr;
    logic [2:0]              reservation_core;
    
    // Registered values
    logic [4:0]              op_r;
    logic [ADDR_WIDTH-1:0]   addr_r;
    logic [DATA_WIDTH-1:0]   wdata_r;
    logic [DATA_WIDTH-1:0]   rdata_r;
    logic [DATA_WIDTH-1:0]   result_r;
    logic                    is_lr_r, is_sc_r, is_amo_r;

    // AMO computation
    logic [DATA_WIDTH-1:0] amo_result;
    
    always_comb begin
        amo_result = rdata_r;
        case (op_r)
            AMO_SWAP: amo_result = wdata_r;
            AMO_ADD:  amo_result = rdata_r + wdata_r;
            AMO_XOR:  amo_result = rdata_r ^ wdata_r;
            AMO_AND:  amo_result = rdata_r & wdata_r;
            AMO_OR:   amo_result = rdata_r | wdata_r;
            AMO_MIN:  amo_result = ($signed(rdata_r) < $signed(wdata_r)) ? rdata_r : wdata_r;
            AMO_MAX:  amo_result = ($signed(rdata_r) > $signed(wdata_r)) ? rdata_r : wdata_r;
            AMO_MINU: amo_result = (rdata_r < wdata_r) ? rdata_r : wdata_r;
            AMO_MAXU: amo_result = (rdata_r > wdata_r) ? rdata_r : wdata_r;
            default:  amo_result = rdata_r;
        endcase
    end

    // SC success check
    wire sc_can_succeed = reservation_valid && 
                          (reservation_addr == addr_r) && 
                          (reservation_core == core_id);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            reservation_valid <= 1'b0;
            reservation_addr <= '0;
            reservation_core <= '0;
            op_r <= '0;
            addr_r <= '0;
            wdata_r <= '0;
            rdata_r <= '0;
            result_r <= '0;
            is_lr_r <= 1'b0;
            is_sc_r <= 1'b0;
            is_amo_r <= 1'b0;
        end else begin
            state <= next_state;
            
            // Snoop invalidation of reservation
            if (snoop_valid && snoop_invalidate && 
                reservation_valid && (snoop_addr[ADDR_WIDTH-1:2] == reservation_addr[ADDR_WIDTH-1:2])) begin
                reservation_valid <= 1'b0;
            end
            
            case (state)
                IDLE: begin
                    if (lr_req_valid) begin
                        is_lr_r <= 1'b1;
                        is_sc_r <= 1'b0;
                        is_amo_r <= 1'b0;
                        addr_r <= lr_addr;
                    end else if (sc_req_valid) begin
                        is_lr_r <= 1'b0;
                        is_sc_r <= 1'b1;
                        is_amo_r <= 1'b0;
                        addr_r <= sc_addr;
                        wdata_r <= sc_wdata;
                    end else if (amo_req_valid) begin
                        is_lr_r <= 1'b0;
                        is_sc_r <= 1'b0;
                        is_amo_r <= 1'b1;
                        op_r <= amo_op;
                        addr_r <= amo_addr;
                        wdata_r <= amo_wdata;
                    end
                end
                
                READ_WAIT: begin
                    if (mem_resp_valid) begin
                        rdata_r <= mem_resp_rdata;
                    end
                end
                
                COMPUTE: begin
                    result_r <= amo_result;
                    // Set reservation on LR
                    if (is_lr_r) begin
                        reservation_valid <= 1'b1;
                        reservation_addr <= addr_r;
                        reservation_core <= core_id;
                    end
                end
                
                WRITE_WAIT: begin
                    // Clear reservation on SC (success or fail)
                    if (is_sc_r) begin
                        reservation_valid <= 1'b0;
                    end
                end
            endcase
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (lr_req_valid || amo_req_valid)
                    next_state = READ_REQ;
                else if (sc_req_valid)
                    next_state = sc_can_succeed ? WRITE_REQ : COMPLETE;
            end
            READ_REQ:   next_state = READ_WAIT;
            READ_WAIT:  next_state = mem_resp_valid ? COMPUTE : READ_WAIT;
            COMPUTE: begin
                if (is_lr_r)
                    next_state = COMPLETE;
                else
                    next_state = WRITE_REQ;
            end
            WRITE_REQ:  next_state = WRITE_WAIT;
            WRITE_WAIT: next_state = mem_resp_valid ? COMPLETE : WRITE_WAIT;
            COMPLETE:   next_state = IDLE;
            default:    next_state = IDLE;
        endcase
    end

    // Memory interface
    assign mem_req_valid = (state == READ_REQ) || (state == WRITE_REQ);
    assign mem_req_write = (state == WRITE_REQ);
    assign mem_req_addr  = addr_r;
    assign mem_req_wdata = is_sc_r ? wdata_r : result_r;

    // Output signals
    assign amo_rdata = rdata_r;
    assign amo_done  = (state == COMPLETE) && is_amo_r;
    assign amo_error = 1'b0;  // No error handling in this simple implementation

    assign lr_rdata = rdata_r;
    assign lr_done  = (state == COMPLETE) && is_lr_r;

    assign sc_success = is_sc_r && sc_can_succeed;
    assign sc_done    = (state == COMPLETE) && is_sc_r;

endmodule









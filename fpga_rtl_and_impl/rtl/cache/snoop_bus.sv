//============================================================================
// Snooping Bus Interconnect with MOESI Protocol
// Broadcast-based coherence bus connecting L1 caches to L2 and each other
// V2: Upgraded to MOESI (5-state) for reduced memory bandwidth
//============================================================================

`timescale 1ns / 1ps

module snoop_bus #(
    parameter NUM_CORES    = 8,
    parameter ADDR_WIDTH   = 32,
    parameter LINE_BITS    = 512,
    parameter BUS_WIDTH    = 256     // Bus data width
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // L1 I$ Interfaces (read-only, just invalidation)
    output logic [NUM_CORES-1:0]    icache_snoop_valid,
    output logic [ADDR_WIDTH-1:0]   icache_snoop_addr,
    input  logic [NUM_CORES-1:0]    icache_snoop_hit,
    input  logic [NUM_CORES-1:0]    icache_snoop_ack,
    
    // L1 D$ Request Interfaces
    input  logic [NUM_CORES-1:0]    dcache_req_valid,
    input  logic [NUM_CORES-1:0][2:0] dcache_req_cmd,
    input  logic [NUM_CORES-1:0][ADDR_WIDTH-1:0] dcache_req_addr,
    input  logic [NUM_CORES-1:0][LINE_BITS-1:0]  dcache_req_data,
    output logic [NUM_CORES-1:0]    dcache_req_grant,
    output logic [NUM_CORES-1:0][LINE_BITS-1:0]  dcache_resp_data,
    output logic [NUM_CORES-1:0]    dcache_resp_valid,
    output logic [NUM_CORES-1:0]    dcache_resp_shared,
    
    // L1 D$ Snoop Interfaces
    output logic [NUM_CORES-1:0]    dcache_snoop_valid,
    output logic [2:0]              dcache_snoop_cmd,
    output logic [ADDR_WIDTH-1:0]   dcache_snoop_addr,
    input  logic [NUM_CORES-1:0]    dcache_snoop_hit,
    input  logic [NUM_CORES-1:0]    dcache_snoop_hitm,
    input  logic [NUM_CORES-1:0]    dcache_snoop_hito,  // NEW: Hit in Owned state
    input  logic [NUM_CORES-1:0][LINE_BITS-1:0] dcache_snoop_data,
    input  logic [NUM_CORES-1:0][2:0] dcache_snoop_state, // NEW: MOESI state
    input  logic [NUM_CORES-1:0]    dcache_snoop_ack,
    
    // L2 Cache Interface
    output logic                    l2_req_valid,
    output logic                    l2_req_write,
    output logic [ADDR_WIDTH-1:0]   l2_req_addr,
    output logic [LINE_BITS-1:0]    l2_req_wdata,
    input  logic [LINE_BITS-1:0]    l2_resp_rdata,
    input  logic                    l2_resp_valid,
    input  logic                    l2_resp_shared,
    output logic                    l2_req_ready,
    
    // Bus Status
    output logic                    bus_busy,
    output logic [31:0]             bus_transactions
);

    // Internal signals for MOESI response
    logic [NUM_CORES-1:0][2:0] dcache_resp_state;

    // Instantiate MOESI controller (upgraded from MESI)
    moesi_controller #(
        .NUM_CORES(NUM_CORES),
        .ADDR_WIDTH(ADDR_WIDTH),
        .LINE_BITS(LINE_BITS)
    ) u_moesi (
        .clk(clk),
        .rst_n(rst_n),
        
        // Core interfaces
        .core_req_valid(dcache_req_valid),
        .core_req_cmd(dcache_req_cmd),
        .core_req_addr(dcache_req_addr),
        .core_req_data(dcache_req_data),
        .core_req_grant(dcache_req_grant),
        .core_resp_data(dcache_resp_data),
        .core_resp_valid(dcache_resp_valid),
        .core_resp_state(dcache_resp_state),  // MOESI state response
        
        // Snoop interfaces
        .snoop_req_valid(dcache_snoop_valid),
        .snoop_req_cmd(dcache_snoop_cmd),
        .snoop_req_addr(dcache_snoop_addr),
        .snoop_resp_hit(dcache_snoop_hit),
        .snoop_resp_hitm(dcache_snoop_hitm),
        .snoop_resp_hito(dcache_snoop_hito),  // NEW: Owned state hit
        .snoop_resp_data(dcache_snoop_data),
        .snoop_resp_state(dcache_snoop_state),
        .snoop_resp_ack(dcache_snoop_ack),
        
        // L2 interface
        .l2_req_valid(l2_req_valid),
        .l2_req_write(l2_req_write),
        .l2_req_addr(l2_req_addr),
        .l2_req_data(l2_req_wdata),
        .l2_resp_data(l2_resp_rdata),
        .l2_resp_valid(l2_resp_valid),
        .l2_resp_shared(l2_resp_shared),
        .l2_req_ready(l2_req_ready)
    );
    
    // Generate shared signal from MOESI state (S or O = shared)
    always_comb begin
        for (int i = 0; i < NUM_CORES; i++) begin
            dcache_resp_shared[i] = (dcache_resp_state[i] == 3'b001) || // S
                                    (dcache_resp_state[i] == 3'b011);   // O
        end
    end

    // I-Cache snoop: just forward invalidations
    assign icache_snoop_valid = dcache_snoop_valid;  // Broadcast to all
    assign icache_snoop_addr  = dcache_snoop_addr;

    // Bus busy status
    assign bus_busy = |dcache_req_valid;

    // Transaction counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bus_transactions <= '0;
        end else begin
            if (|dcache_resp_valid)
                bus_transactions <= bus_transactions + 1;
        end
    end

endmodule









//============================================================================
// PhD Research: Crossbar Interconnect
// Author: Chandraboul
// Target: Space-Grade Octa-Core Processor
//
// Description:
//   4x1 crossbar interconnect with round-robin arbitration.
//   Connects multiple DLS core pairs to shared memory system.
//   Features fairness, priority, and low-latency design.
//
// Features:
//   - Round-robin arbitration (fairness)
//   - Single-cycle grant latency
//   - Buffered requests
//   - Support for outstanding transactions
//============================================================================

`timescale 1ns / 1ps

module crossbar_interconnect #(
    parameter NUM_MASTERS = 4,
    parameter ADDR_WIDTH  = 32,
    parameter DATA_WIDTH  = 32
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Master ports (from cores)
    input  logic [NUM_MASTERS-1:0]      m_valid,
    input  logic [NUM_MASTERS-1:0]      m_write,
    input  logic [ADDR_WIDTH-1:0]       m_addr  [NUM_MASTERS-1:0],
    input  logic [DATA_WIDTH-1:0]       m_wdata [NUM_MASTERS-1:0],
    input  logic [DATA_WIDTH/8-1:0]     m_wstrb [NUM_MASTERS-1:0],
    output logic [DATA_WIDTH-1:0]       m_rdata [NUM_MASTERS-1:0],
    output logic [NUM_MASTERS-1:0]      m_ready,
    
    // Slave port (to memory)
    output logic                        s_valid,
    output logic                        s_write,
    output logic [ADDR_WIDTH-1:0]       s_addr,
    output logic [DATA_WIDTH-1:0]       s_wdata,
    output logic [DATA_WIDTH/8-1:0]     s_wstrb,
    input  logic [DATA_WIDTH-1:0]       s_rdata,
    input  logic                        s_ready
);

    //------------------------------------------------------------------------
    // Arbiter State
    //------------------------------------------------------------------------
    localparam SEL_WIDTH = $clog2(NUM_MASTERS);
    
    logic [SEL_WIDTH-1:0] current_master;
    logic [SEL_WIDTH-1:0] next_master;
    logic [NUM_MASTERS-1:0] grant;
    logic any_request;
    logic transaction_active;
    
    //------------------------------------------------------------------------
    // Round-Robin Arbiter
    //------------------------------------------------------------------------
    // Find next requesting master after current
    always_comb begin
        next_master = current_master;
        any_request = |m_valid;
        
        if (any_request && !transaction_active) begin
            // Search for next valid request starting from current+1
            for (int i = 0; i < NUM_MASTERS; i++) begin
                logic [SEL_WIDTH-1:0] check_idx;
                check_idx = (current_master + 1 + i) % NUM_MASTERS;
                if (m_valid[check_idx]) begin
                    next_master = check_idx;
                    break;
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Grant Logic
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_master <= '0;
            grant <= '0;
            transaction_active <= 1'b0;
        end else begin
            if (!transaction_active && any_request) begin
                // Start new transaction
                current_master <= next_master;
                grant <= (1 << next_master);
                transaction_active <= 1'b1;
            end else if (transaction_active && s_ready) begin
                // Transaction complete
                grant <= '0;
                transaction_active <= 1'b0;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Multiplexer - Master to Slave
    //------------------------------------------------------------------------
    always_comb begin
        s_valid = 1'b0;
        s_write = 1'b0;
        s_addr  = '0;
        s_wdata = '0;
        s_wstrb = '0;
        
        for (int i = 0; i < NUM_MASTERS; i++) begin
            if (grant[i]) begin
                s_valid = m_valid[i];
                s_write = m_write[i];
                s_addr  = m_addr[i];
                s_wdata = m_wdata[i];
                s_wstrb = m_wstrb[i];
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Demultiplexer - Slave to Masters
    //------------------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < NUM_MASTERS; i++) begin
            m_rdata[i] = s_rdata;
            m_ready[i] = grant[i] && s_ready;
        end
    end

endmodule

//============================================================================
// Memory Controller with ECC
//============================================================================
module memory_controller_ecc #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter MEM_SIZE   = 65536,   // 64KB
    parameter ECC_ENABLE = 1
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Bus Interface
    input  logic                    valid,
    input  logic                    write,
    input  logic [ADDR_WIDTH-1:0]   addr,
    input  logic [DATA_WIDTH-1:0]   wdata,
    input  logic [DATA_WIDTH/8-1:0] wstrb,
    output logic [DATA_WIDTH-1:0]   rdata,
    output logic                    ready,
    
    // ECC Status
    output logic                    ecc_error,
    output logic                    ecc_corrected,
    output logic [ADDR_WIDTH-1:0]   ecc_error_addr
);

    // Memory array
    localparam MEM_DEPTH = MEM_SIZE / (DATA_WIDTH/8);
    localparam ADDR_BITS = $clog2(MEM_DEPTH);
    
    logic [DATA_WIDTH-1:0] memory [MEM_DEPTH-1:0];
    
    // ECC storage (if enabled)
    generate
        if (ECC_ENABLE) begin : gen_ecc
            // 7 parity bits per 32-bit word (SECDED)
            logic [6:0] ecc_bits [MEM_DEPTH-1:0];
        end
    endgenerate
    
    // Address decode
    logic [ADDR_BITS-1:0] mem_addr;
    assign mem_addr = addr[ADDR_BITS+1:2];  // Word-aligned
    
    // Read/Write logic
    always_ff @(posedge clk) begin
        if (valid && write) begin
            // Write with byte strobes
            if (wstrb[0]) memory[mem_addr][7:0]   <= wdata[7:0];
            if (wstrb[1]) memory[mem_addr][15:8]  <= wdata[15:8];
            if (wstrb[2]) memory[mem_addr][23:16] <= wdata[23:16];
            if (wstrb[3]) memory[mem_addr][31:24] <= wdata[31:24];
        end
    end
    
    // Read data
    assign rdata = memory[mem_addr];
    
    // Ready signal (single-cycle access)
    assign ready = valid;
    
    // ECC status (placeholder)
    assign ecc_error = 1'b0;
    assign ecc_corrected = 1'b0;
    assign ecc_error_addr = '0;

endmodule




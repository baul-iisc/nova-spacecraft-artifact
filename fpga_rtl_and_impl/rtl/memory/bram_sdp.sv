//============================================================================
// Simple Dual-Port Block RAM - Guaranteed BRAM Inference
// One write port, one read port, registered outputs
//============================================================================

`timescale 1ns / 1ps

module bram_sdp #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 10,
    parameter DEPTH      = 1024
)(
    input  logic                    clk,
    
    // Write port
    input  logic                    wr_en,
    input  logic [ADDR_WIDTH-1:0]   wr_addr,
    input  logic [DATA_WIDTH-1:0]   wr_data,
    
    // Read port
    input  logic                    rd_en,
    input  logic [ADDR_WIDTH-1:0]   rd_addr,
    output logic [DATA_WIDTH-1:0]   rd_data
);

    // Infer Block RAM - no reset on memory array
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [DEPTH-1:0];
    
    // Write port - synchronous write
    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end
    
    // Read port - synchronous read with registered output
    always_ff @(posedge clk) begin
        if (rd_en) begin
            rd_data <= mem[rd_addr];
        end
    end

endmodule

//============================================================================
// True Dual-Port Block RAM - Guaranteed BRAM Inference  
// Both ports can read and write, registered outputs
//============================================================================

module bram_tdp #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 10,
    parameter DEPTH      = 1024
)(
    input  logic                    clk,
    
    // Port A
    input  logic                    a_en,
    input  logic                    a_wr,
    input  logic [ADDR_WIDTH-1:0]   a_addr,
    input  logic [DATA_WIDTH-1:0]   a_wdata,
    output logic [DATA_WIDTH-1:0]   a_rdata,
    
    // Port B
    input  logic                    b_en,
    input  logic                    b_wr,
    input  logic [ADDR_WIDTH-1:0]   b_addr,
    input  logic [DATA_WIDTH-1:0]   b_wdata,
    output logic [DATA_WIDTH-1:0]   b_rdata
);

    // Infer Block RAM - no reset on memory array
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [DEPTH-1:0];
    
    // Port A
    always_ff @(posedge clk) begin
        if (a_en) begin
            if (a_wr) begin
                mem[a_addr] <= a_wdata;
            end
            a_rdata <= mem[a_addr];
        end
    end
    
    // Port B  
    always_ff @(posedge clk) begin
        if (b_en) begin
            if (b_wr) begin
                mem[b_addr] <= b_wdata;
            end
            b_rdata <= mem[b_addr];
        end
    end

endmodule

//============================================================================
// Byte-Write Block RAM - For data caches with byte enables
//============================================================================

module bram_byte_wr #(
    parameter DATA_WIDTH = 64,      // Must be multiple of 8
    parameter ADDR_WIDTH = 10,
    parameter DEPTH      = 1024
)(
    input  logic                    clk,
    
    // Write port with byte enables
    input  logic                    wr_en,
    input  logic [ADDR_WIDTH-1:0]   wr_addr,
    input  logic [DATA_WIDTH-1:0]   wr_data,
    input  logic [DATA_WIDTH/8-1:0] wr_be,
    
    // Read port
    input  logic                    rd_en,
    input  logic [ADDR_WIDTH-1:0]   rd_addr,
    output logic [DATA_WIDTH-1:0]   rd_data
);

    localparam NUM_BYTES = DATA_WIDTH / 8;
    
    // Infer Block RAM with byte-write enables
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [DEPTH-1:0];
    
    // Write port with byte enables
    always_ff @(posedge clk) begin
        if (wr_en) begin
            for (int i = 0; i < NUM_BYTES; i++) begin
                if (wr_be[i]) begin
                    mem[wr_addr][i*8 +: 8] <= wr_data[i*8 +: 8];
                end
            end
        end
    end
    
    // Read port - synchronous read
    always_ff @(posedge clk) begin
        if (rd_en) begin
            rd_data <= mem[rd_addr];
        end
    end

endmodule


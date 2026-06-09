//============================================================================
// PhD Research: Triple Modular Redundancy (TMR) Voter
// Author: Chandraboul
// Target: Space-Grade Fault Tolerance
//
// Description:
//   TMR voter for radiation-hardened spacecraft applications.
//   Uses majority voting to mask Single Event Upsets (SEUs).
//
// Usage:
//   Instantiate critical logic three times, connect outputs to this voter.
//   If one copy experiences an SEU, the voter masks the error.
//
// References:
//   - ESA ECSS-E-HB-50A: Space Engineering Data Systems
//   - NASA/TM-2000-210193: Radiation Effects on FPGAs
//============================================================================

`timescale 1ns / 1ps

module tmr_voter #(
    parameter WIDTH = 32
)(
    input  logic [WIDTH-1:0] in_a,
    input  logic [WIDTH-1:0] in_b,
    input  logic [WIDTH-1:0] in_c,
    output logic [WIDTH-1:0] out,
    output logic             error_detected,
    output logic [2:0]       vote_status  // Which inputs agree
);

    // Majority voting - bit by bit
    genvar i;
    generate
        for (i = 0; i < WIDTH; i++) begin : voter_bits
            // Majority function: (A & B) | (B & C) | (A & C)
            assign out[i] = (in_a[i] & in_b[i]) | 
                           (in_b[i] & in_c[i]) | 
                           (in_a[i] & in_c[i]);
        end
    endgenerate
    
    // Error detection
    logic ab_match, bc_match, ac_match;
    assign ab_match = (in_a == in_b);
    assign bc_match = (in_b == in_c);
    assign ac_match = (in_a == in_c);
    
    // Error detected if any pair doesn't match
    assign error_detected = ~(ab_match & bc_match & ac_match);
    
    // Vote status: which inputs agree with the output
    assign vote_status[0] = (in_a == out);  // A agrees
    assign vote_status[1] = (in_b == out);  // B agrees
    assign vote_status[2] = (in_c == out);  // C agrees

endmodule

//============================================================================
// TMR Register with Scrubbing
//============================================================================
module tmr_register #(
    parameter WIDTH = 32,
    parameter RESET_VALUE = 0
)(
    input  logic             clk,
    input  logic             rst_n,
    input  logic             we,
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q,
    output logic             seu_detected,
    input  logic             scrub_enable
);

    // Three copies of the register
    logic [WIDTH-1:0] reg_a, reg_b, reg_c;
    logic [WIDTH-1:0] voted_value;
    logic             error;
    
    // TMR Voter
    tmr_voter #(.WIDTH(WIDTH)) voter (
        .in_a           (reg_a),
        .in_b           (reg_b),
        .in_c           (reg_c),
        .out            (voted_value),
        .error_detected (error),
        .vote_status    ()
    );
    
    // Register A
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_a <= RESET_VALUE;
        else if (we)
            reg_a <= d;
        else if (scrub_enable && error)
            reg_a <= voted_value;  // Self-correct
    end
    
    // Register B
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_b <= RESET_VALUE;
        else if (we)
            reg_b <= d;
        else if (scrub_enable && error)
            reg_b <= voted_value;
    end
    
    // Register C
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reg_c <= RESET_VALUE;
        else if (we)
            reg_c <= d;
        else if (scrub_enable && error)
            reg_c <= voted_value;
    end
    
    assign q = voted_value;
    assign seu_detected = error;

endmodule

//============================================================================
// SECDED (Single Error Correct, Double Error Detect) ECC
// For memory protection
//============================================================================
module ecc_secded_enc #(
    parameter DATA_WIDTH = 32,
    parameter PARITY_WIDTH = 7  // For 32-bit data: 7 parity bits
)(
    input  logic [DATA_WIDTH-1:0]   data_in,
    output logic [DATA_WIDTH+PARITY_WIDTH-1:0] data_out
);

    // Hamming (38,32) SECDED encoding
    // Parity bit positions: 1, 2, 4, 8, 16, 32, 64 (overall parity at MSB)
    
    logic [6:0] parity;
    
    // Calculate parity bits (simplified - expand for full implementation)
    assign parity[0] = ^{data_in[0], data_in[1], data_in[3], data_in[4], 
                         data_in[6], data_in[8], data_in[10], data_in[11],
                         data_in[13], data_in[15], data_in[17], data_in[19],
                         data_in[21], data_in[23], data_in[25], data_in[26],
                         data_in[28], data_in[30]};
    
    assign parity[1] = ^{data_in[0], data_in[2], data_in[3], data_in[5],
                         data_in[6], data_in[9], data_in[10], data_in[12],
                         data_in[13], data_in[16], data_in[17], data_in[20],
                         data_in[21], data_in[24], data_in[25], data_in[27],
                         data_in[28], data_in[31]};
    
    assign parity[2] = ^{data_in[1], data_in[2], data_in[3], data_in[7],
                         data_in[8], data_in[9], data_in[10], data_in[14],
                         data_in[15], data_in[16], data_in[17], data_in[22],
                         data_in[23], data_in[24], data_in[25], data_in[29],
                         data_in[30], data_in[31]};
    
    assign parity[3] = ^{data_in[4], data_in[5], data_in[6], data_in[7],
                         data_in[8], data_in[9], data_in[10], data_in[18],
                         data_in[19], data_in[20], data_in[21], data_in[22],
                         data_in[23], data_in[24], data_in[25]};
    
    assign parity[4] = ^{data_in[11], data_in[12], data_in[13], data_in[14],
                         data_in[15], data_in[16], data_in[17], data_in[18],
                         data_in[19], data_in[20], data_in[21], data_in[22],
                         data_in[23], data_in[24], data_in[25]};
    
    assign parity[5] = ^{data_in[26], data_in[27], data_in[28], data_in[29],
                         data_in[30], data_in[31]};
    
    // Overall parity (for double error detection)
    assign parity[6] = ^{data_in, parity[5:0]};
    
    assign data_out = {parity, data_in};

endmodule

// Note: watchdog_timer module is defined separately in watchdog_timer.sv



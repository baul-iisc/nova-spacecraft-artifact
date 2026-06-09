//============================================================================
// SECDED ECC Module for Space-Grade Cache
// Single Error Correct, Double Error Detect
// Author: Chandraboul
//
// Implements Hamming(72,64) code for 64-bit data words
// - 64 data bits + 8 check bits = 72 bits total
// - Corrects any single-bit error
// - Detects any double-bit error
//============================================================================

`timescale 1ns / 1ps

module ecc_secded #(
    parameter DATA_WIDTH = 64,   // Data width (supports 32, 64)
    parameter ECC_WIDTH  = 8     // ECC bits (8 for 64-bit, 7 for 32-bit)
)(
    input  logic                     clk,
    input  logic                     rst_n,
    
    // Encode Interface
    input  logic [DATA_WIDTH-1:0]    data_in,
    output logic [DATA_WIDTH+ECC_WIDTH-1:0] encoded_out,
    
    // Decode Interface
    input  logic [DATA_WIDTH+ECC_WIDTH-1:0] encoded_in,
    output logic [DATA_WIDTH-1:0]    data_out,
    output logic                     single_error,    // Correctable error
    output logic                     double_error,    // Uncorrectable error
    output logic [6:0]               error_position   // Position of single error
);

    // Syndrome calculation for Hamming(72,64)
    logic [ECC_WIDTH-1:0] syndrome;
    logic [ECC_WIDTH-1:0] check_bits_calc;
    logic [ECC_WIDTH-1:0] check_bits_stored;
    logic overall_parity;
    logic overall_parity_calc;

    //------------------------------------------------------------------------
    // ECC Encoding - Generate check bits
    //------------------------------------------------------------------------
    // Hamming code check bit positions: 1, 2, 4, 8, 16, 32, 64 (plus overall parity)
    
    always_comb begin
        // Check bit c0 (position 1): covers bits 1,3,5,7,9,11,...
        check_bits_calc[0] = data_in[0] ^ data_in[1] ^ data_in[3] ^ data_in[4] ^ 
                             data_in[6] ^ data_in[8] ^ data_in[10] ^ data_in[11] ^
                             data_in[13] ^ data_in[15] ^ data_in[17] ^ data_in[19] ^
                             data_in[21] ^ data_in[23] ^ data_in[25] ^ data_in[26] ^
                             data_in[28] ^ data_in[30] ^ data_in[32] ^ data_in[34] ^
                             data_in[36] ^ data_in[38] ^ data_in[40] ^ data_in[42] ^
                             data_in[44] ^ data_in[46] ^ data_in[48] ^ data_in[50] ^
                             data_in[52] ^ data_in[54] ^ data_in[56] ^ data_in[57] ^
                             data_in[59] ^ data_in[61] ^ data_in[63];
        
        // Check bit c1 (position 2): covers bits 2,3,6,7,10,11,...
        check_bits_calc[1] = data_in[0] ^ data_in[2] ^ data_in[3] ^ data_in[5] ^
                             data_in[6] ^ data_in[9] ^ data_in[10] ^ data_in[12] ^
                             data_in[13] ^ data_in[16] ^ data_in[17] ^ data_in[19] ^
                             data_in[20] ^ data_in[22] ^ data_in[23] ^ data_in[25] ^
                             data_in[26] ^ data_in[29] ^ data_in[30] ^ data_in[32] ^
                             data_in[33] ^ data_in[35] ^ data_in[36] ^ data_in[39] ^
                             data_in[40] ^ data_in[42] ^ data_in[43] ^ data_in[45] ^
                             data_in[46] ^ data_in[49] ^ data_in[50] ^ data_in[52] ^
                             data_in[53] ^ data_in[55] ^ data_in[56] ^ data_in[58] ^
                             data_in[59] ^ data_in[62] ^ data_in[63];
        
        // Check bit c2 (position 4): covers bits 4-7, 12-15, 20-23,...
        check_bits_calc[2] = data_in[1] ^ data_in[2] ^ data_in[3] ^ data_in[7] ^
                             data_in[8] ^ data_in[9] ^ data_in[10] ^ data_in[14] ^
                             data_in[15] ^ data_in[16] ^ data_in[17] ^ data_in[21] ^
                             data_in[22] ^ data_in[23] ^ data_in[24] ^ data_in[28] ^
                             data_in[29] ^ data_in[30] ^ data_in[31] ^ data_in[35] ^
                             data_in[36] ^ data_in[37] ^ data_in[38] ^ data_in[42] ^
                             data_in[43] ^ data_in[44] ^ data_in[45] ^ data_in[49] ^
                             data_in[50] ^ data_in[51] ^ data_in[52] ^ data_in[56] ^
                             data_in[57] ^ data_in[58] ^ data_in[59] ^ data_in[63];
        
        // Check bit c3 (position 8): covers bits 8-15, 24-31, 40-47, 56-63
        check_bits_calc[3] = data_in[4] ^ data_in[5] ^ data_in[6] ^ data_in[7] ^
                             data_in[8] ^ data_in[9] ^ data_in[10] ^ data_in[18] ^
                             data_in[19] ^ data_in[20] ^ data_in[21] ^ data_in[22] ^
                             data_in[23] ^ data_in[24] ^ data_in[25] ^ data_in[33] ^
                             data_in[34] ^ data_in[35] ^ data_in[36] ^ data_in[37] ^
                             data_in[38] ^ data_in[39] ^ data_in[40] ^ data_in[48] ^
                             data_in[49] ^ data_in[50] ^ data_in[51] ^ data_in[52] ^
                             data_in[53] ^ data_in[54] ^ data_in[55];
        
        // Check bit c4 (position 16): covers bits 16-31, 48-63
        check_bits_calc[4] = data_in[11] ^ data_in[12] ^ data_in[13] ^ data_in[14] ^
                             data_in[15] ^ data_in[16] ^ data_in[17] ^ data_in[18] ^
                             data_in[19] ^ data_in[20] ^ data_in[21] ^ data_in[22] ^
                             data_in[23] ^ data_in[24] ^ data_in[25] ^ data_in[41] ^
                             data_in[42] ^ data_in[43] ^ data_in[44] ^ data_in[45] ^
                             data_in[46] ^ data_in[47] ^ data_in[48] ^ data_in[49] ^
                             data_in[50] ^ data_in[51] ^ data_in[52] ^ data_in[53] ^
                             data_in[54] ^ data_in[55] ^ data_in[56];
        
        // Check bit c5 (position 32): covers bits 32-63
        check_bits_calc[5] = data_in[26] ^ data_in[27] ^ data_in[28] ^ data_in[29] ^
                             data_in[30] ^ data_in[31] ^ data_in[32] ^ data_in[33] ^
                             data_in[34] ^ data_in[35] ^ data_in[36] ^ data_in[37] ^
                             data_in[38] ^ data_in[39] ^ data_in[40] ^ data_in[41] ^
                             data_in[42] ^ data_in[43] ^ data_in[44] ^ data_in[45] ^
                             data_in[46] ^ data_in[47] ^ data_in[48] ^ data_in[49] ^
                             data_in[50] ^ data_in[51] ^ data_in[52] ^ data_in[53] ^
                             data_in[54] ^ data_in[55] ^ data_in[56];
        
        // Check bit c6 (position 64): covers bits 57-63 (upper bits)
        check_bits_calc[6] = data_in[57] ^ data_in[58] ^ data_in[59] ^ data_in[60] ^
                             data_in[61] ^ data_in[62] ^ data_in[63];
        
        // Overall parity (for double error detection)
        check_bits_calc[7] = ^data_in ^ ^check_bits_calc[6:0];
    end
    
    // Encoded output: [ECC | DATA]
    assign encoded_out = {check_bits_calc, data_in};

    //------------------------------------------------------------------------
    // ECC Decoding - Syndrome calculation and error correction
    //------------------------------------------------------------------------
    
    // Extract stored check bits and data
    assign check_bits_stored = encoded_in[DATA_WIDTH+ECC_WIDTH-1:DATA_WIDTH];
    
    // Recalculate check bits from stored data
    logic [ECC_WIDTH-1:0] check_bits_recalc;
    logic [DATA_WIDTH-1:0] stored_data;
    
    assign stored_data = encoded_in[DATA_WIDTH-1:0];
    
    always_comb begin
        // Recalculate using same formulas as encoding
        check_bits_recalc[0] = stored_data[0] ^ stored_data[1] ^ stored_data[3] ^ stored_data[4] ^ 
                               stored_data[6] ^ stored_data[8] ^ stored_data[10] ^ stored_data[11] ^
                               stored_data[13] ^ stored_data[15] ^ stored_data[17] ^ stored_data[19] ^
                               stored_data[21] ^ stored_data[23] ^ stored_data[25] ^ stored_data[26] ^
                               stored_data[28] ^ stored_data[30] ^ stored_data[32] ^ stored_data[34] ^
                               stored_data[36] ^ stored_data[38] ^ stored_data[40] ^ stored_data[42] ^
                               stored_data[44] ^ stored_data[46] ^ stored_data[48] ^ stored_data[50] ^
                               stored_data[52] ^ stored_data[54] ^ stored_data[56] ^ stored_data[57] ^
                               stored_data[59] ^ stored_data[61] ^ stored_data[63];
        
        check_bits_recalc[1] = stored_data[0] ^ stored_data[2] ^ stored_data[3] ^ stored_data[5] ^
                               stored_data[6] ^ stored_data[9] ^ stored_data[10] ^ stored_data[12] ^
                               stored_data[13] ^ stored_data[16] ^ stored_data[17] ^ stored_data[19] ^
                               stored_data[20] ^ stored_data[22] ^ stored_data[23] ^ stored_data[25] ^
                               stored_data[26] ^ stored_data[29] ^ stored_data[30] ^ stored_data[32] ^
                               stored_data[33] ^ stored_data[35] ^ stored_data[36] ^ stored_data[39] ^
                               stored_data[40] ^ stored_data[42] ^ stored_data[43] ^ stored_data[45] ^
                               stored_data[46] ^ stored_data[49] ^ stored_data[50] ^ stored_data[52] ^
                               stored_data[53] ^ stored_data[55] ^ stored_data[56] ^ stored_data[58] ^
                               stored_data[59] ^ stored_data[62] ^ stored_data[63];
        
        check_bits_recalc[2] = stored_data[1] ^ stored_data[2] ^ stored_data[3] ^ stored_data[7] ^
                               stored_data[8] ^ stored_data[9] ^ stored_data[10] ^ stored_data[14] ^
                               stored_data[15] ^ stored_data[16] ^ stored_data[17] ^ stored_data[21] ^
                               stored_data[22] ^ stored_data[23] ^ stored_data[24] ^ stored_data[28] ^
                               stored_data[29] ^ stored_data[30] ^ stored_data[31] ^ stored_data[35] ^
                               stored_data[36] ^ stored_data[37] ^ stored_data[38] ^ stored_data[42] ^
                               stored_data[43] ^ stored_data[44] ^ stored_data[45] ^ stored_data[49] ^
                               stored_data[50] ^ stored_data[51] ^ stored_data[52] ^ stored_data[56] ^
                               stored_data[57] ^ stored_data[58] ^ stored_data[59] ^ stored_data[63];
        
        check_bits_recalc[3] = stored_data[4] ^ stored_data[5] ^ stored_data[6] ^ stored_data[7] ^
                               stored_data[8] ^ stored_data[9] ^ stored_data[10] ^ stored_data[18] ^
                               stored_data[19] ^ stored_data[20] ^ stored_data[21] ^ stored_data[22] ^
                               stored_data[23] ^ stored_data[24] ^ stored_data[25] ^ stored_data[33] ^
                               stored_data[34] ^ stored_data[35] ^ stored_data[36] ^ stored_data[37] ^
                               stored_data[38] ^ stored_data[39] ^ stored_data[40] ^ stored_data[48] ^
                               stored_data[49] ^ stored_data[50] ^ stored_data[51] ^ stored_data[52] ^
                               stored_data[53] ^ stored_data[54] ^ stored_data[55];
        
        check_bits_recalc[4] = stored_data[11] ^ stored_data[12] ^ stored_data[13] ^ stored_data[14] ^
                               stored_data[15] ^ stored_data[16] ^ stored_data[17] ^ stored_data[18] ^
                               stored_data[19] ^ stored_data[20] ^ stored_data[21] ^ stored_data[22] ^
                               stored_data[23] ^ stored_data[24] ^ stored_data[25] ^ stored_data[41] ^
                               stored_data[42] ^ stored_data[43] ^ stored_data[44] ^ stored_data[45] ^
                               stored_data[46] ^ stored_data[47] ^ stored_data[48] ^ stored_data[49] ^
                               stored_data[50] ^ stored_data[51] ^ stored_data[52] ^ stored_data[53] ^
                               stored_data[54] ^ stored_data[55] ^ stored_data[56];
        
        check_bits_recalc[5] = stored_data[26] ^ stored_data[27] ^ stored_data[28] ^ stored_data[29] ^
                               stored_data[30] ^ stored_data[31] ^ stored_data[32] ^ stored_data[33] ^
                               stored_data[34] ^ stored_data[35] ^ stored_data[36] ^ stored_data[37] ^
                               stored_data[38] ^ stored_data[39] ^ stored_data[40] ^ stored_data[41] ^
                               stored_data[42] ^ stored_data[43] ^ stored_data[44] ^ stored_data[45] ^
                               stored_data[46] ^ stored_data[47] ^ stored_data[48] ^ stored_data[49] ^
                               stored_data[50] ^ stored_data[51] ^ stored_data[52] ^ stored_data[53] ^
                               stored_data[54] ^ stored_data[55] ^ stored_data[56];
        
        check_bits_recalc[6] = stored_data[57] ^ stored_data[58] ^ stored_data[59] ^ stored_data[60] ^
                               stored_data[61] ^ stored_data[62] ^ stored_data[63];
        
        check_bits_recalc[7] = ^stored_data ^ ^check_bits_recalc[6:0];
    end
    
    // Syndrome = stored XOR recalculated
    assign syndrome = check_bits_stored ^ check_bits_recalc;
    
    // Overall parity check
    assign overall_parity = ^encoded_in;
    
    // Error detection
    assign single_error = (syndrome[6:0] != 7'b0) && overall_parity;
    assign double_error = (syndrome[6:0] != 7'b0) && !overall_parity;
    assign error_position = syndrome[6:0];
    
    // Error correction (flip the erroneous bit)
    logic [DATA_WIDTH-1:0] corrected_data;
    
    always_comb begin
        corrected_data = stored_data;
        if (single_error && error_position < DATA_WIDTH) begin
            corrected_data[error_position] = ~stored_data[error_position];
        end
    end
    
    assign data_out = corrected_data;

endmodule

//============================================================================
// Simplified 32-bit SECDED for Tags
//============================================================================
module ecc_secded_32 (
    input  logic [31:0]  data_in,
    output logic [38:0]  encoded_out,   // 32 + 7 ECC bits
    
    input  logic [38:0]  encoded_in,
    output logic [31:0]  data_out,
    output logic         single_error,
    output logic         double_error
);

    logic [6:0] check_bits;
    logic [6:0] syndrome;
    
    // Encoding
    always_comb begin
        check_bits[0] = data_in[0] ^ data_in[1] ^ data_in[3] ^ data_in[4] ^ data_in[6] ^
                        data_in[8] ^ data_in[10] ^ data_in[11] ^ data_in[13] ^ data_in[15] ^
                        data_in[17] ^ data_in[19] ^ data_in[21] ^ data_in[23] ^ data_in[25] ^
                        data_in[26] ^ data_in[28] ^ data_in[30];
        check_bits[1] = data_in[0] ^ data_in[2] ^ data_in[3] ^ data_in[5] ^ data_in[6] ^
                        data_in[9] ^ data_in[10] ^ data_in[12] ^ data_in[13] ^ data_in[16] ^
                        data_in[17] ^ data_in[19] ^ data_in[20] ^ data_in[22] ^ data_in[23] ^
                        data_in[25] ^ data_in[26] ^ data_in[29] ^ data_in[30];
        check_bits[2] = data_in[1] ^ data_in[2] ^ data_in[3] ^ data_in[7] ^ data_in[8] ^
                        data_in[9] ^ data_in[10] ^ data_in[14] ^ data_in[15] ^ data_in[16] ^
                        data_in[17] ^ data_in[21] ^ data_in[22] ^ data_in[23] ^ data_in[24] ^
                        data_in[28] ^ data_in[29] ^ data_in[30] ^ data_in[31];
        check_bits[3] = data_in[4] ^ data_in[5] ^ data_in[6] ^ data_in[7] ^ data_in[8] ^
                        data_in[9] ^ data_in[10] ^ data_in[18] ^ data_in[19] ^ data_in[20] ^
                        data_in[21] ^ data_in[22] ^ data_in[23] ^ data_in[24] ^ data_in[25];
        check_bits[4] = data_in[11] ^ data_in[12] ^ data_in[13] ^ data_in[14] ^ data_in[15] ^
                        data_in[16] ^ data_in[17] ^ data_in[18] ^ data_in[19] ^ data_in[20] ^
                        data_in[21] ^ data_in[22] ^ data_in[23] ^ data_in[24] ^ data_in[25];
        check_bits[5] = data_in[26] ^ data_in[27] ^ data_in[28] ^ data_in[29] ^ data_in[30] ^
                        data_in[31];
        check_bits[6] = ^data_in ^ ^check_bits[5:0];  // Overall parity
    end
    
    assign encoded_out = {check_bits, data_in};
    
    // Decoding (simplified - full implementation similar to 64-bit)
    logic [6:0] stored_check;
    logic [31:0] stored_data;
    logic [6:0] recalc_check;
    
    assign stored_check = encoded_in[38:32];
    assign stored_data = encoded_in[31:0];
    
    always_comb begin
        recalc_check[0] = stored_data[0] ^ stored_data[1] ^ stored_data[3] ^ stored_data[4] ^ stored_data[6] ^
                          stored_data[8] ^ stored_data[10] ^ stored_data[11] ^ stored_data[13] ^ stored_data[15] ^
                          stored_data[17] ^ stored_data[19] ^ stored_data[21] ^ stored_data[23] ^ stored_data[25] ^
                          stored_data[26] ^ stored_data[28] ^ stored_data[30];
        recalc_check[1] = stored_data[0] ^ stored_data[2] ^ stored_data[3] ^ stored_data[5] ^ stored_data[6] ^
                          stored_data[9] ^ stored_data[10] ^ stored_data[12] ^ stored_data[13] ^ stored_data[16] ^
                          stored_data[17] ^ stored_data[19] ^ stored_data[20] ^ stored_data[22] ^ stored_data[23] ^
                          stored_data[25] ^ stored_data[26] ^ stored_data[29] ^ stored_data[30];
        recalc_check[2] = stored_data[1] ^ stored_data[2] ^ stored_data[3] ^ stored_data[7] ^ stored_data[8] ^
                          stored_data[9] ^ stored_data[10] ^ stored_data[14] ^ stored_data[15] ^ stored_data[16] ^
                          stored_data[17] ^ stored_data[21] ^ stored_data[22] ^ stored_data[23] ^ stored_data[24] ^
                          stored_data[28] ^ stored_data[29] ^ stored_data[30] ^ stored_data[31];
        recalc_check[3] = stored_data[4] ^ stored_data[5] ^ stored_data[6] ^ stored_data[7] ^ stored_data[8] ^
                          stored_data[9] ^ stored_data[10] ^ stored_data[18] ^ stored_data[19] ^ stored_data[20] ^
                          stored_data[21] ^ stored_data[22] ^ stored_data[23] ^ stored_data[24] ^ stored_data[25];
        recalc_check[4] = stored_data[11] ^ stored_data[12] ^ stored_data[13] ^ stored_data[14] ^ stored_data[15] ^
                          stored_data[16] ^ stored_data[17] ^ stored_data[18] ^ stored_data[19] ^ stored_data[20] ^
                          stored_data[21] ^ stored_data[22] ^ stored_data[23] ^ stored_data[24] ^ stored_data[25];
        recalc_check[5] = stored_data[26] ^ stored_data[27] ^ stored_data[28] ^ stored_data[29] ^ stored_data[30] ^
                          stored_data[31];
        recalc_check[6] = ^stored_data ^ ^recalc_check[5:0];
    end
    
    assign syndrome = stored_check ^ recalc_check;
    assign single_error = (syndrome[5:0] != 6'b0) && syndrome[6];
    assign double_error = (syndrome[5:0] != 6'b0) && !syndrome[6];
    
    // Correct single bit error
    logic [31:0] corrected;
    always_comb begin
        corrected = stored_data;
        if (single_error && syndrome[5:0] < 32)
            corrected[syndrome[5:0]] = ~stored_data[syndrome[5:0]];
    end
    assign data_out = corrected;

endmodule









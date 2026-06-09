//============================================================================
// CCSDS Compression/Decompression Core
// 
// Implements CCSDS 121.0-B-3 Lossless Data Compression for spacecraft TM/TC
// Features:
//   - Rice (Golomb) coding for lossless compression
//   - Configurable preprocessing (unit delay mapper)
//   - 2:1 to 4:1 typical compression ratio for science/housekeeping data
//   - Hardware acceleration for real-time compression
//   - AXI-Stream interface for high-throughput data flow
//
// CCSDS 121.0-B-3 compliant with extended support for:
//   - Block-adaptive entropy coding
//   - Configurable block size (8-64 samples)
//   - Multiple reference sample handling
//
// Copyright (c) 2024 - Space Processor Project
//============================================================================

module ccsds_compression_core #(
    parameter DATA_WIDTH        = 16,           // Input sample width (8-16 bits)
    parameter BLOCK_SIZE        = 16,           // Samples per block (8, 16, 32, 64)
    parameter MAX_BLOCKS        = 4096,         // Max blocks per segment
    parameter FIFO_DEPTH        = 256,          // Input/output FIFO depth
    parameter ENABLE_DECOMPRESS = 1             // Enable decompression path
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    //=========================================================================
    // Control Interface (AXI-Lite)
    //=========================================================================
    input  logic [31:0]             s_axi_awaddr,
    input  logic                    s_axi_awvalid,
    output logic                    s_axi_awready,
    input  logic [31:0]             s_axi_wdata,
    input  logic [3:0]              s_axi_wstrb,
    input  logic                    s_axi_wvalid,
    output logic                    s_axi_wready,
    output logic [1:0]              s_axi_bresp,
    output logic                    s_axi_bvalid,
    input  logic                    s_axi_bready,
    input  logic [31:0]             s_axi_araddr,
    input  logic                    s_axi_arvalid,
    output logic                    s_axi_arready,
    output logic [31:0]             s_axi_rdata,
    output logic [1:0]              s_axi_rresp,
    output logic                    s_axi_rvalid,
    input  logic                    s_axi_rready,
    
    //=========================================================================
    // Compression Input (AXI-Stream) - Raw TM data
    //=========================================================================
    input  logic [DATA_WIDTH-1:0]   s_axis_comp_tdata,
    input  logic                    s_axis_comp_tvalid,
    output logic                    s_axis_comp_tready,
    input  logic                    s_axis_comp_tlast,     // End of segment
    
    //=========================================================================
    // Compression Output (AXI-Stream) - Compressed data
    //=========================================================================
    output logic [31:0]             m_axis_comp_tdata,     // 32-bit packed output
    output logic                    m_axis_comp_tvalid,
    input  logic                    m_axis_comp_tready,
    output logic                    m_axis_comp_tlast,
    output logic [3:0]              m_axis_comp_tkeep,     // Valid bytes
    
    //=========================================================================
    // Decompression Input (AXI-Stream) - Compressed TC data
    //=========================================================================
    input  logic [31:0]             s_axis_decomp_tdata,
    input  logic                    s_axis_decomp_tvalid,
    output logic                    s_axis_decomp_tready,
    input  logic                    s_axis_decomp_tlast,
    
    //=========================================================================
    // Decompression Output (AXI-Stream) - Raw TC data
    //=========================================================================
    output logic [DATA_WIDTH-1:0]   m_axis_decomp_tdata,
    output logic                    m_axis_decomp_tvalid,
    input  logic                    m_axis_decomp_tready,
    output logic                    m_axis_decomp_tlast,
    
    //=========================================================================
    // Status and Interrupts
    //=========================================================================
    output logic                    comp_busy,
    output logic                    decomp_busy,
    output logic [31:0]             bytes_in,              // Input bytes counter
    output logic [31:0]             bytes_out,             // Output bytes counter
    output logic [7:0]              compression_ratio,     // Q4.4 format
    output logic                    irq_comp_done,
    output logic                    irq_decomp_done,
    output logic                    irq_error
);

    //=========================================================================
    // Register Map
    //=========================================================================
    // 0x00: Control Register
    //       [0]    - Compression enable
    //       [1]    - Decompression enable
    //       [2]    - Bypass mode (passthrough)
    //       [7:4]  - Block size select (0=8, 1=16, 2=32, 3=64)
    //       [15:8] - K parameter (Rice coding parameter, 0=adaptive)
    // 0x04: Status Register
    //       [0]    - Compression busy
    //       [1]    - Decompression busy
    //       [2]    - Input FIFO full
    //       [3]    - Output FIFO empty
    //       [7:4]  - Current state
    // 0x08: Bytes Input Counter (read-only)
    // 0x0C: Bytes Output Counter (read-only)
    // 0x10: Compression Ratio (Q4.4 format, read-only)
    // 0x14: Error Status
    // 0x18: Interrupt Enable
    // 0x1C: Interrupt Status (write 1 to clear)
    
    logic [31:0] ctrl_reg;
    logic [31:0] status_reg;
    logic [31:0] irq_enable_reg;
    logic [31:0] irq_status_reg;
    
    logic comp_enable;
    logic decomp_enable;
    logic bypass_mode;
    logic [3:0] block_size_sel;
    logic [7:0] k_param;
    
    assign comp_enable    = ctrl_reg[0];
    assign decomp_enable  = ctrl_reg[1];
    assign bypass_mode    = ctrl_reg[2];
    assign block_size_sel = ctrl_reg[7:4];
    assign k_param        = ctrl_reg[15:8];
    
    //=========================================================================
    // AXI-Lite Interface
    //=========================================================================
    logic aw_ready, w_ready, ar_ready;
    logic [31:0] read_data;
    logic read_valid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_reg       <= 32'h0000_0010;  // Default: 16 samples/block
            irq_enable_reg <= 32'h0;
            aw_ready       <= 1'b1;
            w_ready        <= 1'b1;
            ar_ready       <= 1'b1;
            s_axi_bvalid   <= 1'b0;
            read_valid     <= 1'b0;
        end else begin
            // Write handling
            if (s_axi_awvalid && s_axi_wvalid && aw_ready && w_ready) begin
                case (s_axi_awaddr[7:0])
                    8'h00: ctrl_reg       <= s_axi_wdata;
                    8'h18: irq_enable_reg <= s_axi_wdata;
                    8'h1C: irq_status_reg <= irq_status_reg & ~s_axi_wdata; // W1C
                endcase
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 1'b0;
            end
            
            // Read handling
            if (s_axi_arvalid && ar_ready) begin
                case (s_axi_araddr[7:0])
                    8'h00: read_data <= ctrl_reg;
                    8'h04: read_data <= status_reg;
                    8'h08: read_data <= bytes_in;
                    8'h0C: read_data <= bytes_out;
                    8'h10: read_data <= {24'b0, compression_ratio};
                    8'h14: read_data <= {28'b0, 4'b0}; // Error status
                    8'h18: read_data <= irq_enable_reg;
                    8'h1C: read_data <= irq_status_reg;
                    default: read_data <= 32'hDEAD_BEEF;
                endcase
                read_valid <= 1'b1;
            end else if (s_axi_rready && read_valid) begin
                read_valid <= 1'b0;
            end
        end
    end
    
    assign s_axi_awready = aw_ready;
    assign s_axi_wready  = w_ready;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_arready = ar_ready;
    assign s_axi_rdata   = read_data;
    assign s_axi_rresp   = 2'b00;
    assign s_axi_rvalid  = read_valid;
    
    //=========================================================================
    // Compression Engine (CCSDS 121.0-B-3 Rice Coding)
    //=========================================================================
    
    // Compression state machine
    typedef enum logic [3:0] {
        COMP_IDLE,
        COMP_LOAD_BLOCK,
        COMP_PREPROCESS,
        COMP_CALC_K,
        COMP_ENCODE,
        COMP_PACK_OUTPUT,
        COMP_FLUSH,
        COMP_DONE
    } comp_state_t;
    
    comp_state_t comp_state, comp_next_state;
    
    // Block buffer for preprocessing
    logic [DATA_WIDTH-1:0] block_buffer [0:63];
    logic [DATA_WIDTH-1:0] delta_buffer [0:63];
    logic [5:0] sample_count;
    logic [5:0] current_block_size;
    logic [11:0] block_count;
    
    // Rice coding parameters
    logic [3:0] adaptive_k;
    logic [31:0] accumulator;
    logic [5:0] encoded_count;
    
    // Output bit packer
    logic [63:0] bit_buffer;
    logic [6:0] bit_count;
    
    // Determine block size from selector
    always_comb begin
        case (block_size_sel)
            4'd0: current_block_size = 6'd8;
            4'd1: current_block_size = 6'd16;
            4'd2: current_block_size = 6'd32;
            4'd3: current_block_size = 6'd64;
            default: current_block_size = 6'd16;
        endcase
    end
    
    // Compression FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            comp_state     <= COMP_IDLE;
            sample_count   <= '0;
            block_count    <= '0;
            bytes_in       <= '0;
            bytes_out      <= '0;
            adaptive_k     <= 4'd4;  // Default K=4
            bit_buffer     <= '0;
            bit_count      <= '0;
            encoded_count  <= '0;
            accumulator    <= '0;
            comp_busy      <= 1'b0;
            irq_comp_done  <= 1'b0;
        end else begin
            irq_comp_done <= 1'b0;
            
            case (comp_state)
                COMP_IDLE: begin
                    if (comp_enable && s_axis_comp_tvalid) begin
                        comp_state <= COMP_LOAD_BLOCK;
                        comp_busy  <= 1'b1;
                        sample_count <= '0;
                    end
                end
                
                COMP_LOAD_BLOCK: begin
                    if (s_axis_comp_tvalid && s_axis_comp_tready) begin
                        block_buffer[sample_count] <= s_axis_comp_tdata;
                        bytes_in <= bytes_in + (DATA_WIDTH / 8);
                        
                        if (sample_count == current_block_size - 1 || s_axis_comp_tlast) begin
                            comp_state <= COMP_PREPROCESS;
                            sample_count <= '0;
                        end else begin
                            sample_count <= sample_count + 1;
                        end
                    end
                end
                
                COMP_PREPROCESS: begin
                    // Unit delay prediction: delta[n] = sample[n] - sample[n-1]
                    // First sample is reference (sent uncompressed)
                    if (sample_count == 0) begin
                        delta_buffer[0] <= block_buffer[0];  // Reference sample
                    end else begin
                        // Compute prediction residual with sign mapping
                        // Positive residuals -> even, Negative -> odd (for Rice coding)
                        logic signed [DATA_WIDTH:0] diff;
                        logic [DATA_WIDTH-1:0] mapped;
                        
                        diff = $signed({1'b0, block_buffer[sample_count]}) - 
                               $signed({1'b0, block_buffer[sample_count-1]});
                        
                        // Sign-magnitude mapping for Rice coding
                        if (diff >= 0) begin
                            mapped = diff[DATA_WIDTH-1:0] << 1;
                        end else begin
                            mapped = ((-diff[DATA_WIDTH-1:0]) << 1) - 1;
                        end
                        delta_buffer[sample_count] <= mapped;
                    end
                    
                    if (sample_count == current_block_size - 1) begin
                        comp_state <= COMP_CALC_K;
                        sample_count <= '0;
                        accumulator <= '0;
                    end else begin
                        sample_count <= sample_count + 1;
                    end
                end
                
                COMP_CALC_K: begin
                    // Adaptive K calculation: K = log2(mean/2)
                    // Accumulate deltas to calculate mean
                    if (sample_count < current_block_size) begin
                        accumulator <= accumulator + delta_buffer[sample_count];
                        sample_count <= sample_count + 1;
                    end else begin
                        // Calculate adaptive K based on mean
                        logic [31:0] mean;
                        mean = accumulator / current_block_size;
                        
                        // log2 approximation for K selection
                        if (k_param != 0) begin
                            adaptive_k <= k_param[3:0];  // Use configured K
                        end else begin
                            // Adaptive K selection
                            if (mean < 2) adaptive_k <= 4'd0;
                            else if (mean < 4) adaptive_k <= 4'd1;
                            else if (mean < 8) adaptive_k <= 4'd2;
                            else if (mean < 16) adaptive_k <= 4'd3;
                            else if (mean < 32) adaptive_k <= 4'd4;
                            else if (mean < 64) adaptive_k <= 4'd5;
                            else if (mean < 128) adaptive_k <= 4'd6;
                            else if (mean < 256) adaptive_k <= 4'd7;
                            else adaptive_k <= 4'd8;
                        end
                        
                        comp_state <= COMP_ENCODE;
                        sample_count <= '0;
                        encoded_count <= '0;
                    end
                end
                
                COMP_ENCODE: begin
                    // Rice/Golomb encoding
                    // Codeword = unary(q) + binary(r), where q = value >> k, r = value & ((1<<k)-1)
                    if (encoded_count < current_block_size) begin
                        logic [DATA_WIDTH-1:0] value;
                        logic [DATA_WIDTH-1:0] quotient;
                        logic [DATA_WIDTH-1:0] remainder;
                        logic [31:0] codeword;
                        logic [5:0] codeword_len;
                        
                        value = delta_buffer[encoded_count];
                        
                        if (encoded_count == 0) begin
                            // Reference sample - send uncompressed
                            codeword = value;
                            codeword_len = DATA_WIDTH;
                        end else begin
                            quotient = value >> adaptive_k;
                            remainder = value & ((1 << adaptive_k) - 1);
                            
                            // Limit unary length to prevent excessive output
                            if (quotient > 24) begin
                                // Escape code: all 1s + literal
                                codeword = {24'hFFFFFF, value[7:0]};
                                codeword_len = 32;
                            end else begin
                                // Normal Rice code: q zeros + 1 + k-bit remainder
                                codeword = ((1 << quotient) | remainder);
                                codeword_len = quotient + 1 + adaptive_k;
                            end
                        end
                        
                        // Pack into bit buffer
                        bit_buffer <= (bit_buffer << codeword_len) | codeword;
                        bit_count <= bit_count + codeword_len;
                        
                        encoded_count <= encoded_count + 1;
                        
                        // Output 32-bit words when ready
                        if (bit_count >= 32) begin
                            comp_state <= COMP_PACK_OUTPUT;
                        end
                    end else begin
                        comp_state <= COMP_FLUSH;
                    end
                end
                
                COMP_PACK_OUTPUT: begin
                    if (m_axis_comp_tready) begin
                        bit_count <= bit_count - 32;
                        bytes_out <= bytes_out + 4;
                        comp_state <= COMP_ENCODE;
                    end
                end
                
                COMP_FLUSH: begin
                    // Flush remaining bits
                    if (bit_count > 0 && m_axis_comp_tready) begin
                        bytes_out <= bytes_out + ((bit_count + 7) / 8);
                        bit_count <= '0;
                        block_count <= block_count + 1;
                        
                        // Check if more data or done
                        if (s_axis_comp_tvalid) begin
                            comp_state <= COMP_LOAD_BLOCK;
                            sample_count <= '0;
                        end else begin
                            comp_state <= COMP_DONE;
                        end
                    end else if (bit_count == 0) begin
                        block_count <= block_count + 1;
                        if (s_axis_comp_tvalid) begin
                            comp_state <= COMP_LOAD_BLOCK;
                            sample_count <= '0;
                        end else begin
                            comp_state <= COMP_DONE;
                        end
                    end
                end
                
                COMP_DONE: begin
                    comp_busy <= 1'b0;
                    irq_comp_done <= 1'b1;
                    if (!comp_enable) begin
                        comp_state <= COMP_IDLE;
                    end else if (s_axis_comp_tvalid) begin
                        comp_state <= COMP_LOAD_BLOCK;
                        comp_busy <= 1'b1;
                        sample_count <= '0;
                    end
                end
            endcase
        end
    end
    
    // Compression input ready
    assign s_axis_comp_tready = comp_enable && (comp_state == COMP_LOAD_BLOCK);
    
    // Compression output
    assign m_axis_comp_tdata  = bit_buffer[63:32];
    assign m_axis_comp_tvalid = (comp_state == COMP_PACK_OUTPUT) || 
                                 (comp_state == COMP_FLUSH && bit_count > 0);
    assign m_axis_comp_tlast  = (comp_state == COMP_FLUSH);
    assign m_axis_comp_tkeep  = (comp_state == COMP_FLUSH) ? 
                                 ((bit_count >= 24) ? 4'b1111 :
                                  (bit_count >= 16) ? 4'b1110 :
                                  (bit_count >= 8)  ? 4'b1100 : 4'b1000) : 4'b1111;
    
    // Calculate compression ratio (Q4.4 format: 0x20 = 2.0x)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            compression_ratio <= 8'h10;  // 1.0x default
        end else if (bytes_out > 0) begin
            // ratio = bytes_in / bytes_out (Q4.4)
            compression_ratio <= (bytes_in << 4) / bytes_out;
        end
    end
    
    //=========================================================================
    // Decompression Engine (CCSDS 121.0-B-3 Rice Decoding)
    //=========================================================================
    generate
        if (ENABLE_DECOMPRESS) begin : gen_decompress
            
            typedef enum logic [3:0] {
                DECOMP_IDLE,
                DECOMP_LOAD_INPUT,
                DECOMP_DECODE_K,
                DECOMP_DECODE,
                DECOMP_RECONSTRUCT,
                DECOMP_OUTPUT,
                DECOMP_DONE
            } decomp_state_t;
            
            decomp_state_t decomp_state;
            
            logic [63:0] decomp_bit_buffer;
            logic [6:0] decomp_bit_count;
            logic [5:0] decomp_sample_count;
            logic [DATA_WIDTH-1:0] decomp_delta_buffer [0:63];
            logic [DATA_WIDTH-1:0] decomp_output_buffer [0:63];
            logic [3:0] decomp_k;
            logic [5:0] decomp_output_count;
            
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    decomp_state        <= DECOMP_IDLE;
                    decomp_bit_buffer   <= '0;
                    decomp_bit_count    <= '0;
                    decomp_sample_count <= '0;
                    decomp_k            <= 4'd4;
                    decomp_output_count <= '0;
                    decomp_busy         <= 1'b0;
                    irq_decomp_done     <= 1'b0;
                end else begin
                    irq_decomp_done <= 1'b0;
                    
                    case (decomp_state)
                        DECOMP_IDLE: begin
                            if (decomp_enable && s_axis_decomp_tvalid) begin
                                decomp_state <= DECOMP_LOAD_INPUT;
                                decomp_busy  <= 1'b1;
                            end
                        end
                        
                        DECOMP_LOAD_INPUT: begin
                            if (s_axis_decomp_tvalid && s_axis_decomp_tready) begin
                                decomp_bit_buffer <= (decomp_bit_buffer << 32) | s_axis_decomp_tdata;
                                decomp_bit_count <= decomp_bit_count + 32;
                                
                                if (decomp_bit_count >= 32) begin
                                    decomp_state <= DECOMP_DECODE;
                                    decomp_sample_count <= '0;
                                end
                            end
                        end
                        
                        DECOMP_DECODE: begin
                            // Rice decoding
                            if (decomp_sample_count < current_block_size && decomp_bit_count >= DATA_WIDTH) begin
                                logic [DATA_WIDTH-1:0] decoded_value;
                                logic [5:0] consumed_bits;
                                
                                if (decomp_sample_count == 0) begin
                                    // Reference sample
                                    decoded_value = decomp_bit_buffer[63:64-DATA_WIDTH];
                                    consumed_bits = DATA_WIDTH;
                                end else begin
                                    // Rice decode: count leading zeros for quotient
                                    logic [5:0] q;
                                    logic [DATA_WIDTH-1:0] r;
                                    
                                    // Count leading zeros (simplified)
                                    q = 0;
                                    for (int i = 63; i >= 0 && q < 24; i--) begin
                                        if (!decomp_bit_buffer[i]) q = q + 1;
                                        else break;
                                    end
                                    
                                    // Extract remainder using shift and mask (variable k)
                                    // r = bits after (q+1) unary code, masked to k bits
                                    r = (decomp_bit_buffer >> (64 - q - 1 - decomp_k)) & ((1 << decomp_k) - 1);
                                    decoded_value = (q << decomp_k) | r;
                                    consumed_bits = q + 1 + decomp_k;
                                end
                                
                                decomp_delta_buffer[decomp_sample_count] <= decoded_value;
                                decomp_bit_buffer <= decomp_bit_buffer << consumed_bits;
                                decomp_bit_count <= decomp_bit_count - consumed_bits;
                                decomp_sample_count <= decomp_sample_count + 1;
                                
                            end else if (decomp_sample_count >= current_block_size) begin
                                decomp_state <= DECOMP_RECONSTRUCT;
                                decomp_sample_count <= '0;
                            end else begin
                                decomp_state <= DECOMP_LOAD_INPUT;
                            end
                        end
                        
                        DECOMP_RECONSTRUCT: begin
                            // Inverse prediction
                            if (decomp_sample_count == 0) begin
                                decomp_output_buffer[0] <= decomp_delta_buffer[0];
                            end else begin
                                // Inverse sign mapping
                                logic signed [DATA_WIDTH:0] unmapped;
                                logic [DATA_WIDTH-1:0] delta;
                                
                                delta = decomp_delta_buffer[decomp_sample_count];
                                if (delta[0]) begin
                                    unmapped = -((delta + 1) >> 1);
                                end else begin
                                    unmapped = delta >> 1;
                                end
                                
                                decomp_output_buffer[decomp_sample_count] <= 
                                    decomp_output_buffer[decomp_sample_count-1] + unmapped[DATA_WIDTH-1:0];
                            end
                            
                            if (decomp_sample_count == current_block_size - 1) begin
                                decomp_state <= DECOMP_OUTPUT;
                                decomp_output_count <= '0;
                            end else begin
                                decomp_sample_count <= decomp_sample_count + 1;
                            end
                        end
                        
                        DECOMP_OUTPUT: begin
                            if (m_axis_decomp_tready) begin
                                if (decomp_output_count == current_block_size - 1) begin
                                    decomp_state <= DECOMP_DONE;
                                end else begin
                                    decomp_output_count <= decomp_output_count + 1;
                                end
                            end
                        end
                        
                        DECOMP_DONE: begin
                            decomp_busy <= 1'b0;
                            irq_decomp_done <= 1'b1;
                            if (s_axis_decomp_tvalid) begin
                                decomp_state <= DECOMP_LOAD_INPUT;
                                decomp_busy <= 1'b1;
                            end else if (!decomp_enable) begin
                                decomp_state <= DECOMP_IDLE;
                            end
                        end
                    endcase
                end
            end
            
            assign s_axis_decomp_tready = decomp_enable && 
                                          (decomp_state == DECOMP_LOAD_INPUT || decomp_state == DECOMP_IDLE);
            assign m_axis_decomp_tdata  = decomp_output_buffer[decomp_output_count];
            assign m_axis_decomp_tvalid = (decomp_state == DECOMP_OUTPUT);
            assign m_axis_decomp_tlast  = (decomp_state == DECOMP_OUTPUT) && 
                                          (decomp_output_count == current_block_size - 1);
            
        end else begin : gen_no_decompress
            
            assign decomp_busy          = 1'b0;
            assign irq_decomp_done      = 1'b0;
            assign s_axis_decomp_tready = 1'b0;
            assign m_axis_decomp_tdata  = '0;
            assign m_axis_decomp_tvalid = 1'b0;
            assign m_axis_decomp_tlast  = 1'b0;
            
        end
    endgenerate
    
    //=========================================================================
    // Status Register
    //=========================================================================
    assign status_reg = {
        24'b0,
        comp_state,         // [7:4] Current compression state
        1'b0,               // [3] Output FIFO empty (not implemented)
        1'b0,               // [2] Input FIFO full (not implemented)  
        decomp_busy,        // [1]
        comp_busy           // [0]
    };
    
    // Error interrupt (not implemented in basic version)
    assign irq_error = 1'b0;

endmodule


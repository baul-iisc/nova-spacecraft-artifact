//============================================================================
// PhD Research: TinyML Accelerator for Spacecraft Vision Processing
// Author: Chandraboul
// Target: FPGA Implementation for GPS-Denied Navigation
//
// Description:
//   Hardware accelerator for TinyML inference optimized for spacecraft
//   vision applications including:
//   - Star tracker image processing
//   - Terrain relative navigation (TRN)
//   - Optical flow computation
//   - Landmark detection for rendezvous & docking
//
// Features:
//   - 2D Convolution Engine (3x3, 5x5 kernels)
//   - Depthwise Separable Convolution support
//   - Activation Functions (ReLU, ReLU6, Sigmoid, Tanh)
//   - Pooling Unit (Max, Average, Global)
//   - INT8 Quantized Operations (TinyML standard)
//   - Batch Normalization fusion
//   - Weight compression support
//============================================================================

`timescale 1ns / 1ps

module tinyml_accelerator #(
    parameter DATA_WIDTH     = 8,           // INT8 for TinyML
    parameter ACC_WIDTH      = 32,          // 32-bit accumulator
    parameter MAX_KERNEL     = 5,           // Max 5x5 kernel
    parameter MAX_CHANNELS   = 128,         // Max input/output channels (increased for TRN)
    parameter BUFFER_DEPTH   = 256,         // Line buffer depth (increased for higher resolution)
    parameter PE_ARRAY_SIZE  = 32           // 32 parallel processing elements (for landing missions)
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Control Interface
    input  logic                    start,
    input  logic [3:0]              operation,      // Operation type
    output logic                    done,
    output logic                    busy,
    output logic                    ready,
    
    // Configuration (loaded via config bus)
    input  logic [15:0]             cfg_input_width,
    input  logic [15:0]             cfg_input_height,
    input  logic [9:0]              cfg_input_channels,
    input  logic [9:0]              cfg_output_channels,
    input  logic [2:0]              cfg_kernel_size,    // 1, 3, or 5
    input  logic [2:0]              cfg_stride,
    input  logic [2:0]              cfg_padding,
    input  logic [1:0]              cfg_pool_size,      // 1, 2, or 4
    input  logic [1:0]              cfg_activation,     // 0=none, 1=ReLU, 2=ReLU6, 3=Sigmoid
    input  logic                    cfg_depthwise,      // Depthwise convolution mode
    input  logic                    cfg_batch_norm,     // Fused batch norm
    input  logic [31:0]             cfg_scale,          // Quantization scale
    input  logic [31:0]             cfg_zero_point,     // Quantization zero point
    
    // Input Feature Map Interface
    input  logic [DATA_WIDTH-1:0]   ifmap_data,
    input  logic                    ifmap_valid,
    output logic                    ifmap_ready,
    input  logic                    ifmap_last,
    
    // Weight/Kernel Interface
    input  logic [DATA_WIDTH-1:0]   weight_data,
    input  logic                    weight_valid,
    output logic                    weight_ready,
    input  logic                    weight_last,
    
    // Bias Interface
    input  logic [ACC_WIDTH-1:0]    bias_data,
    input  logic                    bias_valid,
    output logic                    bias_ready,
    
    // Output Feature Map Interface
    output logic [DATA_WIDTH-1:0]   ofmap_data,
    output logic                    ofmap_valid,
    input  logic                    ofmap_ready,
    output logic                    ofmap_last,
    
    // Batch Norm Parameters (when enabled)
    input  logic [31:0]             bn_gamma,
    input  logic [31:0]             bn_beta,
    input  logic [31:0]             bn_mean,
    input  logic [31:0]             bn_var,
    
    // Status and Debug
    output logic [31:0]             cycle_count,
    output logic [31:0]             ops_count,          // Operations performed
    output logic [15:0]             current_row,
    output logic [15:0]             current_col,
    output logic [7:0]              error_flags
);

    //------------------------------------------------------------------------
    // Operation Codes
    //------------------------------------------------------------------------
    localparam OP_CONV2D          = 4'h0;   // Standard 2D convolution
    localparam OP_DEPTHWISE_CONV  = 4'h1;   // Depthwise convolution
    localparam OP_POINTWISE_CONV  = 4'h2;   // 1x1 convolution
    localparam OP_MAX_POOL        = 4'h3;   // Max pooling
    localparam OP_AVG_POOL        = 4'h4;   // Average pooling
    localparam OP_GLOBAL_AVG_POOL = 4'h5;   // Global average pooling
    localparam OP_FULLY_CONNECTED = 4'h6;   // Fully connected layer
    localparam OP_ADD             = 4'h7;   // Element-wise add (residual)
    localparam OP_RELU            = 4'h8;   // ReLU activation only
    localparam OP_SOFTMAX         = 4'h9;   // Softmax (for classification)
    localparam OP_BATCH_NORM      = 4'hA;   // Batch normalization
    localparam OP_UPSAMPLE        = 4'hB;   // Bilinear upsampling
    
    //------------------------------------------------------------------------
    // Activation Function Codes
    //------------------------------------------------------------------------
    localparam ACT_NONE    = 2'b00;
    localparam ACT_RELU    = 2'b01;
    localparam ACT_RELU6   = 2'b10;
    localparam ACT_SIGMOID = 2'b11;
    
    //------------------------------------------------------------------------
    // State Machine
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE,
        LOAD_WEIGHTS,
        LOAD_BIAS,
        PROCESS_INIT,
        PROCESS_CONV,
        PROCESS_POOL,
        PROCESS_FC,
        PROCESS_ACTIVATION,
        ACCUMULATE,
        QUANTIZE,
        OUTPUT,
        DONE_STATE
    } state_t;
    
    state_t state, next_state;
    
    //------------------------------------------------------------------------
    // Internal Signals
    //------------------------------------------------------------------------
    // Line buffer for sliding window
    logic [DATA_WIDTH-1:0] line_buffer [MAX_KERNEL-1:0][BUFFER_DEPTH-1:0];
    logic [$clog2(BUFFER_DEPTH)-1:0] lb_wr_ptr, lb_rd_ptr;
    
    // Kernel weight storage
    logic [DATA_WIDTH-1:0] kernel_weights [MAX_KERNEL-1:0][MAX_KERNEL-1:0][MAX_CHANNELS-1:0];
    logic [9:0] weight_ch_cnt;
    logic [2:0] weight_row_cnt, weight_col_cnt;
    
    // Bias storage
    logic [ACC_WIDTH-1:0] bias_values [MAX_CHANNELS-1:0];
    logic [9:0] bias_cnt;
    
    // Processing Element Array
    logic [DATA_WIDTH-1:0]  pe_input_a [PE_ARRAY_SIZE-1:0];
    logic [DATA_WIDTH-1:0]  pe_input_b [PE_ARRAY_SIZE-1:0];
    logic [ACC_WIDTH-1:0]   pe_output  [PE_ARRAY_SIZE-1:0];
    logic [PE_ARRAY_SIZE-1:0] pe_valid;
    
    // Accumulator array
    logic [ACC_WIDTH-1:0] accumulators [MAX_CHANNELS-1:0];
    
    // Position counters
    logic [15:0] row_cnt, col_cnt;
    logic [9:0]  in_ch_cnt, out_ch_cnt;
    logic [2:0]  k_row_cnt, k_col_cnt;
    
    // Sliding window buffer
    logic [DATA_WIDTH-1:0] window [MAX_KERNEL-1:0][MAX_KERNEL-1:0];
    
    // Pipeline registers
    logic [ACC_WIDTH-1:0] conv_result;
    logic [ACC_WIDTH-1:0] pool_result;
    logic [DATA_WIDTH-1:0] activated_result;
    logic [DATA_WIDTH-1:0] quantized_result;
    
    // Statistics
    logic [31:0] total_cycles;
    logic [31:0] total_ops;
    
    //------------------------------------------------------------------------
    // Processing Element (PE) - Multiply-Accumulate
    //------------------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < PE_ARRAY_SIZE; gi++) begin : pe_array
            tinyml_pe #(
                .DATA_WIDTH(DATA_WIDTH),
                .ACC_WIDTH(ACC_WIDTH)
            ) pe_inst (
                .clk        (clk),
                .rst_n      (rst_n),
                .clear      (state == PROCESS_INIT),
                .input_a    (pe_input_a[gi]),
                .input_b    (pe_input_b[gi]),
                .valid_in   (pe_valid[gi]),
                .acc_out    (pe_output[gi])
            );
        end
    endgenerate
    
    //------------------------------------------------------------------------
    // State Machine - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (start) begin
                    next_state = LOAD_WEIGHTS;
                end
            end
            
            LOAD_WEIGHTS: begin
                if (weight_last && weight_valid) begin
                    next_state = LOAD_BIAS;
                end
            end
            
            LOAD_BIAS: begin
                if (bias_cnt == cfg_output_channels - 1 && bias_valid) begin
                    next_state = PROCESS_INIT;
                end
            end
            
            PROCESS_INIT: begin
                case (operation)
                    OP_CONV2D, OP_DEPTHWISE_CONV, OP_POINTWISE_CONV:
                        next_state = PROCESS_CONV;
                    OP_MAX_POOL, OP_AVG_POOL, OP_GLOBAL_AVG_POOL:
                        next_state = PROCESS_POOL;
                    OP_FULLY_CONNECTED:
                        next_state = PROCESS_FC;
                    OP_RELU, OP_BATCH_NORM:
                        next_state = PROCESS_ACTIVATION;
                    default:
                        next_state = DONE_STATE;
                endcase
            end
            
            PROCESS_CONV: begin
                if (row_cnt == cfg_input_height - 1 && 
                    col_cnt == cfg_input_width - 1 &&
                    out_ch_cnt == cfg_output_channels - 1) begin
                    next_state = DONE_STATE;
                end else if (k_row_cnt == cfg_kernel_size - 1 &&
                            k_col_cnt == cfg_kernel_size - 1 &&
                            in_ch_cnt == cfg_input_channels - 1) begin
                    next_state = ACCUMULATE;
                end
            end
            
            PROCESS_POOL: begin
                if (row_cnt >= cfg_input_height && col_cnt >= cfg_input_width) begin
                    next_state = DONE_STATE;
                end
            end
            
            PROCESS_FC: begin
                if (in_ch_cnt == cfg_input_channels - 1 &&
                    out_ch_cnt == cfg_output_channels - 1) begin
                    next_state = DONE_STATE;
                end else if (in_ch_cnt == cfg_input_channels - 1) begin
                    next_state = ACCUMULATE;
                end
            end
            
            PROCESS_ACTIVATION: begin
                if (row_cnt == cfg_input_height - 1 && 
                    col_cnt == cfg_input_width - 1) begin
                    next_state = DONE_STATE;
                end
            end
            
            ACCUMULATE: begin
                next_state = QUANTIZE;
            end
            
            QUANTIZE: begin
                next_state = OUTPUT;
            end
            
            OUTPUT: begin
                if (ofmap_ready) begin
                    if (operation == OP_CONV2D || operation == OP_DEPTHWISE_CONV ||
                        operation == OP_POINTWISE_CONV) begin
                        next_state = PROCESS_CONV;
                    end else if (operation == OP_FULLY_CONNECTED) begin
                        next_state = PROCESS_FC;
                    end else begin
                        next_state = DONE_STATE;
                    end
                end
            end
            
            DONE_STATE: begin
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    //------------------------------------------------------------------------
    // Weight Loading Logic - SYNCHRONOUS reset, NO reset for weights array
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            weight_ch_cnt  <= '0;
            weight_row_cnt <= '0;
            weight_col_cnt <= '0;
        end else if (state == LOAD_WEIGHTS && weight_valid) begin
            // Update counters (BRAM write handled in separate always_ff)
            if (weight_col_cnt == cfg_kernel_size - 1) begin
                weight_col_cnt <= '0;
                if (weight_row_cnt == cfg_kernel_size - 1) begin
                    weight_row_cnt <= '0;
                    if (weight_ch_cnt < MAX_CHANNELS - 1) begin
                        weight_ch_cnt <= weight_ch_cnt + 1;
                    end
                end else begin
                    weight_row_cnt <= weight_row_cnt + 1;
                end
            end else begin
                weight_col_cnt <= weight_col_cnt + 1;
            end
        end else if (state == IDLE) begin
            weight_ch_cnt  <= '0;
            weight_row_cnt <= '0;
            weight_col_cnt <= '0;
        end
    end
    
    // Weight RAM update - NO RESET for BRAM inference
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] kernel_weights_bram [MAX_KERNEL-1:0][MAX_KERNEL-1:0][MAX_CHANNELS-1:0];
    always_ff @(posedge clk) begin
        if (state == LOAD_WEIGHTS && weight_valid) begin
            kernel_weights_bram[weight_row_cnt][weight_col_cnt][weight_ch_cnt] <= weight_data;
        end
    end
    
    // Alias for compatibility
    always_comb begin
        for (int r = 0; r < MAX_KERNEL; r++)
            for (int c = 0; c < MAX_KERNEL; c++)
                for (int ch = 0; ch < MAX_CHANNELS; ch++)
                    kernel_weights[r][c][ch] = kernel_weights_bram[r][c][ch];
    end
    
    //------------------------------------------------------------------------
    // Bias Loading Logic - SYNCHRONOUS reset, NO reset for bias array
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            bias_cnt <= '0;
        end else if (state == LOAD_BIAS && bias_valid) begin
            // BRAM write handled in separate always_ff
            if (bias_cnt < MAX_CHANNELS - 1) begin
                bias_cnt <= bias_cnt + 1;
            end
        end else if (state == IDLE) begin
            bias_cnt <= '0;
        end
    end
    
    // Bias RAM update - NO RESET for BRAM inference
    (* ram_style = "block" *) logic [ACC_WIDTH-1:0] bias_values_bram [MAX_CHANNELS-1:0];
    always_ff @(posedge clk) begin
        if (state == LOAD_BIAS && bias_valid) begin
            bias_values_bram[bias_cnt] <= bias_data;
        end
    end
    
    // Alias for compatibility
    always_comb begin
        for (int i = 0; i < MAX_CHANNELS; i++)
            bias_values[i] = bias_values_bram[i];
    end
    
    //------------------------------------------------------------------------
    // Line Buffer Management - SYNCHRONOUS reset, NO reset for buffer
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            lb_wr_ptr <= '0;
            lb_rd_ptr <= '0;
        end else if (state == PROCESS_CONV && ifmap_valid) begin
            // Shift line buffers
            for (int r = MAX_KERNEL - 1; r > 0; r--) begin
                line_buffer[r][lb_wr_ptr] <= line_buffer[r-1][lb_wr_ptr];
            end
            line_buffer[0][lb_wr_ptr] <= ifmap_data;
            
            // Update write pointer
            if (lb_wr_ptr == cfg_input_width - 1) begin
                lb_wr_ptr <= '0;
            end else begin
                lb_wr_ptr <= lb_wr_ptr + 1;
            end
        end else if (state == IDLE) begin
            lb_wr_ptr <= '0;
            lb_rd_ptr <= '0;
        end
    end
    
    //------------------------------------------------------------------------
    // Sliding Window Extraction - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int r = 0; r < MAX_KERNEL; r++) begin
                for (int c = 0; c < MAX_KERNEL; c++) begin
                    window[r][c] <= '0;
                end
            end
        end else if (state == PROCESS_CONV) begin
            // Extract window from line buffers
            for (int r = 0; r < MAX_KERNEL; r++) begin
                for (int c = 0; c < MAX_KERNEL; c++) begin
                    logic [$clog2(BUFFER_DEPTH)-1:0] col_addr;
                    col_addr = lb_rd_ptr + c;
                    if (col_addr >= cfg_input_width) begin
                        col_addr = col_addr - cfg_input_width;
                    end
                    window[r][c] <= line_buffer[r][col_addr];
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Convolution Computation - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    logic [ACC_WIDTH-1:0] conv_accumulator;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            conv_accumulator <= '0;
            k_row_cnt <= '0;
            k_col_cnt <= '0;
            in_ch_cnt <= '0;
        end else if (state == PROCESS_INIT) begin
            conv_accumulator <= '0;
            k_row_cnt <= '0;
            k_col_cnt <= '0;
            in_ch_cnt <= '0;
        end else if (state == PROCESS_CONV) begin
            // MAC operation: acc += window[r][c] * kernel[r][c][ch]
            logic signed [DATA_WIDTH-1:0] pixel_val;
            logic signed [DATA_WIDTH-1:0] weight_val;
            logic signed [2*DATA_WIDTH-1:0] product;
            
            pixel_val  = $signed(window[k_row_cnt][k_col_cnt]);
            weight_val = $signed(kernel_weights[k_row_cnt][k_col_cnt][cfg_depthwise ? in_ch_cnt : out_ch_cnt]);
            product    = pixel_val * weight_val;
            
            conv_accumulator <= $signed(conv_accumulator) + $signed(product);
            
            // Update kernel position counters
            if (k_col_cnt == cfg_kernel_size - 1) begin
                k_col_cnt <= '0;
                if (k_row_cnt == cfg_kernel_size - 1) begin
                    k_row_cnt <= '0;
                    if (in_ch_cnt == cfg_input_channels - 1) begin
                        in_ch_cnt <= '0;
                    end else begin
                        in_ch_cnt <= in_ch_cnt + 1;
                    end
                end else begin
                    k_row_cnt <= k_row_cnt + 1;
                end
            end else begin
                k_col_cnt <= k_col_cnt + 1;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Pooling Computation - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] pool_max;
    logic [ACC_WIDTH-1:0]  pool_sum;
    logic [3:0]            pool_count;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pool_max   <= '0;
            pool_sum   <= '0;
            pool_count <= '0;
        end else if (state == PROCESS_INIT) begin
            pool_max   <= {DATA_WIDTH{1'b1}} >> 1;  // Most negative for signed
            pool_sum   <= '0;
            pool_count <= '0;
        end else if (state == PROCESS_POOL && ifmap_valid) begin
            // Max pooling
            if ($signed(ifmap_data) > $signed(pool_max)) begin
                pool_max <= ifmap_data;
            end
            
            // Average pooling accumulation
            pool_sum   <= pool_sum + $signed(ifmap_data);
            pool_count <= pool_count + 1;
        end
    end
    
    //------------------------------------------------------------------------
    // Activation Function Unit
    //------------------------------------------------------------------------
    logic [ACC_WIDTH-1:0] pre_activation;
    logic [DATA_WIDTH-1:0] post_activation;
    
    // ReLU: max(0, x)
    // ReLU6: min(max(0, x), 6)
    // Sigmoid: 1/(1+exp(-x)) - approximated with LUT
    
    always_comb begin
        post_activation = '0;
        
        case (cfg_activation)
            ACT_NONE: begin
                post_activation = pre_activation[DATA_WIDTH-1:0];
            end
            
            ACT_RELU: begin
                if ($signed(pre_activation) < 0) begin
                    post_activation = '0;
                end else begin
                    post_activation = pre_activation[DATA_WIDTH-1:0];
                end
            end
            
            ACT_RELU6: begin
                if ($signed(pre_activation) < 0) begin
                    post_activation = '0;
                end else if ($signed(pre_activation) > 32'd6) begin
                    post_activation = 8'd6;  // Scaled for INT8
                end else begin
                    post_activation = pre_activation[DATA_WIDTH-1:0];
                end
            end
            
            ACT_SIGMOID: begin
                // Piecewise linear approximation of sigmoid
                // sigmoid(x) ≈ 0 if x < -4, 1 if x > 4, linear in between
                if ($signed(pre_activation) < -32'sd4) begin
                    post_activation = '0;
                end else if ($signed(pre_activation) > 32'sd4) begin
                    post_activation = {DATA_WIDTH{1'b1}};  // Max value
                end else begin
                    // Linear approximation: 0.5 + x/8
                    post_activation = (pre_activation >>> 3) + (8'd1 << (DATA_WIDTH-1));
                end
            end
            
            default: begin
                post_activation = pre_activation[DATA_WIDTH-1:0];
            end
        endcase
    end
    
    //------------------------------------------------------------------------
    // Quantization Unit (Requantization for next layer)
    //------------------------------------------------------------------------
    logic [ACC_WIDTH-1:0] scaled_result;
    logic [DATA_WIDTH-1:0] quant_result;
    
    always_comb begin
        // Requantization: out = (acc * scale) >> shift + zero_point
        // Simplified: out = ((acc - zero_point_in) * scale >> 31) + zero_point_out
        scaled_result = (conv_accumulator * cfg_scale[15:0]) >>> 16;
        
        // Clamp to INT8 range [-128, 127]
        if ($signed(scaled_result) < -128) begin
            quant_result = -8'sd128;
        end else if ($signed(scaled_result) > 127) begin
            quant_result = 8'sd127;
        end else begin
            quant_result = scaled_result[DATA_WIDTH-1:0];
        end
    end
    
    //------------------------------------------------------------------------
    // Batch Normalization (Fused with convolution)
    //------------------------------------------------------------------------
    logic [ACC_WIDTH-1:0] bn_result;
    
    always_comb begin
        if (cfg_batch_norm) begin
            // BN: y = gamma * (x - mean) / sqrt(var + eps) + beta
            // Fused: y = gamma/sqrt(var) * x + (beta - gamma*mean/sqrt(var))
            // Approximated for fixed-point
            logic [ACC_WIDTH-1:0] normalized;
            normalized = ($signed(conv_accumulator) - $signed(bn_mean)) * $signed(bn_gamma[15:0]);
            bn_result = (normalized >>> 16) + $signed(bn_beta);
        end else begin
            bn_result = conv_accumulator + bias_values[out_ch_cnt];
        end
    end
    
    //------------------------------------------------------------------------
    // Position Counter Management - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            row_cnt    <= '0;
            col_cnt    <= '0;
            out_ch_cnt <= '0;
        end else if (state == IDLE) begin
            row_cnt    <= '0;
            col_cnt    <= '0;
            out_ch_cnt <= '0;
        end else if (state == OUTPUT && ofmap_ready) begin
            // Update output position
            if (out_ch_cnt == cfg_output_channels - 1) begin
                out_ch_cnt <= '0;
                if (col_cnt == (cfg_input_width - 1) / cfg_stride) begin
                    col_cnt <= '0;
                    if (row_cnt < (cfg_input_height - 1) / cfg_stride) begin
                        row_cnt <= row_cnt + 1;
                    end
                end else begin
                    col_cnt <= col_cnt + 1;
                end
            end else begin
                out_ch_cnt <= out_ch_cnt + 1;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Output Logic - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ofmap_data  <= '0;
            ofmap_valid <= 1'b0;
            ofmap_last  <= 1'b0;
        end else if (state == OUTPUT) begin
            pre_activation <= bn_result;
            ofmap_data     <= post_activation;
            ofmap_valid    <= 1'b1;
            ofmap_last     <= (row_cnt == (cfg_input_height - 1) / cfg_stride) &&
                             (col_cnt == (cfg_input_width - 1) / cfg_stride) &&
                             (out_ch_cnt == cfg_output_channels - 1);
        end else begin
            ofmap_valid <= 1'b0;
            ofmap_last  <= 1'b0;
        end
    end
    
    //------------------------------------------------------------------------
    // Statistics and Status - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            total_cycles <= '0;
            total_ops    <= '0;
        end else if (state == IDLE && start) begin
            total_cycles <= '0;
            total_ops    <= '0;
        end else if (state != IDLE && state != DONE_STATE) begin
            total_cycles <= total_cycles + 1;
            
            // Count MAC operations
            if (state == PROCESS_CONV) begin
                total_ops <= total_ops + 1;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Interface Signals
    //------------------------------------------------------------------------
    assign busy        = (state != IDLE);
    assign done        = (state == DONE_STATE);
    assign ready       = (state == IDLE);
    
    assign ifmap_ready  = (state == PROCESS_CONV || state == PROCESS_POOL || 
                          state == PROCESS_ACTIVATION);
    assign weight_ready = (state == LOAD_WEIGHTS);
    assign bias_ready   = (state == LOAD_BIAS);
    
    assign cycle_count  = total_cycles;
    assign ops_count    = total_ops;
    assign current_row  = row_cnt;
    assign current_col  = col_cnt;
    assign error_flags  = '0;  // TODO: Add overflow/underflow detection

endmodule

//============================================================================
// TinyML Processing Element (PE) - MAC Unit
//============================================================================
module tinyml_pe #(
    parameter DATA_WIDTH = 8,
    parameter ACC_WIDTH  = 32
)(
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    clear,
    input  logic [DATA_WIDTH-1:0]   input_a,
    input  logic [DATA_WIDTH-1:0]   input_b,
    input  logic                    valid_in,
    output logic [ACC_WIDTH-1:0]    acc_out
);

    logic signed [ACC_WIDTH-1:0] accumulator;
    logic signed [2*DATA_WIDTH-1:0] product;
    
    // SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            accumulator <= '0;
        end else if (clear) begin
            accumulator <= '0;
        end else if (valid_in) begin
            product = $signed(input_a) * $signed(input_b);
            accumulator <= accumulator + product;
        end
    end
    
    assign acc_out = accumulator;

endmodule



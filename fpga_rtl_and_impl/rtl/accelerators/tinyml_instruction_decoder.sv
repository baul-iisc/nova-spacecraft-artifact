//============================================================================
// PhD Research: TinyML Custom RISC-V Instruction Decoder
// Author: Chandraboul
// Target: Custom Instructions for Vision Processing Acceleration
//
// Description:
//   Decodes custom RISC-V instructions for TinyML operations.
//   Uses Custom-2 opcode space (1011011) to avoid conflicts with
//   existing matrix (Custom-3) and trigonometric instructions.
//
// Instruction Format (R-type):
//   [31:25] func7  - Operation details
//   [24:20] rs2    - Source register 2
//   [19:15] rs1    - Source register 1
//   [14:12] func3  - Operation category
//   [11:7]  rd     - Destination register
//   [6:0]   opcode - 1011011 (Custom-2)
//
// Instruction Categories:
//   func3 = 000: Layer configuration
//   func3 = 001: Convolution operations
//   func3 = 010: Pooling operations
//   func3 = 011: Activation functions
//   func3 = 100: Data movement
//   func3 = 101: Control operations
//   func3 = 110: Status/Debug
//   func3 = 111: Reserved
//============================================================================

`timescale 1ns / 1ps

module tinyml_instruction_decoder #(
    parameter XLEN = 32
)(
    input  logic                clk,
    input  logic                rst_n,
    
    // PCPI Interface (from CPU)
    input  logic                pcpi_valid,
    input  logic [31:0]         pcpi_insn,
    input  logic [31:0]         pcpi_rs1,
    input  logic [31:0]         pcpi_rs2,
    output logic                pcpi_ready,
    output logic                pcpi_wait,
    output logic [31:0]         pcpi_rd,
    
    // TinyML Accelerator Control Interface
    output logic                ml_start,
    output logic [3:0]          ml_operation,
    input  logic                ml_done,
    input  logic                ml_busy,
    input  logic                ml_ready,
    
    // TinyML Configuration Interface
    output logic [15:0]         ml_cfg_input_width,
    output logic [15:0]         ml_cfg_input_height,
    output logic [9:0]          ml_cfg_input_channels,
    output logic [9:0]          ml_cfg_output_channels,
    output logic [2:0]          ml_cfg_kernel_size,
    output logic [2:0]          ml_cfg_stride,
    output logic [2:0]          ml_cfg_padding,
    output logic [1:0]          ml_cfg_pool_size,
    output logic [1:0]          ml_cfg_activation,
    output logic                ml_cfg_depthwise,
    output logic                ml_cfg_batch_norm,
    output logic [31:0]         ml_cfg_scale,
    output logic [31:0]         ml_cfg_zero_point,
    
    // Memory Controller Interface
    output logic                mem_cfg_valid,
    output logic [3:0]          mem_cfg_cmd,
    output logic [31:0]         mem_cfg_base_addr,
    output logic [23:0]         mem_cfg_length,
    output logic                mem_cfg_compressed,
    input  logic                mem_cfg_done,
    input  logic                mem_cfg_busy,
    
    // Data Interface (for immediate data transfer)
    output logic [7:0]          weight_data,
    output logic                weight_valid,
    input  logic                weight_ready,
    output logic [31:0]         bias_data,
    output logic                bias_valid,
    input  logic                bias_ready,
    
    // Status from Accelerator
    input  logic [31:0]         ml_cycle_count,
    input  logic [31:0]         ml_ops_count,
    input  logic [7:0]          ml_error_flags,
    
    // Debug Interface
    output logic [31:0]         debug_reg
);

    //------------------------------------------------------------------------
    // Opcode Definition
    //------------------------------------------------------------------------
    localparam OPCODE_CUSTOM2 = 7'b1011011;
    
    //------------------------------------------------------------------------
    // func3 Categories
    //------------------------------------------------------------------------
    localparam F3_CONFIG      = 3'b000;
    localparam F3_CONV        = 3'b001;
    localparam F3_POOL        = 3'b010;
    localparam F3_ACTIVATION  = 3'b011;
    localparam F3_DATA        = 3'b100;
    localparam F3_CONTROL     = 3'b101;
    localparam F3_STATUS      = 3'b110;
    localparam F3_RESERVED    = 3'b111;
    
    //------------------------------------------------------------------------
    // func7 Operations (for each func3 category)
    //------------------------------------------------------------------------
    // F3_CONFIG operations
    localparam CFG_INPUT_DIM      = 7'b0000000;  // Set input width/height
    localparam CFG_CHANNELS       = 7'b0000001;  // Set in/out channels
    localparam CFG_KERNEL         = 7'b0000010;  // Set kernel/stride/padding
    localparam CFG_QUANTIZATION   = 7'b0000011;  // Set scale/zero_point
    localparam CFG_POOL           = 7'b0000100;  // Set pool size/type
    localparam CFG_ACTIVATION     = 7'b0000101;  // Set activation function
    localparam CFG_FLAGS          = 7'b0000110;  // Set depthwise/batch_norm
    
    // F3_CONV operations
    localparam CONV_START         = 7'b0000000;  // Start convolution
    localparam CONV_DEPTHWISE     = 7'b0000001;  // Start depthwise conv
    localparam CONV_POINTWISE     = 7'b0000010;  // Start 1x1 conv
    
    // F3_POOL operations
    localparam POOL_MAX           = 7'b0000000;  // Max pooling
    localparam POOL_AVG           = 7'b0000001;  // Average pooling
    localparam POOL_GLOBAL_AVG    = 7'b0000010;  // Global average pooling
    
    // F3_ACTIVATION operations
    localparam ACT_RELU           = 7'b0000000;  // ReLU
    localparam ACT_RELU6          = 7'b0000001;  // ReLU6
    localparam ACT_SIGMOID        = 7'b0000010;  // Sigmoid
    localparam ACT_SOFTMAX        = 7'b0000011;  // Softmax
    
    // F3_DATA operations
    localparam DATA_LOAD_WEIGHTS  = 7'b0000000;  // Load weights from memory
    localparam DATA_LOAD_ACT      = 7'b0000001;  // Load activations
    localparam DATA_STORE_ACT     = 7'b0000010;  // Store activations
    localparam DATA_PREFETCH_W    = 7'b0000011;  // Prefetch weights
    localparam DATA_PREFETCH_A    = 7'b0000100;  // Prefetch activations
    localparam DATA_SWAP_BUF      = 7'b0000101;  // Swap double buffers
    localparam DATA_WRITE_WEIGHT  = 7'b0000110;  // Write single weight
    localparam DATA_WRITE_BIAS    = 7'b0000111;  // Write bias
    
    // F3_CONTROL operations
    localparam CTRL_RUN_LAYER     = 7'b0000000;  // Run complete layer
    localparam CTRL_WAIT          = 7'b0000001;  // Wait for completion
    localparam CTRL_ABORT         = 7'b0000010;  // Abort current operation
    localparam CTRL_RESET         = 7'b0000011;  // Reset accelerator
    
    // F3_STATUS operations
    localparam STAT_CYCLES        = 7'b0000000;  // Read cycle count
    localparam STAT_OPS           = 7'b0000001;  // Read ops count
    localparam STAT_FLAGS         = 7'b0000010;  // Read error flags
    localparam STAT_BUSY          = 7'b0000011;  // Read busy status
    
    //------------------------------------------------------------------------
    // ML Operation Codes (to accelerator)
    //------------------------------------------------------------------------
    localparam ML_OP_CONV2D          = 4'h0;
    localparam ML_OP_DEPTHWISE_CONV  = 4'h1;
    localparam ML_OP_POINTWISE_CONV  = 4'h2;
    localparam ML_OP_MAX_POOL        = 4'h3;
    localparam ML_OP_AVG_POOL        = 4'h4;
    localparam ML_OP_GLOBAL_AVG_POOL = 4'h5;
    localparam ML_OP_FULLY_CONNECTED = 4'h6;
    localparam ML_OP_ADD             = 4'h7;
    localparam ML_OP_RELU            = 4'h8;
    localparam ML_OP_SOFTMAX         = 4'h9;
    localparam ML_OP_BATCH_NORM      = 4'hA;
    
    //------------------------------------------------------------------------
    // Instruction Field Extraction
    //------------------------------------------------------------------------
    logic [6:0]  opcode;
    logic [4:0]  rd;
    logic [2:0]  func3;
    logic [4:0]  rs1;
    logic [4:0]  rs2;
    logic [6:0]  func7;
    
    assign opcode = pcpi_insn[6:0];
    assign rd     = pcpi_insn[11:7];
    assign func3  = pcpi_insn[14:12];
    assign rs1    = pcpi_insn[19:15];
    assign rs2    = pcpi_insn[24:20];
    assign func7  = pcpi_insn[31:25];
    
    logic is_ml_insn;
    assign is_ml_insn = (opcode == OPCODE_CUSTOM2);
    
    //------------------------------------------------------------------------
    // State Machine
    //------------------------------------------------------------------------
    typedef enum logic [2:0] {
        IDLE,
        DECODE,
        EXECUTE,
        WAIT_DONE,
        WRITEBACK
    } state_t;
    
    state_t state, next_state;
    
    // Registered instruction fields
    logic [6:0]  func7_reg;
    logic [2:0]  func3_reg;
    logic [4:0]  rd_reg;
    logic [31:0] rs1_reg, rs2_reg;
    logic [31:0] result_reg;
    
    //------------------------------------------------------------------------
    // State Register
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
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
                if (pcpi_valid && is_ml_insn) begin
                    next_state = DECODE;
                end
            end
            
            DECODE: begin
                next_state = EXECUTE;
            end
            
            EXECUTE: begin
                case (func3_reg)
                    F3_CONFIG, F3_STATUS: begin
                        next_state = WRITEBACK;
                    end
                    F3_CONV, F3_POOL, F3_ACTIVATION: begin
                        next_state = WAIT_DONE;
                    end
                    F3_DATA: begin
                        if (func7_reg == DATA_WRITE_WEIGHT || func7_reg == DATA_WRITE_BIAS) begin
                            next_state = WRITEBACK;
                        end else begin
                            next_state = WAIT_DONE;
                        end
                    end
                    F3_CONTROL: begin
                        if (func7_reg == CTRL_WAIT) begin
                            next_state = WAIT_DONE;
                        end else begin
                            next_state = WRITEBACK;
                        end
                    end
                    default: next_state = WRITEBACK;
                endcase
            end
            
            WAIT_DONE: begin
                if (ml_done || mem_cfg_done) begin
                    next_state = WRITEBACK;
                end
            end
            
            WRITEBACK: begin
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    //------------------------------------------------------------------------
    // Register Capture
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            func7_reg <= '0;
            func3_reg <= '0;
            rd_reg    <= '0;
            rs1_reg   <= '0;
            rs2_reg   <= '0;
        end else if (state == IDLE && pcpi_valid && is_ml_insn) begin
            func7_reg <= func7;
            func3_reg <= func3;
            rd_reg    <= rd;
            rs1_reg   <= pcpi_rs1;
            rs2_reg   <= pcpi_rs2;
        end
    end
    
    //------------------------------------------------------------------------
    // Configuration Registers
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ml_cfg_input_width    <= 16'd32;
            ml_cfg_input_height   <= 16'd32;
            ml_cfg_input_channels <= 10'd1;
            ml_cfg_output_channels <= 10'd1;
            ml_cfg_kernel_size    <= 3'd3;
            ml_cfg_stride         <= 3'd1;
            ml_cfg_padding        <= 3'd1;
            ml_cfg_pool_size      <= 2'd2;
            ml_cfg_activation     <= 2'd0;
            ml_cfg_depthwise      <= 1'b0;
            ml_cfg_batch_norm     <= 1'b0;
            ml_cfg_scale          <= 32'h0001_0000;  // 1.0 in Q16.16
            ml_cfg_zero_point     <= 32'd0;
        end else if (state == EXECUTE && func3_reg == F3_CONFIG) begin
            case (func7_reg)
                CFG_INPUT_DIM: begin
                    ml_cfg_input_width  <= rs1_reg[15:0];
                    ml_cfg_input_height <= rs2_reg[15:0];
                end
                CFG_CHANNELS: begin
                    ml_cfg_input_channels  <= rs1_reg[9:0];
                    ml_cfg_output_channels <= rs2_reg[9:0];
                end
                CFG_KERNEL: begin
                    ml_cfg_kernel_size <= rs1_reg[2:0];
                    ml_cfg_stride      <= rs1_reg[5:3];
                    ml_cfg_padding     <= rs1_reg[8:6];
                end
                CFG_QUANTIZATION: begin
                    ml_cfg_scale      <= rs1_reg;
                    ml_cfg_zero_point <= rs2_reg;
                end
                CFG_POOL: begin
                    ml_cfg_pool_size <= rs1_reg[1:0];
                end
                CFG_ACTIVATION: begin
                    ml_cfg_activation <= rs1_reg[1:0];
                end
                CFG_FLAGS: begin
                    ml_cfg_depthwise  <= rs1_reg[0];
                    ml_cfg_batch_norm <= rs1_reg[1];
                end
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // Accelerator Start Signal
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ml_start     <= 1'b0;
            ml_operation <= 4'h0;
        end else if (state == EXECUTE) begin
            case (func3_reg)
                F3_CONV: begin
                    ml_start <= 1'b1;
                    case (func7_reg)
                        CONV_START:     ml_operation <= ML_OP_CONV2D;
                        CONV_DEPTHWISE: ml_operation <= ML_OP_DEPTHWISE_CONV;
                        CONV_POINTWISE: ml_operation <= ML_OP_POINTWISE_CONV;
                        default:        ml_operation <= ML_OP_CONV2D;
                    endcase
                end
                F3_POOL: begin
                    ml_start <= 1'b1;
                    case (func7_reg)
                        POOL_MAX:        ml_operation <= ML_OP_MAX_POOL;
                        POOL_AVG:        ml_operation <= ML_OP_AVG_POOL;
                        POOL_GLOBAL_AVG: ml_operation <= ML_OP_GLOBAL_AVG_POOL;
                        default:         ml_operation <= ML_OP_MAX_POOL;
                    endcase
                end
                F3_ACTIVATION: begin
                    ml_start <= 1'b1;
                    case (func7_reg)
                        ACT_RELU:    ml_operation <= ML_OP_RELU;
                        ACT_SOFTMAX: ml_operation <= ML_OP_SOFTMAX;
                        default:     ml_operation <= ML_OP_RELU;
                    endcase
                end
                default: begin
                    ml_start <= 1'b0;
                end
            endcase
        end else begin
            ml_start <= 1'b0;
        end
    end
    
    //------------------------------------------------------------------------
    // Memory Controller Interface
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_cfg_valid      <= 1'b0;
            mem_cfg_cmd        <= 4'h0;
            mem_cfg_base_addr  <= '0;
            mem_cfg_length     <= '0;
            mem_cfg_compressed <= 1'b0;
        end else if (state == EXECUTE && func3_reg == F3_DATA) begin
            case (func7_reg)
                DATA_LOAD_WEIGHTS: begin
                    mem_cfg_valid     <= 1'b1;
                    mem_cfg_cmd       <= 4'h1;
                    mem_cfg_base_addr <= rs1_reg;
                    mem_cfg_length    <= rs2_reg[23:0];
                    mem_cfg_compressed <= 1'b0;
                end
                DATA_LOAD_ACT: begin
                    mem_cfg_valid     <= 1'b1;
                    mem_cfg_cmd       <= 4'h2;
                    mem_cfg_base_addr <= rs1_reg;
                    mem_cfg_length    <= rs2_reg[23:0];
                end
                DATA_STORE_ACT: begin
                    mem_cfg_valid     <= 1'b1;
                    mem_cfg_cmd       <= 4'h3;
                    mem_cfg_base_addr <= rs1_reg;
                    mem_cfg_length    <= rs2_reg[23:0];
                end
                DATA_PREFETCH_W: begin
                    mem_cfg_valid     <= 1'b1;
                    mem_cfg_cmd       <= 4'h4;
                    mem_cfg_base_addr <= rs1_reg;
                    mem_cfg_length    <= rs2_reg[23:0];
                end
                DATA_PREFETCH_A: begin
                    mem_cfg_valid     <= 1'b1;
                    mem_cfg_cmd       <= 4'h5;
                    mem_cfg_base_addr <= rs1_reg;
                    mem_cfg_length    <= rs2_reg[23:0];
                end
                DATA_SWAP_BUF: begin
                    mem_cfg_valid <= 1'b1;
                    mem_cfg_cmd   <= rs1_reg[0] ? 4'h7 : 4'h6;
                end
                default: begin
                    mem_cfg_valid <= 1'b0;
                end
            endcase
        end else begin
            mem_cfg_valid <= 1'b0;
        end
    end
    
    //------------------------------------------------------------------------
    // Direct Data Write Interface
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_data  <= '0;
            weight_valid <= 1'b0;
            bias_data    <= '0;
            bias_valid   <= 1'b0;
        end else if (state == EXECUTE && func3_reg == F3_DATA) begin
            case (func7_reg)
                DATA_WRITE_WEIGHT: begin
                    weight_data  <= rs1_reg[7:0];
                    weight_valid <= 1'b1;
                    bias_valid   <= 1'b0;
                end
                DATA_WRITE_BIAS: begin
                    bias_data    <= rs1_reg;
                    bias_valid   <= 1'b1;
                    weight_valid <= 1'b0;
                end
                default: begin
                    weight_valid <= 1'b0;
                    bias_valid   <= 1'b0;
                end
            endcase
        end else begin
            weight_valid <= 1'b0;
            bias_valid   <= 1'b0;
        end
    end
    
    //------------------------------------------------------------------------
    // Result Multiplexer
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result_reg <= '0;
        end else if (state == EXECUTE || state == WAIT_DONE) begin
            case (func3_reg)
                F3_STATUS: begin
                    case (func7_reg)
                        STAT_CYCLES: result_reg <= ml_cycle_count;
                        STAT_OPS:    result_reg <= ml_ops_count;
                        STAT_FLAGS:  result_reg <= {24'b0, ml_error_flags};
                        STAT_BUSY:   result_reg <= {30'b0, ml_busy, mem_cfg_busy};
                        default:     result_reg <= '0;
                    endcase
                end
                F3_CONFIG: begin
                    // Return configuration confirmation
                    result_reg <= 32'h0000_0001;  // Success
                end
                default: begin
                    result_reg <= ml_cycle_count;  // Return cycles for operations
                end
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // PCPI Response
    //------------------------------------------------------------------------
    assign pcpi_ready = (state == WRITEBACK);
    assign pcpi_wait  = is_ml_insn && (state != IDLE) && (state != WRITEBACK);
    assign pcpi_rd    = result_reg;
    
    //------------------------------------------------------------------------
    // Debug Register
    //------------------------------------------------------------------------
    assign debug_reg = {8'b0, func7_reg, func3_reg, 3'b0, state, 8'b0};

endmodule



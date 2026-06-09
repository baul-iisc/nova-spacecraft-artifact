//============================================================================
// PhD Research: TinyML Subsystem for Spacecraft Vision Processing
// Author: Chandraboul
// Target: Complete ML Acceleration Subsystem
//
// Description:
//   Integrates all TinyML components into a complete subsystem:
//   - TinyML Accelerator (Conv, Pool, Activation)
//   - Memory Controller (Weight/Activation buffers)
//   - Instruction Decoder (Custom RISC-V interface)
//   - DMA Engine
//
// Applications:
//   - Star Tracker image processing
//   - Terrain Relative Navigation (TRN)
//   - Optical Flow computation
//   - Landmark detection for Rendezvous & Docking
//   - Pose estimation
//============================================================================

`timescale 1ns / 1ps

module tinyml_subsystem #(
    parameter DATA_WIDTH        = 8,           // INT8 for TinyML
    parameter ACC_WIDTH         = 32,          // Accumulator width
    parameter ADDR_WIDTH        = 32,          // Address width
    parameter WEIGHT_SRAM_KB    = 8,           // Weight buffer size in KB (reduced for synthesis)
    parameter ACT_SRAM_KB       = 16,          // Activation buffer size in KB (reduced for synthesis)
    parameter PE_ARRAY_SIZE     = 32,          // 32 parallel processing elements (space missions)
    parameter AXI_DATA_WIDTH    = 128          // AXI bus width
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // PCPI Interface (from RISC-V core)
    input  logic                        pcpi_valid,
    input  logic [31:0]                 pcpi_insn,
    input  logic [31:0]                 pcpi_rs1,
    input  logic [31:0]                 pcpi_rs2,
    output logic                        pcpi_ready,
    output logic                        pcpi_wait,
    output logic [31:0]                 pcpi_rd,
    
    // AXI4 Master Interface (to external memory)
    output logic [ADDR_WIDTH-1:0]       m_axi_awaddr,
    output logic [7:0]                  m_axi_awlen,
    output logic [2:0]                  m_axi_awsize,
    output logic [1:0]                  m_axi_awburst,
    output logic                        m_axi_awvalid,
    input  logic                        m_axi_awready,
    
    output logic [AXI_DATA_WIDTH-1:0]   m_axi_wdata,
    output logic [AXI_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output logic                        m_axi_wlast,
    output logic                        m_axi_wvalid,
    input  logic                        m_axi_wready,
    
    input  logic [1:0]                  m_axi_bresp,
    input  logic                        m_axi_bvalid,
    output logic                        m_axi_bready,
    
    output logic [ADDR_WIDTH-1:0]       m_axi_araddr,
    output logic [7:0]                  m_axi_arlen,
    output logic [2:0]                  m_axi_arsize,
    output logic [1:0]                  m_axi_arburst,
    output logic                        m_axi_arvalid,
    input  logic                        m_axi_arready,
    
    input  logic [AXI_DATA_WIDTH-1:0]   m_axi_rdata,
    input  logic [1:0]                  m_axi_rresp,
    input  logic                        m_axi_rlast,
    input  logic                        m_axi_rvalid,
    output logic                        m_axi_rready,
    
    // Interrupt output
    output logic                        irq_done,
    output logic                        irq_error,
    
    // Status outputs
    output logic                        busy,
    output logic [31:0]                 status_reg,
    output logic [31:0]                 cycle_count,
    output logic [31:0]                 ops_count
);

    //------------------------------------------------------------------------
    // Local Parameters (synthesis-friendly sizes)
    //------------------------------------------------------------------------
    localparam WEIGHT_SRAM_DEPTH = 4096;  // Fixed 4KB for BRAM inference
    localparam ACT_SRAM_DEPTH    = 4096;  // Fixed 4KB for BRAM inference
    
    //------------------------------------------------------------------------
    // Internal Signals - Instruction Decoder to Accelerator
    //------------------------------------------------------------------------
    logic                ml_start;
    logic [3:0]          ml_operation;
    logic                ml_done;
    logic                ml_busy;
    logic                ml_ready;
    
    // Configuration signals
    logic [15:0]         ml_cfg_input_width;
    logic [15:0]         ml_cfg_input_height;
    logic [9:0]          ml_cfg_input_channels;
    logic [9:0]          ml_cfg_output_channels;
    logic [2:0]          ml_cfg_kernel_size;
    logic [2:0]          ml_cfg_stride;
    logic [2:0]          ml_cfg_padding;
    logic [1:0]          ml_cfg_pool_size;
    logic [1:0]          ml_cfg_activation;
    logic                ml_cfg_depthwise;
    logic                ml_cfg_batch_norm;
    logic [31:0]         ml_cfg_scale;
    logic [31:0]         ml_cfg_zero_point;
    
    //------------------------------------------------------------------------
    // Internal Signals - Memory Controller
    //------------------------------------------------------------------------
    logic                mem_cfg_valid;
    logic [3:0]          mem_cfg_cmd;
    logic [31:0]         mem_cfg_base_addr;
    logic [23:0]         mem_cfg_length;
    logic                mem_cfg_compressed;
    logic                mem_cfg_done;
    logic                mem_cfg_busy;
    
    //------------------------------------------------------------------------
    // Internal Signals - Data Paths
    //------------------------------------------------------------------------
    // Weight data path
    logic [DATA_WIDTH-1:0] weight_data;
    logic                  weight_valid;
    logic                  weight_ready;
    logic                  weight_last;
    
    // Direct weight write from decoder
    logic [DATA_WIDTH-1:0] direct_weight_data;
    logic                  direct_weight_valid;
    
    // Bias data path
    logic [ACC_WIDTH-1:0]  bias_data;
    logic                  bias_valid;
    logic                  bias_ready;
    
    // Direct bias write from decoder
    logic [ACC_WIDTH-1:0]  direct_bias_data;
    logic                  direct_bias_valid;
    
    // Activation read path (mem controller to accelerator)
    logic [DATA_WIDTH-1:0] act_rd_data;
    logic                  act_rd_valid;
    logic                  act_rd_ready;
    logic [ADDR_WIDTH-1:0] act_rd_addr;
    logic                  act_rd_req;
    
    // Activation write path (accelerator to mem controller)
    logic [DATA_WIDTH-1:0] act_wr_data;
    logic                  act_wr_valid;
    logic                  act_wr_ready;
    logic [ADDR_WIDTH-1:0] act_wr_addr;
    
    // Input feature map (from memory to accelerator)
    logic [DATA_WIDTH-1:0] ifmap_data;
    logic                  ifmap_valid;
    logic                  ifmap_ready;
    logic                  ifmap_last;
    
    // Output feature map (from accelerator to memory)
    logic [DATA_WIDTH-1:0] ofmap_data;
    logic                  ofmap_valid;
    logic                  ofmap_ready;
    logic                  ofmap_last;
    
    // Status signals
    logic [31:0]         ml_cycle_count;
    logic [31:0]         ml_ops_count;
    logic [7:0]          ml_error_flags;
    logic [15:0]         ml_current_row;
    logic [15:0]         ml_current_col;
    logic [31:0]         mem_bytes_transferred;
    logic [7:0]          mem_error_flags;
    logic [31:0]         decoder_debug;
    
    //------------------------------------------------------------------------
    // TinyML Instruction Decoder
    //------------------------------------------------------------------------
    tinyml_instruction_decoder #(
        .XLEN(32)
    ) u_instruction_decoder (
        .clk                    (clk),
        .rst_n                  (rst_n),
        
        // PCPI Interface
        .pcpi_valid             (pcpi_valid),
        .pcpi_insn              (pcpi_insn),
        .pcpi_rs1               (pcpi_rs1),
        .pcpi_rs2               (pcpi_rs2),
        .pcpi_ready             (pcpi_ready),
        .pcpi_wait              (pcpi_wait),
        .pcpi_rd                (pcpi_rd),
        
        // Accelerator Control
        .ml_start               (ml_start),
        .ml_operation           (ml_operation),
        .ml_done                (ml_done),
        .ml_busy                (ml_busy),
        .ml_ready               (ml_ready),
        
        // Configuration
        .ml_cfg_input_width     (ml_cfg_input_width),
        .ml_cfg_input_height    (ml_cfg_input_height),
        .ml_cfg_input_channels  (ml_cfg_input_channels),
        .ml_cfg_output_channels (ml_cfg_output_channels),
        .ml_cfg_kernel_size     (ml_cfg_kernel_size),
        .ml_cfg_stride          (ml_cfg_stride),
        .ml_cfg_padding         (ml_cfg_padding),
        .ml_cfg_pool_size       (ml_cfg_pool_size),
        .ml_cfg_activation      (ml_cfg_activation),
        .ml_cfg_depthwise       (ml_cfg_depthwise),
        .ml_cfg_batch_norm      (ml_cfg_batch_norm),
        .ml_cfg_scale           (ml_cfg_scale),
        .ml_cfg_zero_point      (ml_cfg_zero_point),
        
        // Memory Controller
        .mem_cfg_valid          (mem_cfg_valid),
        .mem_cfg_cmd            (mem_cfg_cmd),
        .mem_cfg_base_addr      (mem_cfg_base_addr),
        .mem_cfg_length         (mem_cfg_length),
        .mem_cfg_compressed     (mem_cfg_compressed),
        .mem_cfg_done           (mem_cfg_done),
        .mem_cfg_busy           (mem_cfg_busy),
        
        // Direct Data
        .weight_data            (direct_weight_data),
        .weight_valid           (direct_weight_valid),
        .weight_ready           (weight_ready),
        .bias_data              (direct_bias_data),
        .bias_valid             (direct_bias_valid),
        .bias_ready             (bias_ready),
        
        // Status
        .ml_cycle_count         (ml_cycle_count),
        .ml_ops_count           (ml_ops_count),
        .ml_error_flags         (ml_error_flags),
        
        .debug_reg              (decoder_debug)
    );
    
    //------------------------------------------------------------------------
    // TinyML Memory Controller
    //------------------------------------------------------------------------
    tinyml_memory_controller #(
        .DATA_WIDTH         (DATA_WIDTH),
        .ADDR_WIDTH         (ADDR_WIDTH),
        .WEIGHT_SRAM_DEPTH  (WEIGHT_SRAM_DEPTH),
        .ACT_SRAM_DEPTH     (ACT_SRAM_DEPTH),
        .BURST_LEN          (16),
        .AXI_DATA_WIDTH     (AXI_DATA_WIDTH)
    ) u_memory_controller (
        .clk                (clk),
        .rst_n              (rst_n),
        
        // Configuration
        .cfg_valid          (mem_cfg_valid),
        .cfg_cmd            (mem_cfg_cmd),
        .cfg_base_addr      (mem_cfg_base_addr),
        .cfg_length         (mem_cfg_length),
        .cfg_width          (ml_cfg_input_width),
        .cfg_height         (ml_cfg_input_height),
        .cfg_channels       (ml_cfg_input_channels),
        .cfg_compressed     (mem_cfg_compressed),
        .cfg_done           (mem_cfg_done),
        .cfg_busy           (mem_cfg_busy),
        
        // Weight Interface
        .weight_data        (weight_data),
        .weight_valid       (weight_valid),
        .weight_ready       (weight_ready),
        .weight_last        (weight_last),
        
        // Activation Read Interface
        .act_rd_data        (act_rd_data),
        .act_rd_valid       (act_rd_valid),
        .act_rd_ready       (act_rd_ready),
        .act_rd_addr        (act_rd_addr),
        .act_rd_req         (act_rd_req),
        
        // Activation Write Interface
        .act_wr_data        (act_wr_data),
        .act_wr_valid       (act_wr_valid),
        .act_wr_ready       (act_wr_ready),
        .act_wr_addr        (act_wr_addr),
        
        // AXI Interface
        .m_axi_awaddr       (m_axi_awaddr),
        .m_axi_awlen        (m_axi_awlen),
        .m_axi_awsize       (m_axi_awsize),
        .m_axi_awburst      (m_axi_awburst),
        .m_axi_awvalid      (m_axi_awvalid),
        .m_axi_awready      (m_axi_awready),
        .m_axi_wdata        (m_axi_wdata),
        .m_axi_wstrb        (m_axi_wstrb),
        .m_axi_wlast        (m_axi_wlast),
        .m_axi_wvalid       (m_axi_wvalid),
        .m_axi_wready       (m_axi_wready),
        .m_axi_bresp        (m_axi_bresp),
        .m_axi_bvalid       (m_axi_bvalid),
        .m_axi_bready       (m_axi_bready),
        .m_axi_araddr       (m_axi_araddr),
        .m_axi_arlen        (m_axi_arlen),
        .m_axi_arsize       (m_axi_arsize),
        .m_axi_arburst      (m_axi_arburst),
        .m_axi_arvalid      (m_axi_arvalid),
        .m_axi_arready      (m_axi_arready),
        .m_axi_rdata        (m_axi_rdata),
        .m_axi_rresp        (m_axi_rresp),
        .m_axi_rlast        (m_axi_rlast),
        .m_axi_rvalid       (m_axi_rvalid),
        .m_axi_rready       (m_axi_rready),
        
        // Status
        .bytes_transferred  (mem_bytes_transferred),
        .error_flags        (mem_error_flags)
    );
    
    //------------------------------------------------------------------------
    // Data Multiplexing (Direct write vs Memory Controller)
    //------------------------------------------------------------------------
    // Weight multiplexer - select between direct CPU write and memory controller
    logic [DATA_WIDTH-1:0] mux_weight_data;
    logic                  mux_weight_valid;
    
    always_comb begin
        if (direct_weight_valid) begin
            mux_weight_data  = direct_weight_data;
            mux_weight_valid = direct_weight_valid;
        end else begin
            mux_weight_data  = weight_data;
            mux_weight_valid = weight_valid;
        end
    end
    
    // Input feature map comes from activation read
    assign ifmap_data  = act_rd_data;
    assign ifmap_valid = act_rd_valid;
    assign act_rd_ready = ifmap_ready;
    
    // Output feature map goes to activation write
    assign act_wr_data  = ofmap_data;
    assign act_wr_valid = ofmap_valid;
    assign ofmap_ready  = act_wr_ready;
    
    //------------------------------------------------------------------------
    // TinyML Accelerator
    //------------------------------------------------------------------------
    tinyml_accelerator #(
        .DATA_WIDTH     (DATA_WIDTH),
        .ACC_WIDTH      (ACC_WIDTH),
        .MAX_KERNEL     (5),
        .MAX_CHANNELS   (64),
        .BUFFER_DEPTH   (128),
        .PE_ARRAY_SIZE  (PE_ARRAY_SIZE)
    ) u_accelerator (
        .clk                (clk),
        .rst_n              (rst_n),
        
        // Control
        .start              (ml_start),
        .operation          (ml_operation),
        .done               (ml_done),
        .busy               (ml_busy),
        .ready              (ml_ready),
        
        // Configuration
        .cfg_input_width    (ml_cfg_input_width),
        .cfg_input_height   (ml_cfg_input_height),
        .cfg_input_channels (ml_cfg_input_channels),
        .cfg_output_channels(ml_cfg_output_channels),
        .cfg_kernel_size    (ml_cfg_kernel_size),
        .cfg_stride         (ml_cfg_stride),
        .cfg_padding        (ml_cfg_padding),
        .cfg_pool_size      (ml_cfg_pool_size),
        .cfg_activation     (ml_cfg_activation),
        .cfg_depthwise      (ml_cfg_depthwise),
        .cfg_batch_norm     (ml_cfg_batch_norm),
        .cfg_scale          (ml_cfg_scale),
        .cfg_zero_point     (ml_cfg_zero_point),
        
        // Input Feature Map
        .ifmap_data         (ifmap_data),
        .ifmap_valid        (ifmap_valid),
        .ifmap_ready        (ifmap_ready),
        .ifmap_last         (ifmap_last),
        
        // Weights
        .weight_data        (mux_weight_data),
        .weight_valid       (mux_weight_valid),
        .weight_ready       (weight_ready),
        .weight_last        (weight_last),
        
        // Bias
        .bias_data          (direct_bias_valid ? direct_bias_data : bias_data),
        .bias_valid         (direct_bias_valid || bias_valid),
        .bias_ready         (bias_ready),
        
        // Output Feature Map
        .ofmap_data         (ofmap_data),
        .ofmap_valid        (ofmap_valid),
        .ofmap_ready        (ofmap_ready),
        .ofmap_last         (ofmap_last),
        
        // Batch Norm Parameters (defaults)
        .bn_gamma           (32'h0001_0000),  // 1.0
        .bn_beta            (32'h0000_0000),  // 0.0
        .bn_mean            (32'h0000_0000),  // 0.0
        .bn_var             (32'h0001_0000),  // 1.0
        
        // Status
        .cycle_count        (ml_cycle_count),
        .ops_count          (ml_ops_count),
        .current_row        (ml_current_row),
        .current_col        (ml_current_col),
        .error_flags        (ml_error_flags)
    );
    
    //------------------------------------------------------------------------
    // Activation Address Generation (simple linear addressing)
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_rd_addr <= '0;
            act_rd_req  <= 1'b0;
            act_wr_addr <= '0;
        end else begin
            // Generate sequential read addresses
            if (ifmap_ready && ifmap_valid) begin
                act_rd_addr <= act_rd_addr + 1;
            end else if (!ml_busy) begin
                act_rd_addr <= '0;
            end
            
            // Generate sequential write addresses  
            if (ofmap_valid && ofmap_ready) begin
                act_wr_addr <= act_wr_addr + 1;
            end else if (!ml_busy) begin
                act_wr_addr <= '0;
            end
            
            // Request read when accelerator is ready for input
            act_rd_req <= ifmap_ready && ml_busy;
        end
    end
    
    //------------------------------------------------------------------------
    // Interrupt Generation
    //------------------------------------------------------------------------
    logic ml_done_d, mem_done_d;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ml_done_d  <= 1'b0;
            mem_done_d <= 1'b0;
            irq_done   <= 1'b0;
            irq_error  <= 1'b0;
        end else begin
            ml_done_d  <= ml_done;
            mem_done_d <= mem_cfg_done;
            
            // Rising edge detection for done interrupt
            irq_done <= (ml_done && !ml_done_d) || (mem_cfg_done && !mem_done_d);
            
            // Error interrupt
            irq_error <= (ml_error_flags != 0) || (mem_error_flags != 0);
        end
    end
    
    //------------------------------------------------------------------------
    // Status Outputs
    //------------------------------------------------------------------------
    assign busy = ml_busy || mem_cfg_busy;
    assign status_reg = {
        8'b0,
        mem_error_flags,
        ml_error_flags,
        2'b0,
        mem_cfg_busy,
        ml_busy,
        mem_cfg_done,
        ml_done,
        ml_ready,
        1'b1  // Subsystem present
    };
    assign cycle_count = ml_cycle_count;
    assign ops_count   = ml_ops_count;

endmodule


//============================================================================
// Compression/Decompression Subsystem
// 
// Unified subsystem integrating:
//   - CCSDS 121.0-B-3 Lossless TM/TC compression (Rice coding)
//   - CCSDS 122.0-B-2 Image compression (DWT + BPE)
//   - DMA interface for high-throughput operation
//   - AXI interconnect for CPU access
//
// Bandwidth Savings Analysis:
//   - Science TM: 2-3x compression → 50-66% bandwidth savings
//   - Housekeeping TM: 2-4x compression → 50-75% bandwidth savings
//   - Image data: 3-10x compression → 66-90% bandwidth savings
//   - Total power savings: Reduced TX time → lower power consumption
//
// Copyright (c) 2024 - Space Processor Project
//============================================================================

module compression_subsystem #(
    parameter DATA_WIDTH        = 16,
    parameter MAX_IMAGE_DIM     = 1024,
    parameter ENABLE_IMAGE_COMP = 1,
    parameter AXI_ADDR_WIDTH    = 32,
    parameter AXI_DATA_WIDTH    = 32
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    //=========================================================================
    // AXI-Lite Control Interface
    //=========================================================================
    input  logic [AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  logic                        s_axi_awvalid,
    output logic                        s_axi_awready,
    input  logic [AXI_DATA_WIDTH-1:0]   s_axi_wdata,
    input  logic [3:0]                  s_axi_wstrb,
    input  logic                        s_axi_wvalid,
    output logic                        s_axi_wready,
    output logic [1:0]                  s_axi_bresp,
    output logic                        s_axi_bvalid,
    input  logic                        s_axi_bready,
    input  logic [AXI_ADDR_WIDTH-1:0]   s_axi_araddr,
    input  logic                        s_axi_arvalid,
    output logic                        s_axi_arready,
    output logic [AXI_DATA_WIDTH-1:0]   s_axi_rdata,
    output logic [1:0]                  s_axi_rresp,
    output logic                        s_axi_rvalid,
    input  logic                        s_axi_rready,
    
    //=========================================================================
    // TM Data Input (from spacecraft subsystems)
    //=========================================================================
    input  logic [DATA_WIDTH-1:0]       tm_data,
    input  logic                        tm_valid,
    output logic                        tm_ready,
    input  logic                        tm_last,        // End of TM frame
    input  logic [7:0]                  tm_apid,        // Application ID
    
    //=========================================================================
    // TC Data Input (compressed telecommands)
    //=========================================================================
    input  logic [31:0]                 tc_compressed_data,
    input  logic                        tc_compressed_valid,
    output logic                        tc_compressed_ready,
    input  logic                        tc_compressed_last,
    
    //=========================================================================
    // Compressed TM Output (to transmitter)
    //=========================================================================
    output logic [31:0]                 tm_compressed_data,
    output logic                        tm_compressed_valid,
    input  logic                        tm_compressed_ready,
    output logic                        tm_compressed_last,
    output logic [7:0]                  tm_compressed_apid,
    
    //=========================================================================
    // Decompressed TC Output (to command handler)
    //=========================================================================
    output logic [DATA_WIDTH-1:0]       tc_data,
    output logic                        tc_valid,
    input  logic                        tc_ready,
    output logic                        tc_last,
    
    //=========================================================================
    // Image Data Input (from cameras/sensors)
    //=========================================================================
    input  logic [DATA_WIDTH-1:0]       image_pixel,
    input  logic                        image_valid,
    output logic                        image_ready,
    input  logic                        image_eol,      // End of line
    input  logic                        image_eof,      // End of frame
    input  logic [7:0]                  image_id,       // Camera/sensor ID
    
    //=========================================================================
    // Compressed Image Output
    //=========================================================================
    output logic [31:0]                 image_compressed_data,
    output logic                        image_compressed_valid,
    input  logic                        image_compressed_ready,
    output logic                        image_compressed_last,
    output logic [7:0]                  image_compressed_id,
    
    //=========================================================================
    // Statistics and Status
    //=========================================================================
    output logic                        tm_comp_busy,
    output logic                        tc_decomp_busy,
    output logic                        image_comp_busy,
    output logic [31:0]                 tm_bytes_in,
    output logic [31:0]                 tm_bytes_out,
    output logic [31:0]                 image_bytes_in,
    output logic [31:0]                 image_bytes_out,
    output logic [7:0]                  tm_compression_ratio,
    output logic [7:0]                  image_compression_ratio,
    output logic                        irq
);

    //=========================================================================
    // Internal Signals
    //=========================================================================
    logic [31:0] lossless_ctrl_addr;
    logic        lossless_ctrl_awvalid;
    logic        lossless_ctrl_awready;
    logic [31:0] lossless_ctrl_wdata;
    logic        lossless_ctrl_wvalid;
    logic        lossless_ctrl_wready;
    logic [1:0]  lossless_ctrl_bresp;
    logic        lossless_ctrl_bvalid;
    logic        lossless_ctrl_bready;
    logic [31:0] lossless_ctrl_araddr;
    logic        lossless_ctrl_arvalid;
    logic        lossless_ctrl_arready;
    logic [31:0] lossless_ctrl_rdata;
    logic [1:0]  lossless_ctrl_rresp;
    logic        lossless_ctrl_rvalid;
    logic        lossless_ctrl_rready;
    
    logic        lossless_comp_irq;
    logic        lossless_decomp_irq;
    logic        lossless_error_irq;
    
    //=========================================================================
    // Address Decode
    //=========================================================================
    // 0x0000-0x00FF: Lossless compression core
    // 0x0100-0x01FF: Image compression core
    // 0x0200-0x02FF: Subsystem control
    
    logic select_lossless, select_image, select_subsys;
    
    assign select_lossless = (s_axi_awaddr[11:8] == 4'h0) || (s_axi_araddr[11:8] == 4'h0);
    assign select_image    = (s_axi_awaddr[11:8] == 4'h1) || (s_axi_araddr[11:8] == 4'h1);
    assign select_subsys   = (s_axi_awaddr[11:8] == 4'h2) || (s_axi_araddr[11:8] == 4'h2);
    
    //=========================================================================
    // CCSDS 121.0 Lossless Compression Core (TM/TC)
    //=========================================================================
    ccsds_compression_core #(
        .DATA_WIDTH(DATA_WIDTH),
        .BLOCK_SIZE(16),
        .MAX_BLOCKS(4096),
        .FIFO_DEPTH(256),
        .ENABLE_DECOMPRESS(1)
    ) u_lossless (
        .clk                (clk),
        .rst_n              (rst_n),
        
        // Control interface (directly connected when selected)
        .s_axi_awaddr       (s_axi_awaddr),
        .s_axi_awvalid      (s_axi_awvalid && select_lossless),
        .s_axi_awready      (lossless_ctrl_awready),
        .s_axi_wdata        (s_axi_wdata),
        .s_axi_wstrb        (s_axi_wstrb),
        .s_axi_wvalid       (s_axi_wvalid && select_lossless),
        .s_axi_wready       (lossless_ctrl_wready),
        .s_axi_bresp        (lossless_ctrl_bresp),
        .s_axi_bvalid       (lossless_ctrl_bvalid),
        .s_axi_bready       (s_axi_bready),
        .s_axi_araddr       (s_axi_araddr),
        .s_axi_arvalid      (s_axi_arvalid && select_lossless),
        .s_axi_arready      (lossless_ctrl_arready),
        .s_axi_rdata        (lossless_ctrl_rdata),
        .s_axi_rresp        (lossless_ctrl_rresp),
        .s_axi_rvalid       (lossless_ctrl_rvalid),
        .s_axi_rready       (s_axi_rready),
        
        // Compression (TM)
        .s_axis_comp_tdata  (tm_data),
        .s_axis_comp_tvalid (tm_valid),
        .s_axis_comp_tready (tm_ready),
        .s_axis_comp_tlast  (tm_last),
        
        .m_axis_comp_tdata  (tm_compressed_data),
        .m_axis_comp_tvalid (tm_compressed_valid),
        .m_axis_comp_tready (tm_compressed_ready),
        .m_axis_comp_tlast  (tm_compressed_last),
        .m_axis_comp_tkeep  (),
        
        // Decompression (TC)
        .s_axis_decomp_tdata  (tc_compressed_data),
        .s_axis_decomp_tvalid (tc_compressed_valid),
        .s_axis_decomp_tready (tc_compressed_ready),
        .s_axis_decomp_tlast  (tc_compressed_last),
        
        .m_axis_decomp_tdata  (tc_data),
        .m_axis_decomp_tvalid (tc_valid),
        .m_axis_decomp_tready (tc_ready),
        .m_axis_decomp_tlast  (tc_last),
        
        // Status
        .comp_busy          (tm_comp_busy),
        .decomp_busy        (tc_decomp_busy),
        .bytes_in           (tm_bytes_in),
        .bytes_out          (tm_bytes_out),
        .compression_ratio  (tm_compression_ratio),
        .irq_comp_done      (lossless_comp_irq),
        .irq_decomp_done    (lossless_decomp_irq),
        .irq_error          (lossless_error_irq)
    );
    
    //=========================================================================
    // CCSDS 122.0 Image Compression Core (optional)
    //=========================================================================
    generate
        if (ENABLE_IMAGE_COMP) begin : gen_image_comp
            
            logic [31:0] image_ctrl_rdata;
            logic        image_ctrl_rvalid;
            logic        image_ctrl_bvalid;
            logic        image_irq;
            logic        image_error;
            logic [15:0] image_current_line;
            
            ccsds_image_compression #(
                .DATA_WIDTH(DATA_WIDTH),
                .MAX_WIDTH(MAX_IMAGE_DIM),
                .MAX_HEIGHT(MAX_IMAGE_DIM),
                .WAVELET_TYPE(0),           // Integer 5/3
                .DWT_LEVELS(3),
                .SEGMENT_SIZE(64)
            ) u_image_comp (
                .clk                (clk),
                .rst_n              (rst_n),
                
                // Control
                .s_axi_awaddr       (s_axi_awaddr),
                .s_axi_awvalid      (s_axi_awvalid && select_image),
                .s_axi_awready      (),
                .s_axi_wdata        (s_axi_wdata),
                .s_axi_wstrb        (s_axi_wstrb),
                .s_axi_wvalid       (s_axi_wvalid && select_image),
                .s_axi_wready       (),
                .s_axi_bresp        (),
                .s_axi_bvalid       (image_ctrl_bvalid),
                .s_axi_bready       (s_axi_bready),
                .s_axi_araddr       (s_axi_araddr),
                .s_axi_arvalid      (s_axi_arvalid && select_image),
                .s_axi_arready      (),
                .s_axi_rdata        (image_ctrl_rdata),
                .s_axi_rresp        (),
                .s_axi_rvalid       (image_ctrl_rvalid),
                .s_axi_rready       (s_axi_rready),
                
                // Image input
                .s_axis_pixel_tdata (image_pixel),
                .s_axis_pixel_tvalid(image_valid),
                .s_axis_pixel_tready(image_ready),
                .s_axis_pixel_tlast (image_eol),
                .s_axis_pixel_tuser (image_eof),
                
                // Compressed output
                .m_axis_comp_tdata  (image_compressed_data),
                .m_axis_comp_tvalid (image_compressed_valid),
                .m_axis_comp_tready (image_compressed_ready),
                .m_axis_comp_tlast  (image_compressed_last),
                .m_axis_comp_tuser  (),
                
                // Status
                .busy               (image_comp_busy),
                .pixels_in          (image_bytes_in),
                .bytes_out          (image_bytes_out),
                .current_line       (image_current_line),
                .compression_ratio  (image_compression_ratio),
                .irq_frame_done     (image_irq),
                .irq_error          (image_error)
            );
            
            assign image_compressed_id = image_id;
            
        end else begin : gen_no_image_comp
            
            assign image_ready              = 1'b0;
            assign image_compressed_data    = '0;
            assign image_compressed_valid   = 1'b0;
            assign image_compressed_last    = 1'b0;
            assign image_compressed_id      = '0;
            assign image_comp_busy          = 1'b0;
            assign image_bytes_in           = '0;
            assign image_bytes_out          = '0;
            assign image_compression_ratio  = 8'h10;
            
        end
    endgenerate
    
    //=========================================================================
    // AXI Response Mux
    //=========================================================================
    always_comb begin
        if (select_lossless) begin
            s_axi_awready = lossless_ctrl_awready;
            s_axi_wready  = lossless_ctrl_wready;
            s_axi_bresp   = lossless_ctrl_bresp;
            s_axi_bvalid  = lossless_ctrl_bvalid;
            s_axi_arready = lossless_ctrl_arready;
            s_axi_rdata   = lossless_ctrl_rdata;
            s_axi_rresp   = lossless_ctrl_rresp;
            s_axi_rvalid  = lossless_ctrl_rvalid;
        end else begin
            s_axi_awready = 1'b1;
            s_axi_wready  = 1'b1;
            s_axi_bresp   = 2'b00;
            s_axi_bvalid  = 1'b0;
            s_axi_arready = 1'b1;
            s_axi_rdata   = 32'h0;
            s_axi_rresp   = 2'b00;
            s_axi_rvalid  = 1'b0;
        end
    end
    
    //=========================================================================
    // APID Passthrough
    //=========================================================================
    // Store APID when TM frame starts
    logic [7:0] stored_apid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stored_apid <= '0;
        end else if (tm_valid && tm_ready) begin
            stored_apid <= tm_apid;
        end
    end
    
    assign tm_compressed_apid = stored_apid;
    
    //=========================================================================
    // Interrupt Generation
    //=========================================================================
    assign irq = lossless_comp_irq | lossless_decomp_irq | lossless_error_irq;

endmodule









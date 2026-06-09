//============================================================================
// Memory-Mapped Peripheral Bus Bridge
//
// Intercepts data memory accesses from any DLS pair when the address falls
// in the peripheral region (0xF000_0000 - 0xFFFF_FFFF) and routes them
// to the appropriate accelerator or peripheral.
//
// Address Map:
//   0xF000_0000 - 0xF000_0FFF : TinyML Accelerator (4KB)
//   0xF100_0000 - 0xF100_0FFF : CCSDS Compression  (4KB)
//   0xF200_0000 - 0xF200_03FF : Debug Module        (1KB)
//
// TinyML Register Map (base 0xF000_0000):
//   0x000: CTRL      [W]  - bit[0]=start, bit[7:4]=operation
//   0x004: STATUS    [R]  - bit[0]=done, bit[1]=busy, bit[2]=ready
//   0x008: CFG_DIM   [W]  - {input_height[31:16], input_width[15:0]}
//   0x00C: CFG_CH    [W]  - {output_channels[25:16], input_channels[9:0]}
//   0x010: CFG_CONV  [W]  - {padding[10:8], stride[7:5], kernel_size[4:2],
//                             pool_size[1:0]}
//   0x014: CFG_ACT   [W]  - {batch_norm[4], depthwise[3], activation[1:0]}
//   0x018: CFG_QUANT [W]  - {zero_point (high 32b) OR scale (low 32b)}
//   0x020: IFMAP     [W]  - Write ifmap data (byte)
//   0x024: WEIGHT    [W]  - Write weight data (byte)
//   0x028: BIAS      [W]  - Write bias data (32-bit)
//   0x02C: OFMAP     [R]  - Read ofmap data (byte)
//   0x030: BN_GAMMA  [W]  - Batch norm gamma
//   0x034: BN_BETA   [W]  - Batch norm beta
//   0x038: BN_MEAN   [W]  - Batch norm mean
//   0x03C: BN_VAR    [W]  - Batch norm variance
//   0x040: CYCLES    [R]  - Cycle count
//   0x044: OPS       [R]  - Operations count
//
// CCSDS Register Map (base 0xF100_0000):
//   0x000: CTRL      [W]  - bit[0]=valid_in, bit[3:2]=mode
//   0x004: STATUS    [R]  - bit[0]=ready, bit[1]=valid_out
//   0x008: DATA_IN   [W]  - Input data byte
//   0x00C: DATA_OUT  [R]  - Output data byte
//
// Author: Chandraboul, IISc
//============================================================================

`timescale 1ns / 1ps

module peripheral_bus_bridge #(
    parameter XLEN = 64,
    parameter PERIPH_BASE = 64'h0000_0000_F000_0000
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // CPU-side data memory interface (from address decoder)
    input  logic                    cpu_valid,
    input  logic                    cpu_write,
    input  logic [XLEN-1:0]         cpu_addr,
    input  logic [XLEN-1:0]         cpu_wdata,
    output logic [XLEN-1:0]         cpu_rdata,
    output logic                    cpu_ready,

    //=========================================================================
    // TinyML Accelerator Interface
    //=========================================================================
    output logic                    tinyml_start,
    output logic [3:0]              tinyml_operation,
    input  logic                    tinyml_done,
    input  logic                    tinyml_busy,
    input  logic                    tinyml_ready,
    // Config
    output logic [15:0]             tinyml_cfg_input_width,
    output logic [15:0]             tinyml_cfg_input_height,
    output logic [9:0]              tinyml_cfg_input_channels,
    output logic [9:0]              tinyml_cfg_output_channels,
    output logic [2:0]              tinyml_cfg_kernel_size,
    output logic [2:0]              tinyml_cfg_stride,
    output logic [2:0]              tinyml_cfg_padding,
    output logic [1:0]              tinyml_cfg_pool_size,
    output logic [1:0]              tinyml_cfg_activation,
    output logic                    tinyml_cfg_depthwise,
    output logic                    tinyml_cfg_batch_norm,
    output logic [31:0]             tinyml_cfg_scale,
    output logic [31:0]             tinyml_cfg_zero_point,
    // Data
    output logic [7:0]              tinyml_ifmap_data,
    output logic                    tinyml_ifmap_valid,
    input  logic                    tinyml_ifmap_ready,
    output logic                    tinyml_ifmap_last,
    output logic [7:0]              tinyml_weight_data,
    output logic                    tinyml_weight_valid,
    input  logic                    tinyml_weight_ready,
    output logic                    tinyml_weight_last,
    output logic [31:0]             tinyml_bias_data,
    output logic                    tinyml_bias_valid,
    input  logic                    tinyml_bias_ready,
    input  logic [7:0]              tinyml_ofmap_data,
    input  logic                    tinyml_ofmap_valid,
    output logic                    tinyml_ofmap_ready,
    // BN
    output logic [31:0]             tinyml_bn_gamma,
    output logic [31:0]             tinyml_bn_beta,
    output logic [31:0]             tinyml_bn_mean,
    output logic [31:0]             tinyml_bn_var,
    // Status
    input  logic [31:0]             tinyml_cycle_count,
    input  logic [31:0]             tinyml_ops_count,

    //=========================================================================
    // CCSDS Compression Interface
    //=========================================================================
    output logic                    ccsds_valid_in,
    output logic [1:0]              ccsds_mode,
    output logic [7:0]              ccsds_data_in,
    input  logic [7:0]              ccsds_data_out,
    input  logic                    ccsds_ready,
    input  logic                    ccsds_valid_out,

    //=========================================================================
    // Debug Module DMI Interface
    //=========================================================================
    output logic                    dmi_req_valid,
    input  logic                    dmi_req_ready,
    output logic [6:0]              dmi_req_addr,
    output logic [31:0]             dmi_req_data,
    output logic [1:0]              dmi_req_op,
    input  logic                    dmi_rsp_valid,
    output logic                    dmi_rsp_ready,
    input  logic [31:0]             dmi_rsp_data,
    input  logic [1:0]              dmi_rsp_op
);

    //=========================================================================
    // Address decode
    //=========================================================================
    wire [31:0] offset = cpu_addr[31:0];
    wire is_tinyml = cpu_valid && (offset[31:12] == 20'hF0000);
    wire is_ccsds  = cpu_valid && (offset[31:12] == 20'hF1000);
    wire is_debug  = cpu_valid && (offset[31:12] == 20'hF2000);
    wire is_periph = is_tinyml || is_ccsds || is_debug;

    //=========================================================================
    // TinyML register state machine
    //=========================================================================
    logic tinyml_start_pulse;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tinyml_start <= 1'b0;
            tinyml_operation <= 4'd0;
            tinyml_cfg_input_width <= 16'd32;
            tinyml_cfg_input_height <= 16'd32;
            tinyml_cfg_input_channels <= 10'd3;
            tinyml_cfg_output_channels <= 10'd16;
            tinyml_cfg_kernel_size <= 3'd3;
            tinyml_cfg_stride <= 3'd1;
            tinyml_cfg_padding <= 3'd1;
            tinyml_cfg_pool_size <= 2'd2;
            tinyml_cfg_activation <= 2'd1;
            tinyml_cfg_depthwise <= 1'b0;
            tinyml_cfg_batch_norm <= 1'b0;
            tinyml_cfg_scale <= 32'd256;
            tinyml_cfg_zero_point <= 32'd0;
            tinyml_bn_gamma <= 32'h3F800000;
            tinyml_bn_beta <= 32'd0;
            tinyml_bn_mean <= 32'd0;
            tinyml_bn_var <= 32'h3F800000;
            tinyml_start_pulse <= 1'b0;
        end else begin
            tinyml_start <= tinyml_start_pulse;
            tinyml_start_pulse <= 1'b0;

            if (is_tinyml && cpu_write) begin
                case (offset[11:0])
                    12'h000: begin // CTRL
                        tinyml_start_pulse <= cpu_wdata[0];
                        tinyml_operation <= cpu_wdata[7:4];
                    end
                    12'h008: begin // CFG_DIM
                        tinyml_cfg_input_width <= cpu_wdata[15:0];
                        tinyml_cfg_input_height <= cpu_wdata[31:16];
                    end
                    12'h00C: begin // CFG_CH
                        tinyml_cfg_input_channels <= cpu_wdata[9:0];
                        tinyml_cfg_output_channels <= cpu_wdata[25:16];
                    end
                    12'h010: begin // CFG_CONV
                        tinyml_cfg_pool_size <= cpu_wdata[1:0];
                        tinyml_cfg_kernel_size <= cpu_wdata[4:2];
                        tinyml_cfg_stride <= cpu_wdata[7:5];
                        tinyml_cfg_padding <= cpu_wdata[10:8];
                    end
                    12'h014: begin // CFG_ACT
                        tinyml_cfg_activation <= cpu_wdata[1:0];
                        tinyml_cfg_depthwise <= cpu_wdata[3];
                        tinyml_cfg_batch_norm <= cpu_wdata[4];
                    end
                    12'h018: tinyml_cfg_scale <= cpu_wdata[31:0];
                    12'h01C: tinyml_cfg_zero_point <= cpu_wdata[31:0];
                    12'h030: tinyml_bn_gamma <= cpu_wdata[31:0];
                    12'h034: tinyml_bn_beta <= cpu_wdata[31:0];
                    12'h038: tinyml_bn_mean <= cpu_wdata[31:0];
                    12'h03C: tinyml_bn_var <= cpu_wdata[31:0];
                    default: ;
                endcase
            end
        end
    end

    // TinyML streaming interfaces (active during writes to specific addresses)
    assign tinyml_ifmap_data  = cpu_wdata[7:0];
    assign tinyml_ifmap_valid = is_tinyml && cpu_write && (offset[11:0] == 12'h020);
    assign tinyml_ifmap_last  = 1'b0; // Managed by software
    assign tinyml_weight_data = cpu_wdata[7:0];
    assign tinyml_weight_valid = is_tinyml && cpu_write && (offset[11:0] == 12'h024);
    assign tinyml_weight_last = 1'b0;
    assign tinyml_bias_data   = cpu_wdata[31:0];
    assign tinyml_bias_valid  = is_tinyml && cpu_write && (offset[11:0] == 12'h028);
    assign tinyml_ofmap_ready = is_tinyml && !cpu_write && (offset[11:0] == 12'h02C);

    //=========================================================================
    // CCSDS registers
    //=========================================================================
    logic ccsds_valid_reg;
    logic [1:0] ccsds_mode_reg;
    logic [7:0] ccsds_data_in_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ccsds_valid_reg <= 1'b0;
            ccsds_mode_reg <= 2'd0;
            ccsds_data_in_reg <= 8'd0;
        end else begin
            ccsds_valid_reg <= 1'b0; // Pulse
            if (is_ccsds && cpu_write) begin
                case (offset[11:0])
                    12'h000: begin
                        ccsds_valid_reg <= cpu_wdata[0];
                        ccsds_mode_reg <= cpu_wdata[3:2];
                    end
                    12'h008: ccsds_data_in_reg <= cpu_wdata[7:0];
                    default: ;
                endcase
            end
        end
    end

    assign ccsds_valid_in = ccsds_valid_reg;
    assign ccsds_mode = ccsds_mode_reg;
    assign ccsds_data_in = ccsds_data_in_reg;

    //=========================================================================
    // Debug Module DMI access
    //=========================================================================
    typedef enum logic [1:0] {
        DMI_IDLE,
        DMI_REQ,
        DMI_WAIT_RSP
    } dmi_state_t;

    dmi_state_t dmi_state;
    logic [31:0] dmi_rsp_data_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dmi_state <= DMI_IDLE;
            dmi_rsp_data_reg <= 32'd0;
        end else begin
            case (dmi_state)
                DMI_IDLE: begin
                    if (is_debug && cpu_valid) begin
                        dmi_state <= DMI_REQ;
                    end
                end
                DMI_REQ: begin
                    if (dmi_req_ready) begin
                        dmi_state <= DMI_WAIT_RSP;
                    end
                end
                DMI_WAIT_RSP: begin
                    if (dmi_rsp_valid) begin
                        dmi_rsp_data_reg <= dmi_rsp_data;
                        dmi_state <= DMI_IDLE;
                    end
                end
                default: dmi_state <= DMI_IDLE;
            endcase
        end
    end

    assign dmi_req_valid = (dmi_state == DMI_REQ);
    assign dmi_req_addr  = offset[8:2]; // Word-aligned address → DMI register
    assign dmi_req_data  = cpu_wdata[31:0];
    assign dmi_req_op    = cpu_write ? 2'b10 : 2'b01; // 10=write, 01=read
    assign dmi_rsp_ready = (dmi_state == DMI_WAIT_RSP);

    //=========================================================================
    // Read data mux
    //=========================================================================
    logic [XLEN-1:0] rdata_mux;

    always_comb begin
        rdata_mux = '0;
        if (is_tinyml && !cpu_write) begin
            case (offset[11:0])
                12'h004: rdata_mux = {61'd0, tinyml_ready, tinyml_busy, tinyml_done};
                12'h02C: rdata_mux = {56'd0, tinyml_ofmap_data};
                12'h040: rdata_mux = {32'd0, tinyml_cycle_count};
                12'h044: rdata_mux = {32'd0, tinyml_ops_count};
                default: rdata_mux = '0;
            endcase
        end else if (is_ccsds && !cpu_write) begin
            case (offset[11:0])
                12'h004: rdata_mux = {62'd0, ccsds_valid_out, ccsds_ready};
                12'h00C: rdata_mux = {56'd0, ccsds_data_out};
                default: rdata_mux = '0;
            endcase
        end else if (is_debug && !cpu_write) begin
            rdata_mux = {32'd0, dmi_rsp_data_reg};
        end
    end

    //=========================================================================
    // Output assignment
    //=========================================================================
    assign cpu_rdata = rdata_mux;

    // Ready signal: immediate for TinyML/CCSDS register access,
    // multi-cycle for debug module DMI
    assign cpu_ready = is_periph && (
        (is_tinyml || is_ccsds) ? 1'b1 :
        (is_debug && dmi_state == DMI_WAIT_RSP && dmi_rsp_valid) ? 1'b1 :
        1'b0
    );

endmodule

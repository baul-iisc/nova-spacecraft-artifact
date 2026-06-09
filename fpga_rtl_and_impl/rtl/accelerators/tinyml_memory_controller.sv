//============================================================================
// PhD Research: TinyML Memory Controller (Synthesis-Optimized)
// Author: Chandraboul
// Target: Weight and Activation Memory Management for Vision Processing
//
// Description:
//   Memory controller for TinyML accelerator - optimized for BRAM inference.
//   Uses synthesis-friendly patterns for Xilinx FPGAs.
//
// Features:
//   - BRAM-inferred weight storage
//   - BRAM-inferred activation buffers
//   - AXI4 interface to external memory
//============================================================================

`timescale 1ns / 1ps

module tinyml_memory_controller #(
    parameter DATA_WIDTH        = 8,           // INT8 data
    parameter ADDR_WIDTH        = 32,          // Memory address width
    parameter WEIGHT_SRAM_DEPTH = 4096,        // 4KB weight buffer (BRAM-friendly)
    parameter ACT_SRAM_DEPTH    = 4096,        // 4KB activation buffer (BRAM-friendly)
    parameter BURST_LEN         = 16,          // AXI burst length
    parameter AXI_DATA_WIDTH    = 128          // AXI data width
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Configuration Interface
    input  logic                        cfg_valid,
    input  logic [3:0]                  cfg_cmd,
    input  logic [ADDR_WIDTH-1:0]       cfg_base_addr,
    input  logic [23:0]                 cfg_length,
    input  logic [15:0]                 cfg_width,
    input  logic [15:0]                 cfg_height,
    input  logic [9:0]                  cfg_channels,
    input  logic                        cfg_compressed,
    output logic                        cfg_done,
    output logic                        cfg_busy,
    
    // Accelerator Weight Interface
    output logic [DATA_WIDTH-1:0]       weight_data,
    output logic                        weight_valid,
    input  logic                        weight_ready,
    output logic                        weight_last,
    
    // Accelerator Activation Read Interface
    output logic [DATA_WIDTH-1:0]       act_rd_data,
    output logic                        act_rd_valid,
    input  logic                        act_rd_ready,
    input  logic [ADDR_WIDTH-1:0]       act_rd_addr,
    input  logic                        act_rd_req,
    
    // Accelerator Activation Write Interface
    input  logic [DATA_WIDTH-1:0]       act_wr_data,
    input  logic                        act_wr_valid,
    output logic                        act_wr_ready,
    input  logic [ADDR_WIDTH-1:0]       act_wr_addr,
    
    // AXI4 Master Interface (simplified - directly tied off for synthesis)
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
    
    // Status
    output logic [31:0]                 bytes_transferred,
    output logic [7:0]                  error_flags
);

    //------------------------------------------------------------------------
    // Local parameters
    //------------------------------------------------------------------------
    localparam WEIGHT_ADDR_WIDTH = $clog2(WEIGHT_SRAM_DEPTH);
    localparam ACT_ADDR_WIDTH    = $clog2(ACT_SRAM_DEPTH);
    
    //------------------------------------------------------------------------
    // Command Codes
    //------------------------------------------------------------------------
    localparam CMD_NOP              = 4'h0;
    localparam CMD_LOAD_WEIGHTS     = 4'h1;
    localparam CMD_LOAD_ACTIVATIONS = 4'h2;
    localparam CMD_STORE_ACTIVATIONS= 4'h3;
    
    //------------------------------------------------------------------------
    // State Machine
    //------------------------------------------------------------------------
    typedef enum logic [2:0] {
        IDLE,
        LOADING,
        STREAMING,
        DONE_STATE
    } state_t;
    
    state_t state, next_state;
    
    //------------------------------------------------------------------------
    // Weight BRAM - Synthesis-friendly single-port RAM
    //------------------------------------------------------------------------
    (* ram_style = "block" *)
    logic [DATA_WIDTH-1:0] weight_mem [0:WEIGHT_SRAM_DEPTH-1];
    
    logic [WEIGHT_ADDR_WIDTH-1:0] weight_wr_addr;
    logic [WEIGHT_ADDR_WIDTH-1:0] weight_rd_addr;
    logic weight_wr_en;
    logic [DATA_WIDTH-1:0] weight_wr_data_reg;
    
    // Weight memory write port
    always_ff @(posedge clk) begin
        if (weight_wr_en) begin
            weight_mem[weight_wr_addr] <= weight_wr_data_reg;
        end
    end
    
    // Weight memory read port
    always_ff @(posedge clk) begin
        weight_data <= weight_mem[weight_rd_addr];
    end
    
    //------------------------------------------------------------------------
    // Activation BRAM - Synthesis-friendly single-port RAM
    //------------------------------------------------------------------------
    (* ram_style = "block" *)
    logic [DATA_WIDTH-1:0] act_mem [0:ACT_SRAM_DEPTH-1];
    
    logic [ACT_ADDR_WIDTH-1:0] act_wr_addr_int;
    logic [ACT_ADDR_WIDTH-1:0] act_rd_addr_int;
    logic act_wr_en;
    
    // Activation memory write port
    always_ff @(posedge clk) begin
        if (act_wr_en) begin
            act_mem[act_wr_addr_int] <= act_wr_data;
        end
    end
    
    // Activation memory read port
    always_ff @(posedge clk) begin
        act_rd_data <= act_mem[act_rd_addr_int];
    end
    
    //------------------------------------------------------------------------
    // Control registers
    //------------------------------------------------------------------------
    logic [3:0]  cmd_reg;
    logic [23:0] length_reg;
    logic [23:0] transfer_cnt;
    
    //------------------------------------------------------------------------
    // State Machine
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
                if (cfg_valid && cfg_cmd != CMD_NOP) begin
                    next_state = LOADING;
                end
            end
            LOADING: begin
                if (transfer_cnt >= length_reg) begin
                    next_state = DONE_STATE;
                end
            end
            DONE_STATE: begin
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end
    
    //------------------------------------------------------------------------
    // Command and Length Registers
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_reg    <= CMD_NOP;
            length_reg <= '0;
        end else if (state == IDLE && cfg_valid) begin
            cmd_reg    <= cfg_cmd;
            length_reg <= cfg_length;
        end
    end
    
    //------------------------------------------------------------------------
    // Transfer Counter
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            transfer_cnt <= '0;
        end else if (state == IDLE) begin
            transfer_cnt <= '0;
        end else if (state == LOADING) begin
            transfer_cnt <= transfer_cnt + 1;
        end
    end
    
    //------------------------------------------------------------------------
    // Weight Write Logic
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_wr_addr    <= '0;
            weight_wr_en      <= 1'b0;
            weight_wr_data_reg <= '0;
        end else if (state == IDLE && cfg_cmd == CMD_LOAD_WEIGHTS) begin
            weight_wr_addr <= '0;
        end else if (state == LOADING && cmd_reg == CMD_LOAD_WEIGHTS) begin
            // Simulated loading - in real design, this would come from AXI
            weight_wr_en   <= 1'b1;
            weight_wr_addr <= weight_wr_addr + 1;
            weight_wr_data_reg <= transfer_cnt[DATA_WIDTH-1:0];
        end else begin
            weight_wr_en <= 1'b0;
        end
    end
    
    //------------------------------------------------------------------------
    // Weight Read Logic (streaming to accelerator)
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_rd_addr <= '0;
            weight_valid   <= 1'b0;
            weight_last    <= 1'b0;
        end else if (state == IDLE) begin
            weight_rd_addr <= '0;
            weight_valid   <= 1'b0;
            weight_last    <= 1'b0;
        end else if (weight_ready && weight_rd_addr < length_reg[WEIGHT_ADDR_WIDTH-1:0]) begin
            weight_rd_addr <= weight_rd_addr + 1;
            weight_valid   <= 1'b1;
            weight_last    <= (weight_rd_addr == length_reg[WEIGHT_ADDR_WIDTH-1:0] - 1);
        end else begin
            weight_valid <= 1'b0;
        end
    end
    
    //------------------------------------------------------------------------
    // Activation Address Logic
    //------------------------------------------------------------------------
    assign act_rd_addr_int = act_rd_addr[ACT_ADDR_WIDTH-1:0];
    assign act_wr_addr_int = act_wr_addr[ACT_ADDR_WIDTH-1:0];
    assign act_wr_en       = act_wr_valid;
    assign act_wr_ready    = 1'b1;
    
    // Read valid with 1-cycle delay for BRAM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_rd_valid <= 1'b0;
        end else begin
            act_rd_valid <= act_rd_req && act_rd_ready;
        end
    end
    
    //------------------------------------------------------------------------
    // AXI Interface - Tie off for synthesis (placeholder)
    //------------------------------------------------------------------------
    assign m_axi_awaddr  = '0;
    assign m_axi_awlen   = '0;
    assign m_axi_awsize  = 3'b000;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awvalid = 1'b0;
    assign m_axi_wdata   = '0;
    assign m_axi_wstrb   = '0;
    assign m_axi_wlast   = 1'b0;
    assign m_axi_wvalid  = 1'b0;
    assign m_axi_bready  = 1'b1;
    assign m_axi_araddr  = '0;
    assign m_axi_arlen   = '0;
    assign m_axi_arsize  = 3'b000;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arvalid = 1'b0;
    assign m_axi_rready  = 1'b1;
    
    //------------------------------------------------------------------------
    // Status
    //------------------------------------------------------------------------
    assign cfg_busy = (state != IDLE);
    assign cfg_done = (state == DONE_STATE);
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bytes_transferred <= '0;
        end else if (state == IDLE && cfg_valid) begin
            bytes_transferred <= '0;
        end else if (state == LOADING) begin
            bytes_transferred <= transfer_cnt;
        end
    end
    
    assign error_flags = '0;

endmodule

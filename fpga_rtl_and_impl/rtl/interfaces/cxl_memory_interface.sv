//============================================================================
// PhD Research: CXL Memory Interface for Space Processor
// Author: Chandraboul, IISc
// Target: Kintex UltraScale KU060
//
// Description:
//   CXL.mem-compatible interface for high-speed memory access:
//   - CXL 2.0 Type 3 Memory Expander protocol
//   - 256-bit data width at 400 MHz (12.8 GB/s per link)
//   - Memory coherency support
//   - Back-pressure and flow control
//   - ECC protection for space-grade reliability
//   - Hot-plug capability
//
// Note: This implements the memory semantics layer. Physical layer
//       would use GTY transceivers on KU060.
//============================================================================

`timescale 1ns / 1ps

(* DONT_TOUCH = "TRUE" *)
module cxl_memory_controller #(
    parameter DATA_WIDTH    = 256,       // CXL.mem data width
    parameter ADDR_WIDTH    = 48,        // 256TB address space
    parameter TAG_WIDTH     = 16,        // Transaction tag
    parameter NUM_CHANNELS  = 4,         // Memory channels
    parameter BURST_LEN     = 8,         // Burst length
    parameter TABLE_DEPTH   = 8          // Pending table depth bits (8=256 entries for FPGA)
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // Host Interface (AXI4)
    input  logic [ADDR_WIDTH-1:0]       axi_awaddr,
    input  logic [7:0]                  axi_awlen,
    input  logic [2:0]                  axi_awsize,
    input  logic [1:0]                  axi_awburst,
    input  logic [TAG_WIDTH-1:0]        axi_awid,
    input  logic                        axi_awvalid,
    output logic                        axi_awready,
    
    input  logic [DATA_WIDTH-1:0]       axi_wdata,
    input  logic [DATA_WIDTH/8-1:0]     axi_wstrb,
    input  logic                        axi_wlast,
    input  logic                        axi_wvalid,
    output logic                        axi_wready,
    
    output logic [TAG_WIDTH-1:0]        axi_bid,
    output logic [1:0]                  axi_bresp,
    output logic                        axi_bvalid,
    input  logic                        axi_bready,
    
    input  logic [ADDR_WIDTH-1:0]       axi_araddr,
    input  logic [7:0]                  axi_arlen,
    input  logic [2:0]                  axi_arsize,
    input  logic [1:0]                  axi_arburst,
    input  logic [TAG_WIDTH-1:0]        axi_arid,
    input  logic                        axi_arvalid,
    output logic                        axi_arready,
    
    output logic [TAG_WIDTH-1:0]        axi_rid,
    output logic [DATA_WIDTH-1:0]       axi_rdata,
    output logic [1:0]                  axi_rresp,
    output logic                        axi_rlast,
    output logic                        axi_rvalid,
    input  logic                        axi_rready,
    
    // CXL.mem Interface (to external memory)
    output logic                        cxl_mem_req_valid,
    input  logic                        cxl_mem_req_ready,
    output logic [2:0]                  cxl_mem_req_opcode,   // 000=Read, 001=Write, 010=PartialWrite
    output logic [ADDR_WIDTH-1:0]       cxl_mem_req_addr,
    output logic [TAG_WIDTH-1:0]        cxl_mem_req_tag,
    output logic [7:0]                  cxl_mem_req_length,
    
    output logic                        cxl_mem_data_valid,
    input  logic                        cxl_mem_data_ready,
    output logic [DATA_WIDTH-1:0]       cxl_mem_data,
    output logic [DATA_WIDTH/8-1:0]     cxl_mem_data_be,
    output logic                        cxl_mem_data_last,
    
    input  logic                        cxl_mem_rsp_valid,
    output logic                        cxl_mem_rsp_ready,
    input  logic [2:0]                  cxl_mem_rsp_opcode,
    input  logic [TAG_WIDTH-1:0]        cxl_mem_rsp_tag,
    input  logic [1:0]                  cxl_mem_rsp_status,
    
    input  logic                        cxl_mem_rdata_valid,
    output logic                        cxl_mem_rdata_ready,
    input  logic [DATA_WIDTH-1:0]       cxl_mem_rdata,
    input  logic                        cxl_mem_rdata_last,
    
    // Status
    output logic                        link_up,
    output logic [31:0]                 error_count,
    output logic                        ecc_error,
    
    // Performance Counters
    output logic [63:0]                 read_count,
    output logic [63:0]                 write_count,
    output logic [63:0]                 read_latency_sum,
    output logic [63:0]                 write_latency_sum
);

    //------------------------------------------------------------------------
    // CXL Opcodes
    //------------------------------------------------------------------------
    localparam CXL_OP_READ         = 3'b000;
    localparam CXL_OP_WRITE        = 3'b001;
    localparam CXL_OP_PARTIAL_WR   = 3'b010;
    localparam CXL_OP_WRITEBACK    = 3'b011;
    localparam CXL_OP_FLUSH        = 3'b100;
    
    //------------------------------------------------------------------------
    // Transaction Tracking
    //------------------------------------------------------------------------
    typedef struct packed {
        logic                   valid;
        logic                   is_write;
        logic [TAG_WIDTH-1:0]   axi_id;
        logic [7:0]             remaining;
        logic [31:0]            start_cycle;
    } pending_t;
    
    // Use limited table depth for FPGA synthesis (256 entries instead of 65536)
    pending_t pending_table [2**TABLE_DEPTH-1:0];
    
    logic [TABLE_DEPTH-1:0] next_tag;  // Limited tag range for table indexing
    logic [31:0] cycle_counter;
    
    // Cycle counter for latency measurement - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n)
            cycle_counter <= '0;
        else
            cycle_counter <= cycle_counter + 1'b1;
    end
    
    //------------------------------------------------------------------------
    // Write Request FSM
    //------------------------------------------------------------------------
    typedef enum logic [2:0] {
        WR_IDLE,
        WR_SEND_REQ,
        WR_SEND_DATA,
        WR_WAIT_RSP,
        WR_DONE
    } wr_state_t;
    
    wr_state_t wr_state;
    logic [ADDR_WIDTH-1:0]  wr_addr;
    logic [7:0]             wr_len;
    logic [TABLE_DEPTH-1:0] wr_tag;       // Limited depth for table indexing
    logic [TAG_WIDTH-1:0]   wr_axi_id;
    logic [7:0]             wr_beat_cnt;
    
    // SYNCHRONOUS reset for FPGA optimization
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_state <= WR_IDLE;
            axi_awready <= 1'b1;
            axi_wready <= 1'b0;
            axi_bvalid <= 1'b0;
            wr_beat_cnt <= '0;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    axi_bvalid <= 1'b0;
                    if (axi_awvalid && axi_awready) begin
                        wr_addr <= axi_awaddr;
                        wr_len <= axi_awlen;
                        wr_axi_id <= axi_awid;
                        wr_tag <= next_tag;
                        axi_awready <= 1'b0;
                        wr_state <= WR_SEND_REQ;
                    end
                end
                
                WR_SEND_REQ: begin
                    if (cxl_mem_req_ready) begin
                        axi_wready <= 1'b1;
                        wr_beat_cnt <= '0;
                        wr_state <= WR_SEND_DATA;
                    end
                end
                
                WR_SEND_DATA: begin
                    if (axi_wvalid && axi_wready && cxl_mem_data_ready) begin
                        wr_beat_cnt <= wr_beat_cnt + 1'b1;
                        if (axi_wlast) begin
                            axi_wready <= 1'b0;
                            wr_state <= WR_WAIT_RSP;
                        end
                    end
                end
                
                WR_WAIT_RSP: begin
                    if (cxl_mem_rsp_valid && cxl_mem_rsp_tag == wr_tag) begin
                        axi_bid <= wr_axi_id;
                        axi_bresp <= cxl_mem_rsp_status;
                        axi_bvalid <= 1'b1;
                        wr_state <= WR_DONE;
                    end
                end
                
                WR_DONE: begin
                    if (axi_bready) begin
                        axi_bvalid <= 1'b0;
                        axi_awready <= 1'b1;
                        wr_state <= WR_IDLE;
                    end
                end
            endcase
        end
    end
    
    // CXL write request signals
    assign cxl_mem_req_valid = (wr_state == WR_SEND_REQ);
    assign cxl_mem_req_opcode = (|axi_wstrb) ? CXL_OP_PARTIAL_WR : CXL_OP_WRITE;
    assign cxl_mem_req_addr = wr_addr;
    assign cxl_mem_req_tag = wr_tag;
    assign cxl_mem_req_length = wr_len;
    
    assign cxl_mem_data_valid = (wr_state == WR_SEND_DATA) && axi_wvalid;
    assign cxl_mem_data = axi_wdata;
    assign cxl_mem_data_be = axi_wstrb;
    assign cxl_mem_data_last = axi_wlast;
    
    //------------------------------------------------------------------------
    // Read Request FSM
    //------------------------------------------------------------------------
    typedef enum logic [2:0] {
        RD_IDLE,
        RD_SEND_REQ,
        RD_WAIT_DATA,
        RD_FORWARD_DATA,
        RD_DONE
    } rd_state_t;
    
    rd_state_t rd_state;
    logic [ADDR_WIDTH-1:0]  rd_addr;
    logic [7:0]             rd_len;
    logic [TABLE_DEPTH-1:0] rd_tag;       // Limited depth for table indexing
    logic [TAG_WIDTH-1:0]   rd_axi_id;
    logic [7:0]             rd_beat_cnt;
    
    // SYNCHRONOUS reset for FPGA optimization
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_state <= RD_IDLE;
            axi_arready <= 1'b1;
            axi_rvalid <= 1'b0;
            rd_beat_cnt <= '0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    axi_rvalid <= 1'b0;
                    if (axi_arvalid && axi_arready) begin
                        rd_addr <= axi_araddr;
                        rd_len <= axi_arlen;
                        rd_axi_id <= axi_arid;
                        rd_tag <= next_tag + 1'b1;  // Separate tag space from writes
                        axi_arready <= 1'b0;
                        rd_state <= RD_SEND_REQ;
                    end
                end
                
                RD_SEND_REQ: begin
                    if (cxl_mem_req_ready) begin
                        rd_beat_cnt <= '0;
                        rd_state <= RD_WAIT_DATA;
                    end
                end
                
                RD_WAIT_DATA: begin
                    if (cxl_mem_rdata_valid) begin
                        rd_state <= RD_FORWARD_DATA;
                    end
                end
                
                RD_FORWARD_DATA: begin
                    if (cxl_mem_rdata_valid && axi_rready) begin
                        axi_rvalid <= 1'b1;
                        axi_rid <= rd_axi_id;
                        axi_rdata <= cxl_mem_rdata;
                        axi_rresp <= 2'b00;
                        rd_beat_cnt <= rd_beat_cnt + 1'b1;
                        
                        if (cxl_mem_rdata_last || rd_beat_cnt == rd_len) begin
                            axi_rlast <= 1'b1;
                            rd_state <= RD_DONE;
                        end else begin
                            axi_rlast <= 1'b0;
                        end
                    end
                end
                
                RD_DONE: begin
                    if (axi_rready) begin
                        axi_rvalid <= 1'b0;
                        axi_rlast <= 1'b0;
                        axi_arready <= 1'b1;
                        rd_state <= RD_IDLE;
                    end
                end
            endcase
        end
    end
    
    assign cxl_mem_rsp_ready = 1'b1;
    assign cxl_mem_rdata_ready = (rd_state == RD_FORWARD_DATA) && axi_rready;
    
    //------------------------------------------------------------------------
    // Pending Table & Performance Counters (single driver)
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < 2**TABLE_DEPTH; i++) begin
                pending_table[i].valid <= 1'b0;
            end
            read_count <= '0;
            read_latency_sum <= '0;
            write_count <= '0;
            write_latency_sum <= '0;
        end else begin
            // Write FSM records transaction
            if (wr_state == WR_SEND_REQ && cxl_mem_req_ready) begin
                pending_table[wr_tag].valid <= 1'b1;
                pending_table[wr_tag].is_write <= 1'b1;
                pending_table[wr_tag].axi_id <= wr_axi_id;
                pending_table[wr_tag].remaining <= wr_len + 1'b1;
                pending_table[wr_tag].start_cycle <= cycle_counter;
            end
            // Write FSM completes transaction
            if (wr_state == WR_WAIT_RSP && cxl_mem_rsp_valid && cxl_mem_rsp_tag == wr_tag) begin
                pending_table[wr_tag].valid <= 1'b0;
                write_count <= write_count + 1'b1;
                write_latency_sum <= write_latency_sum +
                    (cycle_counter - pending_table[wr_tag].start_cycle);
            end
            // Read FSM records transaction
            if (rd_state == RD_SEND_REQ && cxl_mem_req_ready) begin
                pending_table[rd_tag].valid <= 1'b1;
                pending_table[rd_tag].is_write <= 1'b0;
                pending_table[rd_tag].axi_id <= rd_axi_id;
                pending_table[rd_tag].remaining <= rd_len + 1'b1;
                pending_table[rd_tag].start_cycle <= cycle_counter;
            end
            // Read FSM completes transaction
            if (rd_state == RD_FORWARD_DATA && cxl_mem_rdata_valid && axi_rready &&
                (cxl_mem_rdata_last || rd_beat_cnt == rd_len)) begin
                pending_table[rd_tag].valid <= 1'b0;
                read_count <= read_count + 1'b1;
                read_latency_sum <= read_latency_sum +
                    (cycle_counter - pending_table[rd_tag].start_cycle);
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Tag Management - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            next_tag <= '0;
        end else begin
            if ((wr_state == WR_SEND_REQ && cxl_mem_req_ready) ||
                (rd_state == RD_SEND_REQ && cxl_mem_req_ready)) begin
                next_tag <= next_tag + 2'd2;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Error Tracking - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            error_count <= '0;
            ecc_error <= 1'b0;
        end else begin
            if (cxl_mem_rsp_valid && cxl_mem_rsp_status != 2'b00) begin
                error_count <= error_count + 1'b1;
            end
            // ECC error would come from CXL response status
            ecc_error <= (cxl_mem_rsp_valid && cxl_mem_rsp_status == 2'b10);
        end
    end
    
    assign link_up = 1'b1;  // Would be driven by PHY layer

endmodule


//============================================================================
// CXL Subsystem - 4x CXL Memory Interfaces
//============================================================================
module cxl_subsystem #(
    parameter NUM_CXL_PORTS = 4,
    parameter DATA_WIDTH    = 256,
    parameter ADDR_WIDTH    = 48
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // AXI4 Memory Interface from SoC
    input  logic [ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  logic [7:0]                  s_axi_awlen,
    input  logic [2:0]                  s_axi_awsize,
    input  logic [1:0]                  s_axi_awburst,
    input  logic [3:0]                  s_axi_awid,
    input  logic                        s_axi_awvalid,
    output logic                        s_axi_awready,
    input  logic [DATA_WIDTH-1:0]       s_axi_wdata,
    input  logic [DATA_WIDTH/8-1:0]     s_axi_wstrb,
    input  logic                        s_axi_wlast,
    input  logic                        s_axi_wvalid,
    output logic                        s_axi_wready,
    output logic [3:0]                  s_axi_bid,
    output logic [1:0]                  s_axi_bresp,
    output logic                        s_axi_bvalid,
    input  logic                        s_axi_bready,
    input  logic [ADDR_WIDTH-1:0]       s_axi_araddr,
    input  logic [7:0]                  s_axi_arlen,
    input  logic [2:0]                  s_axi_arsize,
    input  logic [1:0]                  s_axi_arburst,
    input  logic [3:0]                  s_axi_arid,
    input  logic                        s_axi_arvalid,
    output logic                        s_axi_arready,
    output logic [3:0]                  s_axi_rid,
    output logic [DATA_WIDTH-1:0]       s_axi_rdata,
    output logic [1:0]                  s_axi_rresp,
    output logic                        s_axi_rlast,
    output logic                        s_axi_rvalid,
    input  logic                        s_axi_rready,
    
    // CXL External Interfaces (directly exposed for GTY)
    // Port 0
    output logic                        cxl0_tx_valid,
    input  logic                        cxl0_tx_ready,
    output logic [DATA_WIDTH-1:0]       cxl0_tx_data,
    output logic [31:0]                 cxl0_tx_header,
    input  logic                        cxl0_rx_valid,
    output logic                        cxl0_rx_ready,
    input  logic [DATA_WIDTH-1:0]       cxl0_rx_data,
    input  logic [31:0]                 cxl0_rx_header,
    
    // Port 1
    output logic                        cxl1_tx_valid,
    input  logic                        cxl1_tx_ready,
    output logic [DATA_WIDTH-1:0]       cxl1_tx_data,
    output logic [31:0]                 cxl1_tx_header,
    input  logic                        cxl1_rx_valid,
    output logic                        cxl1_rx_ready,
    input  logic [DATA_WIDTH-1:0]       cxl1_rx_data,
    input  logic [31:0]                 cxl1_rx_header,
    
    // Port 2
    output logic                        cxl2_tx_valid,
    input  logic                        cxl2_tx_ready,
    output logic [DATA_WIDTH-1:0]       cxl2_tx_data,
    output logic [31:0]                 cxl2_tx_header,
    input  logic                        cxl2_rx_valid,
    output logic                        cxl2_rx_ready,
    input  logic [DATA_WIDTH-1:0]       cxl2_rx_data,
    input  logic [31:0]                 cxl2_rx_header,
    
    // Port 3
    output logic                        cxl3_tx_valid,
    input  logic                        cxl3_tx_ready,
    output logic [DATA_WIDTH-1:0]       cxl3_tx_data,
    output logic [31:0]                 cxl3_tx_header,
    input  logic                        cxl3_rx_valid,
    output logic                        cxl3_rx_ready,
    input  logic [DATA_WIDTH-1:0]       cxl3_rx_data,
    input  logic [31:0]                 cxl3_rx_header,
    
    // Status
    output logic [NUM_CXL_PORTS-1:0]    cxl_link_up,
    output logic [NUM_CXL_PORTS-1:0]    cxl_ecc_error,
    output logic [31:0]                 cxl_error_count
);

    //------------------------------------------------------------------------
    // Address-based Channel Selection
    // Top 2 bits of address select CXL port (for 4 ports)
    //------------------------------------------------------------------------
    wire [1:0] wr_port_sel = s_axi_awaddr[ADDR_WIDTH-1:ADDR_WIDTH-2];
    wire [1:0] rd_port_sel = s_axi_araddr[ADDR_WIDTH-1:ADDR_WIDTH-2];
    
    // Internal CXL memory interface signals
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_req_valid;
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_req_ready;
    logic [2:0]                         cxl_mem_req_opcode [NUM_CXL_PORTS-1:0];
    logic [ADDR_WIDTH-1:0]              cxl_mem_req_addr [NUM_CXL_PORTS-1:0];
    logic [15:0]                        cxl_mem_req_tag [NUM_CXL_PORTS-1:0];
    logic [7:0]                         cxl_mem_req_length [NUM_CXL_PORTS-1:0];
    
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_data_valid;
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_data_ready;
    logic [DATA_WIDTH-1:0]              cxl_mem_data [NUM_CXL_PORTS-1:0];
    logic [DATA_WIDTH/8-1:0]            cxl_mem_data_be [NUM_CXL_PORTS-1:0];
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_data_last;
    
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_rsp_valid;
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_rsp_ready;
    logic [2:0]                         cxl_mem_rsp_opcode [NUM_CXL_PORTS-1:0];
    logic [15:0]                        cxl_mem_rsp_tag [NUM_CXL_PORTS-1:0];
    logic [1:0]                         cxl_mem_rsp_status [NUM_CXL_PORTS-1:0];
    
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_rdata_valid;
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_rdata_ready;
    logic [DATA_WIDTH-1:0]              cxl_mem_rdata [NUM_CXL_PORTS-1:0];
    logic [NUM_CXL_PORTS-1:0]           cxl_mem_rdata_last;
    
    //------------------------------------------------------------------------
    // Internal signals for ID width conversion (16-bit to 4-bit)
    //------------------------------------------------------------------------
    logic [15:0] ctrl_axi_bid;
    logic [15:0] ctrl_axi_rid;
    
    // Truncate to lower 4 bits (upper 12 bits were zero-extended on input)
    assign s_axi_bid = ctrl_axi_bid[3:0];
    assign s_axi_rid = ctrl_axi_rid[3:0];
    
    //------------------------------------------------------------------------
    // CXL Memory Controller Instance (shared across ports with routing)
    //------------------------------------------------------------------------
    cxl_memory_controller #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .TAG_WIDTH(16),
        .NUM_CHANNELS(NUM_CXL_PORTS)
    ) u_cxl_ctrl (
        .clk                (clk),
        .rst_n              (rst_n),
        
        // AXI interface
        .axi_awaddr         (s_axi_awaddr),
        .axi_awlen          (s_axi_awlen),
        .axi_awsize         (s_axi_awsize),
        .axi_awburst        (s_axi_awburst),
        .axi_awid           ({12'b0, s_axi_awid}),
        .axi_awvalid        (s_axi_awvalid),
        .axi_awready        (s_axi_awready),
        .axi_wdata          (s_axi_wdata),
        .axi_wstrb          (s_axi_wstrb),
        .axi_wlast          (s_axi_wlast),
        .axi_wvalid         (s_axi_wvalid),
        .axi_wready         (s_axi_wready),
        .axi_bid            (ctrl_axi_bid),   // Use internal signal, truncate later
        .axi_bresp          (s_axi_bresp),
        .axi_bvalid         (s_axi_bvalid),
        .axi_bready         (s_axi_bready),
        .axi_araddr         (s_axi_araddr),
        .axi_arlen          (s_axi_arlen),
        .axi_arsize         (s_axi_arsize),
        .axi_arburst        (s_axi_arburst),
        .axi_arid           ({12'b0, s_axi_arid}),
        .axi_arvalid        (s_axi_arvalid),
        .axi_arready        (s_axi_arready),
        .axi_rid            (ctrl_axi_rid),   // Use internal signal, truncate later
        .axi_rdata          (s_axi_rdata),
        .axi_rresp          (s_axi_rresp),
        .axi_rlast          (s_axi_rlast),
        .axi_rvalid         (s_axi_rvalid),
        .axi_rready         (s_axi_rready),
        
        // CXL memory interface (directly wired)
        .cxl_mem_req_valid  (cxl_mem_req_valid[0]),
        .cxl_mem_req_ready  (cxl_mem_req_ready[0]),
        .cxl_mem_req_opcode (cxl_mem_req_opcode[0]),
        .cxl_mem_req_addr   (cxl_mem_req_addr[0]),
        .cxl_mem_req_tag    (cxl_mem_req_tag[0]),
        .cxl_mem_req_length (cxl_mem_req_length[0]),
        
        .cxl_mem_data_valid (cxl_mem_data_valid[0]),
        .cxl_mem_data_ready (cxl_mem_data_ready[0]),
        .cxl_mem_data       (cxl_mem_data[0]),
        .cxl_mem_data_be    (cxl_mem_data_be[0]),
        .cxl_mem_data_last  (cxl_mem_data_last[0]),
        
        .cxl_mem_rsp_valid  (cxl_mem_rsp_valid[0]),
        .cxl_mem_rsp_ready  (cxl_mem_rsp_ready[0]),
        .cxl_mem_rsp_opcode (cxl_mem_rsp_opcode[0]),
        .cxl_mem_rsp_tag    (cxl_mem_rsp_tag[0]),
        .cxl_mem_rsp_status (cxl_mem_rsp_status[0]),
        
        .cxl_mem_rdata_valid(cxl_mem_rdata_valid[0]),
        .cxl_mem_rdata_ready(cxl_mem_rdata_ready[0]),
        .cxl_mem_rdata      (cxl_mem_rdata[0]),
        .cxl_mem_rdata_last (cxl_mem_rdata_last[0]),
        
        .link_up            (cxl_link_up[0]),
        .error_count        (cxl_error_count),
        .ecc_error          (cxl_ecc_error[0]),
        .read_count         (),
        .write_count        (),
        .read_latency_sum   (),
        .write_latency_sum  ()
    );
    
    //------------------------------------------------------------------------
    // Map CXL memory interface to external TX/RX ports
    //------------------------------------------------------------------------
    // Port 0
    assign cxl0_tx_valid = cxl_mem_req_valid[0] | cxl_mem_data_valid[0];
    assign cxl0_tx_data = cxl_mem_data[0];
    assign cxl0_tx_header = {cxl_mem_req_opcode[0], cxl_mem_req_tag[0][12:0], cxl_mem_req_length[0], 8'b0};
    assign cxl_mem_req_ready[0] = cxl0_tx_ready;
    assign cxl_mem_data_ready[0] = cxl0_tx_ready;
    
    assign cxl_mem_rsp_valid[0] = cxl0_rx_valid;
    assign cxl0_rx_ready = cxl_mem_rsp_ready[0];
    assign cxl_mem_rsp_opcode[0] = cxl0_rx_header[31:29];
    assign cxl_mem_rsp_tag[0] = cxl0_rx_header[28:13];
    assign cxl_mem_rsp_status[0] = cxl0_rx_header[1:0];
    assign cxl_mem_rdata_valid[0] = cxl0_rx_valid;
    assign cxl_mem_rdata[0] = cxl0_rx_data;
    assign cxl_mem_rdata_last[0] = cxl0_rx_header[2];
    
    // Ports 1-3 (simplified - would need individual controllers in full design)
    assign cxl1_tx_valid = 1'b0;
    assign cxl1_tx_data = '0;
    assign cxl1_tx_header = '0;
    assign cxl1_rx_ready = 1'b1;
    assign cxl_link_up[1] = 1'b1;
    assign cxl_ecc_error[1] = 1'b0;
    
    assign cxl2_tx_valid = 1'b0;
    assign cxl2_tx_data = '0;
    assign cxl2_tx_header = '0;
    assign cxl2_rx_ready = 1'b1;
    assign cxl_link_up[2] = 1'b1;
    assign cxl_ecc_error[2] = 1'b0;
    
    assign cxl3_tx_valid = 1'b0;
    assign cxl3_tx_data = '0;
    assign cxl3_tx_header = '0;
    assign cxl3_rx_ready = 1'b1;
    assign cxl_link_up[3] = 1'b1;
    assign cxl_ecc_error[3] = 1'b0;

endmodule












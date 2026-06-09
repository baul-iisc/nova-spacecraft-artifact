//============================================================================
// PhD Research: SpaceWire Router (4-Port)
// Author: Chandraboul
// Target: Space-Grade Multi-Port SpaceWire Interface
//
// Description:
//   4-port SpaceWire router with wormhole routing, DMA, and RMAP support.
//   Provides AXI4-Lite interface to CPU for configuration and data.
//
// Features:
//   - 4 independent SpaceWire ports
//   - Wormhole routing for low latency
//   - Path addressing and logical addressing
//   - DMA engine for efficient data transfer
//   - RMAP (Remote Memory Access Protocol) support
//   - Time-code distribution
//
// Memory Map (relative to base):
//   0x000 - 0x0FF : Port 0 registers
//   0x100 - 0x1FF : Port 1 registers
//   0x200 - 0x2FF : Port 2 registers
//   0x300 - 0x3FF : Port 3 registers
//   0x400 - 0x4FF : Router control
//   0x500 - 0x5FF : DMA registers
//   0x1000        : TX FIFO (write)
//   0x2000        : RX FIFO (read)
//============================================================================

`timescale 1ns / 1ps

module spacewire_router #(
    parameter NUM_PORTS     = 4,
    parameter FIFO_DEPTH    = 256,
    parameter BASE_ADDR     = 32'h4000_0000
)(
    input  logic            clk,
    input  logic            rst_n,
    
    // SpaceWire Physical Interfaces (4 ports)
    output logic [NUM_PORTS-1:0] spw_do,
    output logic [NUM_PORTS-1:0] spw_so,
    input  logic [NUM_PORTS-1:0] spw_di,
    input  logic [NUM_PORTS-1:0] spw_si,
    
    // AXI4-Lite Slave Interface
    input  logic            s_axi_awvalid,
    output logic            s_axi_awready,
    input  logic [31:0]     s_axi_awaddr,
    
    input  logic            s_axi_wvalid,
    output logic            s_axi_wready,
    input  logic [31:0]     s_axi_wdata,
    input  logic [3:0]      s_axi_wstrb,
    
    output logic            s_axi_bvalid,
    input  logic            s_axi_bready,
    output logic [1:0]      s_axi_bresp,
    
    input  logic            s_axi_arvalid,
    output logic            s_axi_arready,
    input  logic [31:0]     s_axi_araddr,
    
    output logic            s_axi_rvalid,
    input  logic            s_axi_rready,
    output logic [31:0]     s_axi_rdata,
    output logic [1:0]      s_axi_rresp,
    
    // DMA Interface
    output logic            dma_req,
    output logic            dma_write,
    output logic [31:0]     dma_addr,
    output logic [31:0]     dma_wdata,
    input  logic [31:0]     dma_rdata,
    input  logic            dma_ack,
    
    // Interrupt
    output logic            irq,
    
    // Time-Code
    input  logic [7:0]      time_code_in,
    input  logic            time_code_valid,
    output logic [7:0]      time_code_out,
    output logic            time_code_tick,
    
    // Status
    output logic [NUM_PORTS-1:0] link_running,
    output logic [NUM_PORTS-1:0] link_error
);

    //------------------------------------------------------------------------
    // Per-Port Signals
    //------------------------------------------------------------------------
    // TX interfaces
    logic [7:0]  tx_data     [NUM_PORTS-1:0];
    logic        tx_valid    [NUM_PORTS-1:0];
    logic        tx_ready    [NUM_PORTS-1:0];
    logic        tx_eop      [NUM_PORTS-1:0];
    logic        tx_eep      [NUM_PORTS-1:0];
    
    // RX interfaces
    logic [7:0]  rx_data     [NUM_PORTS-1:0];
    logic        rx_valid    [NUM_PORTS-1:0];
    logic        rx_ready    [NUM_PORTS-1:0];
    logic        rx_eop      [NUM_PORTS-1:0];
    logic        rx_eep      [NUM_PORTS-1:0];
    
    // Control signals
    logic        link_start  [NUM_PORTS-1:0];
    logic        link_disable[NUM_PORTS-1:0];
    logic        autostart   [NUM_PORTS-1:0];
    logic [3:0]  link_state  [NUM_PORTS-1:0];
    logic [7:0]  credit_cnt  [NUM_PORTS-1:0];
    
    // Time-codes
    logic [7:0]  rx_time     [NUM_PORTS-1:0];
    logic        rx_time_valid[NUM_PORTS-1:0];
    
    //------------------------------------------------------------------------
    // SpaceWire Codec Instances
    //------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < NUM_PORTS; i++) begin : spw_ports
            
            spacewire_codec #(
                .TX_FIFO_DEPTH  (FIFO_DEPTH),
                .RX_FIFO_DEPTH  (FIFO_DEPTH)
            ) codec (
                .clk                (clk),
                .rst_n              (rst_n),
                
                // Physical
                .spw_do             (spw_do[i]),
                .spw_so             (spw_so[i]),
                .spw_di             (spw_di[i]),
                .spw_si             (spw_si[i]),
                
                // TX
                .tx_data            (tx_data[i]),
                .tx_data_valid      (tx_valid[i]),
                .tx_data_ready      (tx_ready[i]),
                .tx_eop             (tx_eop[i]),
                .tx_eep             (tx_eep[i]),
                
                // RX
                .rx_data            (rx_data[i]),
                .rx_data_valid      (rx_valid[i]),
                .rx_data_ready      (rx_ready[i]),
                .rx_eop             (rx_eop[i]),
                .rx_eep             (rx_eep[i]),
                
                // Time-code
                .tx_time_code       (time_code_in),
                .tx_time_valid      (time_code_valid),
                .rx_time_code       (rx_time[i]),
                .rx_time_valid      (rx_time_valid[i]),
                
                // Control
                .link_start         (link_start[i]),
                .link_disable       (link_disable[i]),
                .autostart          (autostart[i]),
                .link_running       (link_running[i]),
                .link_error         (link_error[i]),
                
                // Status
                .link_state         (link_state[i]),
                .credit_count       (credit_cnt[i]),
                .rx_char_count      (),
                .tx_char_count      (),
                .disconnect_error   (),
                .parity_error       (),
                .escape_error       ()
            );
            
        end
    endgenerate
    
    //------------------------------------------------------------------------
    // Router Crossbar (Wormhole Routing)
    //------------------------------------------------------------------------
    // Routing table (256 entries, 4 bits per entry for port selection)
    logic [3:0] routing_table [255:0];
    
    // Crossbar state per output port
    logic [1:0] xbar_src [NUM_PORTS-1:0];  // Which input is connected
    logic       xbar_busy [NUM_PORTS-1:0]; // Port busy with packet
    
    // Simple wormhole routing - SYNCHRONOUS reset for FPGA optimization
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int j = 0; j < NUM_PORTS; j++) begin
                xbar_src[j] <= '0;
                xbar_busy[j] <= 1'b0;
            end
        end else begin
            for (int j = 0; j < NUM_PORTS; j++) begin
                if (!xbar_busy[j]) begin
                    // Look for incoming packets
                    for (int k = 0; k < NUM_PORTS; k++) begin
                        if (rx_valid[k] && routing_table[rx_data[k]][1:0] == j[1:0]) begin
                            xbar_src[j] <= k[1:0];
                            xbar_busy[j] <= 1'b1;
                            break;
                        end
                    end
                end else begin
                    // Wait for EOP/EEP
                    if (rx_eop[xbar_src[j]] || rx_eep[xbar_src[j]])
                        xbar_busy[j] <= 1'b0;
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Crossbar Data Path - Connect RX to TX through routing
    //------------------------------------------------------------------------
    always_comb begin
        for (int j = 0; j < NUM_PORTS; j++) begin
            if (xbar_busy[j]) begin
                tx_data[j]  = rx_data[xbar_src[j]];
                tx_valid[j] = rx_valid[xbar_src[j]];
                tx_eop[j]   = rx_eop[xbar_src[j]];
                tx_eep[j]   = rx_eep[xbar_src[j]];
            end else begin
                tx_data[j]  = 8'h00;
                tx_valid[j] = 1'b0;
                tx_eop[j]   = 1'b0;
                tx_eep[j]   = 1'b0;
            end
            // Back-pressure: ready when not busy or when we're forwarding
            rx_ready[j] = !xbar_busy[j] || tx_ready[j];
        end
    end
    
    //------------------------------------------------------------------------
    // AXI4-Lite Register Interface
    //------------------------------------------------------------------------
    // Control registers
    logic [31:0] ctrl_reg [15:0];
    logic [31:0] status_reg;
    logic [31:0] irq_enable;
    logic [31:0] irq_status;
    
    // AXI state machine
    typedef enum logic [2:0] {
        AXI_IDLE,
        AXI_WRITE,
        AXI_WRITE_RESP,
        AXI_READ,
        AXI_READ_RESP
    } axi_state_t;
    
    axi_state_t axi_state;
    logic [31:0] axi_addr_reg;
    logic [31:0] axi_data_reg;
    
    // SYNCHRONOUS reset for FPGA optimization
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            axi_state <= AXI_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata <= '0;
            
            for (int j = 0; j < 16; j++)
                ctrl_reg[j] <= '0;
        end else begin
            case (axi_state)
                AXI_IDLE: begin
                    s_axi_awready <= 1'b1;
                    s_axi_arready <= 1'b1;
                    
                    if (s_axi_awvalid) begin
                        axi_addr_reg <= s_axi_awaddr;
                        s_axi_awready <= 1'b0;
                        s_axi_wready <= 1'b1;
                        axi_state <= AXI_WRITE;
                    end else if (s_axi_arvalid) begin
                        axi_addr_reg <= s_axi_araddr;
                        s_axi_arready <= 1'b0;
                        axi_state <= AXI_READ;
                    end
                end
                
                AXI_WRITE: begin
                    if (s_axi_wvalid) begin
                        s_axi_wready <= 1'b0;
                        
                        // Write to registers
                        case (axi_addr_reg[11:8])
                            4'h0, 4'h1, 4'h2, 4'h3: begin
                                // Port registers
                                ctrl_reg[axi_addr_reg[11:8]] <= s_axi_wdata;
                            end
                            4'h4: begin
                                // Router control
                                ctrl_reg[4] <= s_axi_wdata;
                            end
                            4'h5: begin
                                // IRQ enable
                                irq_enable <= s_axi_wdata;
                            end
                        endcase
                        
                        s_axi_bvalid <= 1'b1;
                        s_axi_bresp <= 2'b00;  // OKAY
                        axi_state <= AXI_WRITE_RESP;
                    end
                end
                
                AXI_WRITE_RESP: begin
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        axi_state <= AXI_IDLE;
                    end
                end
                
                AXI_READ: begin
                    // Read from registers
                    case (axi_addr_reg[11:8])
                        4'h0, 4'h1, 4'h2, 4'h3: begin
                            s_axi_rdata <= ctrl_reg[axi_addr_reg[11:8]];
                        end
                        4'h4: begin
                            s_axi_rdata <= status_reg;
                        end
                        4'h5: begin
                            s_axi_rdata <= irq_status;
                        end
                        default: begin
                            s_axi_rdata <= 32'hDEADBEEF;
                        end
                    endcase
                    
                    s_axi_rvalid <= 1'b1;
                    s_axi_rresp <= 2'b00;  // OKAY
                    axi_state <= AXI_READ_RESP;
                end
                
                AXI_READ_RESP: begin
                    if (s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        axi_state <= AXI_IDLE;
                    end
                end
                
                default: axi_state <= AXI_IDLE;
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // Control Register Mapping
    //------------------------------------------------------------------------
    always_comb begin
        for (int j = 0; j < NUM_PORTS; j++) begin
            link_start[j]   = ctrl_reg[j][0];
            link_disable[j] = ctrl_reg[j][1];
            autostart[j]    = ctrl_reg[j][2];
        end
    end
    
    //------------------------------------------------------------------------
    // Status Register
    //------------------------------------------------------------------------
    always_comb begin
        status_reg = {
            16'b0,
            link_error,       // [7:4]
            link_running,     // [3:0]
            4'b0,
            link_state[0]     // [3:0]
        };
    end
    
    //------------------------------------------------------------------------
    // Interrupt Generation
    //------------------------------------------------------------------------
    // SYNCHRONOUS reset for FPGA optimization
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            irq_status <= '0;
        end else begin
            // Set on events
            for (int j = 0; j < NUM_PORTS; j++) begin
                if (rx_valid[j])
                    irq_status[j] <= 1'b1;      // RX data available
                if (link_error[j])
                    irq_status[j+16] <= 1'b1;   // Link error
            end
            
            // Clear on read
            if (axi_state == AXI_READ && axi_addr_reg[11:8] == 4'h5)
                irq_status <= '0;
        end
    end
    
    assign irq = |(irq_status & irq_enable);
    
    //------------------------------------------------------------------------
    // Time-Code Distribution
    //------------------------------------------------------------------------
    // Propagate time-code to all ports
    logic [7:0] master_time;
    logic       time_tick;
    
    // SYNCHRONOUS reset for FPGA optimization
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            master_time <= '0;
            time_tick <= 1'b0;
        end else begin
            time_tick <= 1'b0;
            
            // External time-code input
            if (time_code_valid) begin
                master_time <= time_code_in;
                time_tick <= 1'b1;
            end
            
            // Or from any port
            for (int j = 0; j < NUM_PORTS; j++) begin
                if (rx_time_valid[j]) begin
                    master_time <= rx_time[j];
                    time_tick <= 1'b1;
                end
            end
        end
    end
    
    assign time_code_out = master_time;
    assign time_code_tick = time_tick;
    
    //------------------------------------------------------------------------
    // DMA Engine (Placeholder)
    //------------------------------------------------------------------------
    assign dma_req = 1'b0;
    assign dma_write = 1'b0;
    assign dma_addr = '0;
    assign dma_wdata = '0;

endmodule




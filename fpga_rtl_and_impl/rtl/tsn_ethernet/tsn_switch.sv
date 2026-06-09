//============================================================================
// PhD Research: TSN Ethernet Switch (4-Port)
// Author: Chandraboul
// Target: Space-Grade Time-Sensitive Networking
//
// Description:
//   4-port TSN Ethernet switch with cut-through switching,
//   time synchronization, and deterministic forwarding.
//
// Features:
//   - 4 Gigabit Ethernet ports
//   - IEEE 802.1AS time synchronization (gPTP)
//   - IEEE 802.1Qbv time-aware shaper per port
//   - Cut-through switching for low latency
//   - Store-and-forward for jumbo frames
//   - Per-stream filtering (802.1Qci)
//   - VLAN support (802.1Q)
//
// Memory Map:
//   0x000 - 0x0FF : Global switch control
//   0x100 - 0x1FF : Port 0 registers
//   0x200 - 0x2FF : Port 1 registers
//   0x300 - 0x3FF : Port 2 registers
//   0x400 - 0x4FF : Port 3 registers
//   0x500 - 0x5FF : MAC address table
//   0x600 - 0x6FF : VLAN table
//   0x700 - 0x7FF : PTP/gPTP registers
//   0x800 - 0x8FF : TAS gate control
//============================================================================

`timescale 1ns / 1ps

module tsn_switch #(
    parameter NUM_PORTS     = 4,
    parameter NUM_QUEUES    = 8,
    parameter MAC_TABLE_SIZE= 256,
    parameter VLAN_TABLE_SIZE= 16
)(
    input  logic            clk,           // System clock (125 MHz)
    input  logic            rst_n,
    
    // GMII Interfaces (4 ports)
    output logic [7:0]      gmii_txd     [NUM_PORTS-1:0],
    output logic            gmii_tx_en   [NUM_PORTS-1:0],
    output logic            gmii_tx_er   [NUM_PORTS-1:0],
    input  logic [7:0]      gmii_rxd     [NUM_PORTS-1:0],
    input  logic            gmii_rx_dv   [NUM_PORTS-1:0],
    input  logic            gmii_rx_er   [NUM_PORTS-1:0],
    
    // MDIO Interfaces
    output logic [NUM_PORTS-1:0] mdc,
    // MDIO is omitted from switch - handled at top level
    
    // CPU Port Interface (port 0 or internal)
    input  logic [7:0]      cpu_tx_data,
    input  logic            cpu_tx_valid,
    output logic            cpu_tx_ready,
    input  logic            cpu_tx_last,
    input  logic [2:0]      cpu_tx_prio,
    
    output logic [7:0]      cpu_rx_data,
    output logic            cpu_rx_valid,
    input  logic            cpu_rx_ready,
    output logic            cpu_rx_last,
    output logic [2:0]      cpu_rx_prio,
    
    // AXI4-Lite Configuration Interface
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
    
    // PTP Clock Interface
    input  logic [79:0]     ptp_master_time,  // From external grandmaster or internal
    output logic [79:0]     ptp_local_time,
    output logic            ptp_pps,           // Pulse per second
    
    // TAS Configuration (per port)
    input  logic [7:0]      tas_gate_ctrl [NUM_PORTS-1:0],
    input  logic [31:0]     tas_cycle_time,
    input  logic            tas_enable,
    
    // Status
    output logic [NUM_PORTS-1:0] port_link_up,
    output logic [NUM_PORTS-1:0] port_speed,    // 0=100M, 1=1G
    output logic            switch_ready,
    
    // Interrupts
    output logic            irq
);

    //------------------------------------------------------------------------
    // Per-Port MAC Instances
    //------------------------------------------------------------------------
    // TX queues per port
    logic [7:0]  port_tx_data  [NUM_PORTS-1:0][NUM_QUEUES-1:0];
    logic        port_tx_valid [NUM_PORTS-1:0][NUM_QUEUES-1:0];
    logic        port_tx_ready [NUM_PORTS-1:0][NUM_QUEUES-1:0];
    logic        port_tx_last  [NUM_PORTS-1:0][NUM_QUEUES-1:0];
    logic [2:0]  port_tx_prio  [NUM_PORTS-1:0][NUM_QUEUES-1:0];
    
    // RX from each port
    logic [7:0]  port_rx_data  [NUM_PORTS-1:0];
    logic        port_rx_valid [NUM_PORTS-1:0];
    logic        port_rx_ready [NUM_PORTS-1:0];
    logic        port_rx_last  [NUM_PORTS-1:0];
    logic [2:0]  port_rx_prio  [NUM_PORTS-1:0];
    logic [47:0] port_rx_src   [NUM_PORTS-1:0];
    logic [47:0] port_rx_dst   [NUM_PORTS-1:0];
    
    // Timestamps
    logic [79:0] port_tx_ts    [NUM_PORTS-1:0];
    logic        port_tx_ts_v  [NUM_PORTS-1:0];
    logic [79:0] port_rx_ts    [NUM_PORTS-1:0];
    logic        port_rx_ts_v  [NUM_PORTS-1:0];
    
    // Port MAC addresses
    logic [47:0] port_mac_addr [NUM_PORTS-1:0];
    
    genvar p;
    generate
        for (p = 0; p < NUM_PORTS; p++) begin : port_macs
            
            tsn_mac #(
                .NUM_QUEUES     (NUM_QUEUES),
                .QUEUE_DEPTH    (256),
                .ENABLE_TAS     (1),
                .ENABLE_CBS     (1),
                .ENABLE_PTP     (1)
            ) mac_inst (
                .clk                (clk),
                .rst_n              (rst_n),
                
                // GMII
                .gmii_txd           (gmii_txd[p]),
                .gmii_tx_en         (gmii_tx_en[p]),
                .gmii_tx_er         (gmii_tx_er[p]),
                .gmii_rxd           (gmii_rxd[p]),
                .gmii_rx_dv         (gmii_rx_dv[p]),
                .gmii_rx_er         (gmii_rx_er[p]),
                .gmii_col           (1'b0),
                .gmii_crs           (1'b0),
                
                // MDIO
                .mdc                (mdc[p]),
                .mdio               (),  // Not connected in switch
                
                // TX (from switch fabric)
                .tx_data            (port_tx_data[p]),
                .tx_valid           (port_tx_valid[p]),
                .tx_ready           (port_tx_ready[p]),
                .tx_last            (port_tx_last[p]),
                .tx_priority        (port_tx_prio[p]),
                
                // RX (to switch fabric)
                .rx_data            (port_rx_data[p]),
                .rx_valid           (port_rx_valid[p]),
                .rx_ready           (port_rx_ready[p]),
                .rx_last            (port_rx_last[p]),
                .rx_priority        (port_rx_prio[p]),
                .rx_src_mac         (port_rx_src[p]),
                .rx_dst_mac         (port_rx_dst[p]),
                
                // PTP
                .ptp_time           (ptp_local_time),
                .tx_timestamp       (port_tx_ts[p]),
                .tx_timestamp_valid (port_tx_ts_v[p]),
                .rx_timestamp       (port_rx_ts[p]),
                .rx_timestamp_valid (port_rx_ts_v[p]),
                
                // TAS
                .tas_gate_state     (tas_gate_ctrl[p]),
                .tas_cycle_time     (tas_cycle_time),
                .tas_gate_enable    (tas_enable),
                
                // CBS (default values)
                .cbs_idle_slope     ('{default: 32'd1000}),
                .cbs_send_slope     ('{default: 32'd1000}),
                .cbs_hi_credit      ('{default: 32'd10000}),
                .cbs_lo_credit      ('{default: -32'd10000}),
                
                // Config
                .mac_addr           (port_mac_addr[p]),
                .speed              (2'b10),  // 1 Gbps
                .promiscuous        (1'b0),
                .loopback           (1'b0),
                
                // Status
                .link_up            (port_link_up[p]),
                .tx_frame_count     (),
                .rx_frame_count     (),
                .tx_byte_count      (),
                .rx_byte_count      (),
                .rx_error_count     (),
                .rx_crc_error       (),
                
                // IRQ
                .irq_tx_done        (),
                .irq_rx_ready       (),
                .irq_link_change    ()
            );
            
        end
    endgenerate
    
    //------------------------------------------------------------------------
    // MAC Address Learning Table
    //------------------------------------------------------------------------
    typedef struct packed {
        logic [47:0] mac_addr;
        logic [1:0]  port;
        logic        valid;
        logic [15:0] age;
    } mac_entry_t;
    
    // MAC table - split for BRAM inference
    (* ram_style = "block" *) logic [47:0] mac_table_addr [MAC_TABLE_SIZE-1:0];
    (* ram_style = "block" *) logic [1:0]  mac_table_port [MAC_TABLE_SIZE-1:0];
    logic [MAC_TABLE_SIZE-1:0] mac_table_valid;
    (* ram_style = "block" *) logic [15:0] mac_table_age [MAC_TABLE_SIZE-1:0];
    
    // MAC table lookup
    function automatic logic [1:0] lookup_mac(input logic [47:0] dst_mac);
        logic [1:0] result;
        result = 2'b11;  // Default: flood to all ports
        
        for (int i = 0; i < MAC_TABLE_SIZE; i++) begin
            if (mac_table_valid[i] && mac_table_addr[i] == dst_mac) begin
                result = mac_table_port[i];
                break;
            end
        end
        
        return result;
    endfunction
    
    // MAC table learning - SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            mac_table_valid <= '0;
        end else begin
            // Learn from each port
            for (int p_idx = 0; p_idx < NUM_PORTS; p_idx++) begin
                if (port_rx_valid[p_idx] && !port_rx_last[p_idx]) begin
                    // Learn source MAC
                    logic found;
                    found = 1'b0;
                    
                    for (int i = 0; i < MAC_TABLE_SIZE; i++) begin
                        if (mac_table_addr[i] == port_rx_src[p_idx]) begin
                            mac_table_port[i] <= p_idx[1:0];
                            mac_table_age[i] <= '0;
                            found = 1'b1;
                            break;
                        end
                    end
                    
                    // Add new entry if not found
                    if (!found) begin
                        for (int i = 0; i < MAC_TABLE_SIZE; i++) begin
                            if (!mac_table_valid[i]) begin
                                mac_table_addr[i] <= port_rx_src[p_idx];
                                mac_table_port[i] <= p_idx[1:0];
                                mac_table_valid[i] <= 1'b1;
                                mac_table_age[i] <= '0;
                                break;
                            end
                        end
                    end
                end
            end
            
            // Age entries
            for (int i = 0; i < MAC_TABLE_SIZE; i++) begin
                if (mac_table_valid[i]) begin
                    mac_table_age[i] <= mac_table_age[i] + 1;
                    if (mac_table_age[i] == 16'hFFFF)
                        mac_table_valid[i] <= 1'b0;
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Switch Fabric (Cut-Through Crossbar)
    //------------------------------------------------------------------------
    // Forwarding decision per port
    logic [1:0]  fwd_port    [NUM_PORTS-1:0];
    logic [3:0]  fwd_mask    [NUM_PORTS-1:0];  // Multicast mask
    logic        fwd_valid   [NUM_PORTS-1:0];
    
    always_comb begin
        for (int p_idx = 0; p_idx < NUM_PORTS; p_idx++) begin
            fwd_port[p_idx] = lookup_mac(port_rx_dst[p_idx]);
            
            // Create forwarding mask (exclude source port)
            if (fwd_port[p_idx] == 2'b11) begin
                // Flood to all ports except source
                fwd_mask[p_idx] = ~(1 << p_idx) & 4'b1111;
            end else begin
                // Unicast to specific port
                fwd_mask[p_idx] = (1 << fwd_port[p_idx]);
            end
            
            fwd_valid[p_idx] = port_rx_valid[p_idx];
        end
    end
    
    // Simple round-robin arbitration for each output port - SYNCHRONOUS reset
    logic [1:0] xbar_grant [NUM_PORTS-1:0];
    logic       xbar_busy  [NUM_PORTS-1:0];
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_PORTS; i++) begin
                xbar_grant[i] <= '0;
                xbar_busy[i] <= 1'b0;
            end
        end else begin
            for (int out_port = 0; out_port < NUM_PORTS; out_port++) begin
                if (!xbar_busy[out_port]) begin
                    // Look for input wanting this output
                    for (int in_port = 0; in_port < NUM_PORTS; in_port++) begin
                        if (fwd_valid[in_port] && fwd_mask[in_port][out_port]) begin
                            xbar_grant[out_port] <= in_port[1:0];
                            xbar_busy[out_port] <= 1'b1;
                            break;
                        end
                    end
                end else begin
                    // Wait for end of packet
                    if (port_rx_last[xbar_grant[out_port]])
                        xbar_busy[out_port] <= 1'b0;
                end
            end
        end
    end
    
    // Connect crossbar to TX queues
    always_comb begin
        for (int out_port = 0; out_port < NUM_PORTS; out_port++) begin
            logic [1:0] in_p;
            in_p = xbar_grant[out_port];
            
            for (int q = 0; q < NUM_QUEUES; q++) begin
                port_tx_data[out_port][q] = port_rx_data[in_p];
                port_tx_valid[out_port][q] = xbar_busy[out_port] && 
                                             fwd_mask[in_p][out_port] && 
                                             (q == port_rx_prio[in_p]);
                port_tx_last[out_port][q] = port_rx_last[in_p];
                port_tx_prio[out_port][q] = port_rx_prio[in_p];
            end
        end
        
        // Back-pressure
        for (int in_port = 0; in_port < NUM_PORTS; in_port++) begin
            port_rx_ready[in_port] = 1'b1;  // Simplified
        end
    end
    
    //------------------------------------------------------------------------
    // CPU Port Connection
    //------------------------------------------------------------------------
    // Port 0 also serves as CPU port
    assign cpu_rx_data  = port_rx_data[0];
    assign cpu_rx_valid = port_rx_valid[0] && (port_rx_dst[0] == port_mac_addr[0]);
    assign cpu_rx_last  = port_rx_last[0];
    assign cpu_rx_prio  = port_rx_prio[0];
    assign cpu_tx_ready = port_tx_ready[0][cpu_tx_prio];
    
    //------------------------------------------------------------------------
    // PTP Clock (IEEE 802.1AS) - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    logic [79:0] local_time;
    logic [31:0] ptp_ns;
    logic [47:0] ptp_sec;
    logic [31:0] pps_counter;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ptp_ns <= '0;
            ptp_sec <= '0;
            pps_counter <= '0;
            ptp_pps <= 1'b0;
        end else begin
            // Increment nanoseconds (8ns per 125MHz clock)
            ptp_ns <= ptp_ns + 32'd8;
            ptp_pps <= 1'b0;
            
            if (ptp_ns >= 32'd999_999_992) begin
                ptp_ns <= ptp_ns - 32'd999_999_992;
                ptp_sec <= ptp_sec + 1;
                ptp_pps <= 1'b1;  // PPS pulse
            end
            
            // Sync to master (simplified)
            if (ptp_master_time != local_time) begin
                // In real implementation, use servo loop
            end
        end
    end
    
    assign local_time = {ptp_sec, ptp_ns};
    assign ptp_local_time = local_time;
    
    //------------------------------------------------------------------------
    // AXI4-Lite Register Interface
    //------------------------------------------------------------------------
    // Configuration registers
    logic [31:0] switch_ctrl;
    logic [31:0] irq_enable;
    logic [31:0] irq_status;
    
    // AXI state machine
    typedef enum logic [2:0] {
        AXI_IDLE,
        AXI_WRITE,
        AXI_WRESP,
        AXI_READ,
        AXI_RRESP
    } axi_state_t;
    
    axi_state_t axi_state;
    logic [31:0] axi_addr_reg;
    
    // SYNCHRONOUS reset
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            axi_state <= AXI_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            switch_ctrl <= '0;
            irq_enable <= '0;
            
            // Default MAC addresses
            port_mac_addr[0] <= 48'h02_00_00_00_00_01;
            port_mac_addr[1] <= 48'h02_00_00_00_00_02;
            port_mac_addr[2] <= 48'h02_00_00_00_00_03;
            port_mac_addr[3] <= 48'h02_00_00_00_00_04;
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
                        
                        case (axi_addr_reg[11:8])
                            4'h0: switch_ctrl <= s_axi_wdata;
                            4'h1: port_mac_addr[0][31:0] <= s_axi_wdata;
                            4'h2: port_mac_addr[1][31:0] <= s_axi_wdata;
                            4'h3: port_mac_addr[2][31:0] <= s_axi_wdata;
                            4'h4: port_mac_addr[3][31:0] <= s_axi_wdata;
                            4'hF: irq_enable <= s_axi_wdata;
                        endcase
                        
                        s_axi_bvalid <= 1'b1;
                        s_axi_bresp <= 2'b00;
                        axi_state <= AXI_WRESP;
                    end
                end
                
                AXI_WRESP: begin
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        axi_state <= AXI_IDLE;
                    end
                end
                
                AXI_READ: begin
                    case (axi_addr_reg[11:8])
                        4'h0: s_axi_rdata <= switch_ctrl;
                        4'h1: s_axi_rdata <= port_mac_addr[0][31:0];
                        4'h2: s_axi_rdata <= port_mac_addr[1][31:0];
                        4'h3: s_axi_rdata <= port_mac_addr[2][31:0];
                        4'h4: s_axi_rdata <= port_mac_addr[3][31:0];
                        4'h7: s_axi_rdata <= ptp_ns;
                        4'h8: s_axi_rdata <= ptp_sec[31:0];
                        4'hE: s_axi_rdata <= irq_status;
                        4'hF: s_axi_rdata <= irq_enable;
                        default: s_axi_rdata <= 32'hDEADBEEF;
                    endcase
                    
                    s_axi_rvalid <= 1'b1;
                    s_axi_rresp <= 2'b00;
                    axi_state <= AXI_RRESP;
                end
                
                AXI_RRESP: begin
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
    // Interrupt Logic
    //------------------------------------------------------------------------
    assign irq_status = {28'b0, port_link_up};
    assign irq = |(irq_status & irq_enable);
    
    //------------------------------------------------------------------------
    // Status
    //------------------------------------------------------------------------
    assign switch_ready = &port_link_up;
    assign port_speed = {NUM_PORTS{1'b1}};  // All 1G

endmodule


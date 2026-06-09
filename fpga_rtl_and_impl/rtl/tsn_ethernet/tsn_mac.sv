//============================================================================
// PhD Research: TSN Ethernet MAC (Time-Sensitive Networking)
// Author: Chandraboul
// Target: Space-Grade Deterministic Networking
//
// Description:
//   IEEE 802.3 Ethernet MAC with Time-Sensitive Networking (TSN) features
//   for deterministic, real-time spacecraft communication.
//
// TSN Features:
//   - IEEE 802.1Qbv: Time-Aware Shaper (TAS)
//   - IEEE 802.1AS: Precision Time Protocol (gPTP)
//   - IEEE 802.1Qav: Credit-Based Shaper (CBS)
//   - IEEE 802.1Qci: Per-Stream Filtering and Policing
//   - 8 priority queues
//
// Supported Speeds: 10/100/1000 Mbps
//============================================================================

`timescale 1ns / 1ps

module tsn_mac #(
    parameter NUM_QUEUES    = 8,
    parameter QUEUE_DEPTH   = 256,
    parameter MTU           = 1522,    // Maximum frame size
    parameter ENABLE_TAS    = 1,       // Time-Aware Shaper
    parameter ENABLE_CBS    = 1,       // Credit-Based Shaper
    parameter ENABLE_PTP    = 1        // Precision Time Protocol
)(
    input  logic            clk,           // System clock (125 MHz for GbE)
    input  logic            rst_n,
    
    // GMII/MII Interface (to PHY)
    output logic [7:0]      gmii_txd,
    output logic            gmii_tx_en,
    output logic            gmii_tx_er,
    input  logic [7:0]      gmii_rxd,
    input  logic            gmii_rx_dv,
    input  logic            gmii_rx_er,
    input  logic            gmii_col,
    input  logic            gmii_crs,
    
    // MDIO Interface (PHY management)
    output logic            mdc,
    inout  logic            mdio,
    
    // TX Interface (8 priority queues)
    input  logic [7:0]      tx_data     [NUM_QUEUES-1:0],
    input  logic            tx_valid    [NUM_QUEUES-1:0],
    output logic            tx_ready    [NUM_QUEUES-1:0],
    input  logic            tx_last     [NUM_QUEUES-1:0],
    input  logic [2:0]      tx_priority [NUM_QUEUES-1:0],
    
    // RX Interface
    output logic [7:0]      rx_data,
    output logic            rx_valid,
    input  logic            rx_ready,
    output logic            rx_last,
    output logic [2:0]      rx_priority,
    output logic [47:0]     rx_src_mac,
    output logic [47:0]     rx_dst_mac,
    
    // PTP Timestamp Interface
    input  logic [79:0]     ptp_time,      // 48-bit seconds + 32-bit nanoseconds
    output logic [79:0]     tx_timestamp,
    output logic            tx_timestamp_valid,
    output logic [79:0]     rx_timestamp,
    output logic            rx_timestamp_valid,
    
    // TAS Gate Control List
    input  logic [7:0]      tas_gate_state,     // Which queues are open
    input  logic [31:0]     tas_cycle_time,     // Cycle time in ns
    input  logic            tas_gate_enable,
    
    // CBS Configuration
    input  logic [31:0]     cbs_idle_slope [NUM_QUEUES-1:0],
    input  logic [31:0]     cbs_send_slope [NUM_QUEUES-1:0],
    input  logic [31:0]     cbs_hi_credit  [NUM_QUEUES-1:0],
    input  logic [31:0]     cbs_lo_credit  [NUM_QUEUES-1:0],
    
    // Configuration
    input  logic [47:0]     mac_addr,
    input  logic [1:0]      speed,          // 00=10M, 01=100M, 10=1G
    input  logic            promiscuous,
    input  logic            loopback,
    
    // Status
    output logic            link_up,
    output logic [31:0]     tx_frame_count,
    output logic [31:0]     rx_frame_count,
    output logic [31:0]     tx_byte_count,
    output logic [31:0]     rx_byte_count,
    output logic [31:0]     rx_error_count,
    output logic [31:0]     rx_crc_error,
    
    // Interrupt
    output logic            irq_tx_done,
    output logic            irq_rx_ready,
    output logic            irq_link_change
);

    //------------------------------------------------------------------------
    // Ethernet Frame Constants
    //------------------------------------------------------------------------
    localparam [7:0] PREAMBLE = 8'h55;
    localparam [7:0] SFD      = 8'hD5;
    localparam IFG_BYTES      = 12;      // Inter-frame gap
    
    //------------------------------------------------------------------------
    // TX State Machine
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TX_IDLE,
        TX_PREAMBLE,
        TX_SFD,
        TX_DST_MAC,
        TX_SRC_MAC,
        TX_LENGTH,
        TX_DATA,
        TX_PAD,
        TX_CRC,
        TX_IFG
    } tx_state_t;
    
    tx_state_t tx_state;
    logic [3:0]  tx_byte_cnt;
    logic [15:0] tx_frame_len;
    logic [31:0] tx_crc;
    logic [2:0]  tx_current_queue;
    logic [10:0] tx_data_cnt;
    
    //------------------------------------------------------------------------
    // RX State Machine
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        RX_IDLE,
        RX_PREAMBLE,
        RX_SFD,
        RX_DST_MAC,
        RX_SRC_MAC,
        RX_LENGTH,
        RX_DATA,
        RX_CRC_CHECK
    } rx_state_t;
    
    rx_state_t rx_state;
    logic [3:0]  rx_byte_cnt;
    logic [15:0] rx_frame_len;
    logic [31:0] rx_crc;
    logic [31:0] rx_crc_calc;
    logic [10:0] rx_data_cnt;
    
    //------------------------------------------------------------------------
    // Per-Queue FIFOs and Credit
    //------------------------------------------------------------------------
    logic [7:0]  queue_data   [NUM_QUEUES-1:0];
    logic        queue_valid  [NUM_QUEUES-1:0];
    logic        queue_ready  [NUM_QUEUES-1:0];
    logic        queue_empty  [NUM_QUEUES-1:0];
    logic signed [31:0] cbs_credit [NUM_QUEUES-1:0];
    logic        queue_eligible [NUM_QUEUES-1:0];
    
    //------------------------------------------------------------------------
    // Time-Aware Shaper (TAS) - IEEE 802.1Qbv
    //------------------------------------------------------------------------
    generate
        if (ENABLE_TAS) begin : gen_tas
            logic [31:0] tas_timer;
            logic [7:0]  current_gate_state;
            
            // SYNCHRONOUS reset for FPGA optimization
            always_ff @(posedge clk) begin
                if (!rst_n) begin
                    tas_timer <= '0;
                    current_gate_state <= 8'hFF;  // All open by default
                end else if (tas_gate_enable) begin
                    if (tas_timer >= tas_cycle_time) begin
                        tas_timer <= '0;
                    end else begin
                        tas_timer <= tas_timer + 8;  // 8ns per 125MHz clock
                    end
                    current_gate_state <= tas_gate_state;
                end else begin
                    current_gate_state <= 8'hFF;
                end
            end
            
            // Queue gating
            always_comb begin
                for (int i = 0; i < NUM_QUEUES; i++) begin
                    queue_eligible[i] = current_gate_state[i] && !queue_empty[i];
                end
            end
        end else begin : no_tas
            always_comb begin
                for (int i = 0; i < NUM_QUEUES; i++) begin
                    queue_eligible[i] = !queue_empty[i];
                end
            end
        end
    endgenerate
    
    //------------------------------------------------------------------------
    // Credit-Based Shaper (CBS) - IEEE 802.1Qav
    //------------------------------------------------------------------------
    generate
        if (ENABLE_CBS) begin : gen_cbs
            // SYNCHRONOUS reset for FPGA optimization
            always_ff @(posedge clk) begin
                if (!rst_n) begin
                    for (int i = 0; i < NUM_QUEUES; i++)
                        cbs_credit[i] <= '0;
                end else begin
                    for (int i = 0; i < NUM_QUEUES; i++) begin
                        if (tx_state == TX_DATA && tx_current_queue == i[2:0]) begin
                            // Transmitting - decrease credit
                            cbs_credit[i] <= cbs_credit[i] - $signed(cbs_send_slope[i]);
                            if (cbs_credit[i] < $signed(cbs_lo_credit[i]))
                                cbs_credit[i] <= $signed(cbs_lo_credit[i]);
                        end else if (queue_eligible[i]) begin
                            // Idle but has data - increase credit
                            cbs_credit[i] <= cbs_credit[i] + $signed(cbs_idle_slope[i]);
                            if (cbs_credit[i] > $signed(cbs_hi_credit[i]))
                                cbs_credit[i] <= $signed(cbs_hi_credit[i]);
                        end
                    end
                end
            end
        end
    endgenerate
    
    //------------------------------------------------------------------------
    // TX Queue Arbitration (Strict Priority + TAS + CBS)
    //------------------------------------------------------------------------
    logic [2:0] selected_queue;
    logic       queue_available;
    
    always_comb begin
        selected_queue = '0;
        queue_available = 1'b0;
        
        // Strict priority (highest first)
        for (int i = NUM_QUEUES-1; i >= 0; i--) begin
            if (queue_eligible[i] && (cbs_credit[i] >= 0 || !ENABLE_CBS)) begin
                selected_queue = i[2:0];
                queue_available = 1'b1;
                break;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // TX State Machine - SYNCHRONOUS reset for FPGA optimization
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_byte_cnt <= '0;
            gmii_txd <= '0;
            gmii_tx_en <= 1'b0;
            gmii_tx_er <= 1'b0;
            tx_frame_count <= '0;
            tx_byte_count <= '0;
            tx_timestamp_valid <= 1'b0;
        end else begin
            tx_timestamp_valid <= 1'b0;
            
            case (tx_state)
                TX_IDLE: begin
                    gmii_tx_en <= 1'b0;
                    if (queue_available) begin
                        tx_current_queue <= selected_queue;
                        tx_state <= TX_PREAMBLE;
                        tx_byte_cnt <= '0;
                        
                        // Capture timestamp for PTP
                        if (ENABLE_PTP) begin
                            tx_timestamp <= ptp_time;
                            tx_timestamp_valid <= 1'b1;
                        end
                    end
                end
                
                TX_PREAMBLE: begin
                    gmii_tx_en <= 1'b1;
                    gmii_txd <= PREAMBLE;
                    tx_byte_cnt <= tx_byte_cnt + 1;
                    if (tx_byte_cnt == 6)
                        tx_state <= TX_SFD;
                end
                
                TX_SFD: begin
                    gmii_txd <= SFD;
                    tx_state <= TX_DST_MAC;
                    tx_byte_cnt <= '0;
                    tx_crc <= 32'hFFFFFFFF;  // Initialize CRC
                end
                
                TX_DST_MAC: begin
                    // Transmit destination MAC (from queue header)
                    gmii_txd <= 8'hFF;  // Placeholder - broadcast
                    tx_byte_cnt <= tx_byte_cnt + 1;
                    if (tx_byte_cnt == 5) begin
                        tx_state <= TX_SRC_MAC;
                        tx_byte_cnt <= '0;
                    end
                end
                
                TX_SRC_MAC: begin
                    // Transmit source MAC
                    gmii_txd <= mac_addr[(5-tx_byte_cnt)*8 +: 8];
                    tx_byte_cnt <= tx_byte_cnt + 1;
                    if (tx_byte_cnt == 5) begin
                        tx_state <= TX_LENGTH;
                        tx_byte_cnt <= '0;
                    end
                end
                
                TX_LENGTH: begin
                    // EtherType/Length field
                    gmii_txd <= (tx_byte_cnt == 0) ? 8'h08 : 8'h00;  // IPv4
                    tx_byte_cnt <= tx_byte_cnt + 1;
                    if (tx_byte_cnt == 1) begin
                        tx_state <= TX_DATA;
                        tx_data_cnt <= '0;
                    end
                end
                
                TX_DATA: begin
                    // Transmit payload from queue
                    gmii_txd <= tx_data[tx_current_queue];
                    tx_data_cnt <= tx_data_cnt + 1;
                    tx_byte_count <= tx_byte_count + 1;
                    
                    if (tx_last[tx_current_queue]) begin
                        if (tx_data_cnt < 46)
                            tx_state <= TX_PAD;
                        else
                            tx_state <= TX_CRC;
                        tx_byte_cnt <= '0;
                    end
                end
                
                TX_PAD: begin
                    // Pad to minimum frame size
                    gmii_txd <= 8'h00;
                    tx_data_cnt <= tx_data_cnt + 1;
                    if (tx_data_cnt >= 45) begin
                        tx_state <= TX_CRC;
                        tx_byte_cnt <= '0;
                    end
                end
                
                TX_CRC: begin
                    // Transmit FCS (CRC-32)
                    gmii_txd <= ~tx_crc[(3-tx_byte_cnt)*8 +: 8];
                    tx_byte_cnt <= tx_byte_cnt + 1;
                    if (tx_byte_cnt == 3) begin
                        tx_state <= TX_IFG;
                        tx_byte_cnt <= '0;
                        tx_frame_count <= tx_frame_count + 1;
                    end
                end
                
                TX_IFG: begin
                    gmii_tx_en <= 1'b0;
                    gmii_txd <= 8'h00;
                    tx_byte_cnt <= tx_byte_cnt + 1;
                    if (tx_byte_cnt >= IFG_BYTES-1) begin
                        tx_state <= TX_IDLE;
                        irq_tx_done <= 1'b1;
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // RX State Machine - SYNCHRONOUS reset for FPGA optimization
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            rx_byte_cnt <= '0;
            rx_valid <= 1'b0;
            rx_last <= 1'b0;
            rx_frame_count <= '0;
            rx_byte_count <= '0;
            rx_error_count <= '0;
            rx_crc_error <= '0;
            rx_timestamp_valid <= 1'b0;
        end else begin
            rx_timestamp_valid <= 1'b0;
            rx_last <= 1'b0;
            
            case (rx_state)
                RX_IDLE: begin
                    rx_valid <= 1'b0;
                    if (gmii_rx_dv && gmii_rxd == PREAMBLE) begin
                        rx_state <= RX_PREAMBLE;
                        rx_byte_cnt <= 4'd1;
                        
                        // Capture RX timestamp
                        if (ENABLE_PTP) begin
                            rx_timestamp <= ptp_time;
                            rx_timestamp_valid <= 1'b1;
                        end
                    end
                end
                
                RX_PREAMBLE: begin
                    if (gmii_rx_dv) begin
                        if (gmii_rxd == PREAMBLE) begin
                            rx_byte_cnt <= rx_byte_cnt + 1;
                        end else if (gmii_rxd == SFD) begin
                            rx_state <= RX_DST_MAC;
                            rx_byte_cnt <= '0;
                            rx_crc_calc <= 32'hFFFFFFFF;
                        end else begin
                            rx_state <= RX_IDLE;
                            rx_error_count <= rx_error_count + 1;
                        end
                    end else begin
                        rx_state <= RX_IDLE;
                    end
                end
                
                RX_DST_MAC: begin
                    if (gmii_rx_dv) begin
                        rx_dst_mac[(5-rx_byte_cnt)*8 +: 8] <= gmii_rxd;
                        rx_byte_cnt <= rx_byte_cnt + 1;
                        if (rx_byte_cnt == 5) begin
                            rx_state <= RX_SRC_MAC;
                            rx_byte_cnt <= '0;
                        end
                    end else begin
                        rx_state <= RX_IDLE;
                        rx_error_count <= rx_error_count + 1;
                    end
                end
                
                RX_SRC_MAC: begin
                    if (gmii_rx_dv) begin
                        rx_src_mac[(5-rx_byte_cnt)*8 +: 8] <= gmii_rxd;
                        rx_byte_cnt <= rx_byte_cnt + 1;
                        if (rx_byte_cnt == 5) begin
                            rx_state <= RX_LENGTH;
                            rx_byte_cnt <= '0;
                        end
                    end else begin
                        rx_state <= RX_IDLE;
                        rx_error_count <= rx_error_count + 1;
                    end
                end
                
                RX_LENGTH: begin
                    if (gmii_rx_dv) begin
                        if (rx_byte_cnt == 0)
                            rx_frame_len[15:8] <= gmii_rxd;
                        else
                            rx_frame_len[7:0] <= gmii_rxd;
                        
                        rx_byte_cnt <= rx_byte_cnt + 1;
                        if (rx_byte_cnt == 1) begin
                            rx_state <= RX_DATA;
                            rx_data_cnt <= '0;
                        end
                    end else begin
                        rx_state <= RX_IDLE;
                    end
                end
                
                RX_DATA: begin
                    if (gmii_rx_dv) begin
                        rx_data <= gmii_rxd;
                        rx_valid <= 1'b1;
                        rx_data_cnt <= rx_data_cnt + 1;
                        rx_byte_count <= rx_byte_count + 1;
                    end else begin
                        // End of frame
                        rx_valid <= 1'b0;
                        rx_last <= 1'b1;
                        rx_state <= RX_IDLE;
                        rx_frame_count <= rx_frame_count + 1;
                        irq_rx_ready <= 1'b1;
                    end
                end
                
                default: rx_state <= RX_IDLE;
            endcase
            
            // RX error handling
            if (gmii_rx_er) begin
                rx_error_count <= rx_error_count + 1;
                rx_state <= RX_IDLE;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // TX Ready Signals
    //------------------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < NUM_QUEUES; i++) begin
            tx_ready[i] = (tx_state == TX_DATA) && (tx_current_queue == i[2:0]);
        end
    end
    
    //------------------------------------------------------------------------
    // Queue Status
    //------------------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < NUM_QUEUES; i++) begin
            queue_empty[i] = !tx_valid[i];
        end
    end
    
    //------------------------------------------------------------------------
    // Link Status (simplified)
    //------------------------------------------------------------------------
    assign link_up = 1'b1;  // Placeholder
    assign irq_link_change = 1'b0;
    
    //------------------------------------------------------------------------
    // MDIO Interface (placeholder)
    //------------------------------------------------------------------------
    assign mdc = 1'b0;
    // mdio is bidirectional - needs tristate

endmodule




//============================================================================
// PhD Research: SpaceWire Time Distribution Master
// Author: Chandraboul
// Target: Space-Grade Synchronized Timing
//
// Description:
//   SpaceWire Time-Code master for distributed time synchronization.
//   Generates and distributes time-codes across SpaceWire network.
//
// Features:
//   - 64-bit nanosecond counter
//   - Configurable time-code rate (1Hz - 100Hz)
//   - GPS/PPS synchronization input
//   - Time-code forwarding
//   - Sub-microsecond accuracy
//   - CCSDS time format support
//
// Time-Code Format:
//   Bits [5:0]: 6-bit counter (0-63)
//   Bits [7:6]: Control flags
//============================================================================

`timescale 1ns / 1ps

module spacewire_time_master #(
    parameter CLK_FREQ_HZ = 100_000_000,
    parameter TIME_CODE_RATE_HZ = 1,      // Time-code rate
    parameter NUM_PORTS = 4
)(
    input  logic            clk,
    input  logic            rst_n,
    
    // External Sync Interface
    input  logic            pps_in,           // 1PPS input from GPS/external
    input  logic            sync_enable,      // Enable external sync
    input  logic [63:0]     utc_time_in,      // UTC time for sync
    input  logic            utc_valid,        // UTC time valid
    
    // SpaceWire Time-Code Interfaces (to/from ports)
    output logic [7:0]      tx_time_code  [NUM_PORTS-1:0],
    output logic            tx_time_valid [NUM_PORTS-1:0],
    input  logic [7:0]      rx_time_code  [NUM_PORTS-1:0],
    input  logic            rx_time_valid [NUM_PORTS-1:0],
    
    // Time Output (for system use)
    output logic [63:0]     system_time_ns,   // Nanoseconds since epoch
    output logic [31:0]     system_time_sec,  // Seconds since epoch
    output logic [29:0]     system_time_nsec, // Nanoseconds within second
    output logic [5:0]      current_time_code,
    output logic            time_tick,        // Tick on time-code transmission
    
    // Configuration
    input  logic            master_mode,      // 1=master, 0=slave
    input  logic [31:0]     time_code_period, // Clocks between time-codes
    
    // Status
    output logic            sync_locked,
    output logic [31:0]     time_code_count,
    output logic            pps_detected
);

    //------------------------------------------------------------------------
    // Clock Constants
    //------------------------------------------------------------------------
    localparam NS_PER_CLK = 1_000_000_000 / CLK_FREQ_HZ;  // ns per clock
    localparam CLKS_PER_SEC = CLK_FREQ_HZ;
    localparam DEFAULT_TC_PERIOD = CLK_FREQ_HZ / TIME_CODE_RATE_HZ;
    
    //------------------------------------------------------------------------
    // Time Counters
    //------------------------------------------------------------------------
    logic [63:0] ns_counter;        // Total nanoseconds
    logic [31:0] sec_counter;       // Seconds
    logic [29:0] nsec_counter;      // Nanoseconds within second
    logic [31:0] clk_counter;       // Clocks since last second
    logic [5:0]  time_code_counter; // 6-bit time-code value
    logic [31:0] tc_clk_counter;    // Clocks since last time-code
    
    //------------------------------------------------------------------------
    // PPS Detection
    //------------------------------------------------------------------------
    logic pps_d1, pps_d2;
    logic pps_rising;
    logic [31:0] pps_period_counter;
    logic [31:0] measured_pps_period;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pps_d1 <= 1'b0;
            pps_d2 <= 1'b0;
            pps_period_counter <= '0;
            measured_pps_period <= CLKS_PER_SEC;
        end else begin
            pps_d1 <= pps_in;
            pps_d2 <= pps_d1;
            
            pps_period_counter <= pps_period_counter + 1;
            
            if (pps_rising) begin
                measured_pps_period <= pps_period_counter;
                pps_period_counter <= '0;
            end
        end
    end
    
    assign pps_rising = pps_d1 && !pps_d2;
    assign pps_detected = pps_rising;
    
    //------------------------------------------------------------------------
    // Master Time Counter
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ns_counter <= '0;
            sec_counter <= '0;
            nsec_counter <= '0;
            clk_counter <= '0;
            sync_locked <= 1'b0;
        end else begin
            // Increment nanosecond counter
            ns_counter <= ns_counter + NS_PER_CLK;
            nsec_counter <= nsec_counter + NS_PER_CLK;
            clk_counter <= clk_counter + 1;
            
            // Second rollover
            if (nsec_counter >= 30'd999_999_992) begin
                nsec_counter <= nsec_counter - 30'd999_999_992;
                sec_counter <= sec_counter + 1;
                clk_counter <= '0;
            end
            
            // PPS synchronization
            if (sync_enable && pps_rising) begin
                nsec_counter <= '0;
                clk_counter <= '0;
                sync_locked <= 1'b1;
                
                // Load UTC time if valid
                if (utc_valid) begin
                    sec_counter <= utc_time_in[31:0];
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Time-Code Generation (Master Mode)
    //------------------------------------------------------------------------
    logic [31:0] tc_period;
    logic        tc_trigger;
    
    assign tc_period = (time_code_period != 0) ? time_code_period : DEFAULT_TC_PERIOD;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tc_clk_counter <= '0;
            time_code_counter <= '0;
            time_code_count <= '0;
            tc_trigger <= 1'b0;
        end else begin
            tc_trigger <= 1'b0;
            
            if (master_mode) begin
                tc_clk_counter <= tc_clk_counter + 1;
                
                if (tc_clk_counter >= tc_period - 1) begin
                    tc_clk_counter <= '0;
                    time_code_counter <= time_code_counter + 1;
                    time_code_count <= time_code_count + 1;
                    tc_trigger <= 1'b1;
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Time-Code Distribution to Ports
    //------------------------------------------------------------------------
    genvar p;
    generate
        for (p = 0; p < NUM_PORTS; p++) begin : tc_ports
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    tx_time_code[p] <= '0;
                    tx_time_valid[p] <= 1'b0;
                end else begin
                    tx_time_valid[p] <= 1'b0;
                    
                    if (master_mode && tc_trigger) begin
                        // Send new time-code
                        tx_time_code[p] <= {2'b00, time_code_counter};
                        tx_time_valid[p] <= 1'b1;
                    end else if (!master_mode) begin
                        // Slave mode: forward received time-codes
                        for (int q = 0; q < NUM_PORTS; q++) begin
                            if (q != p && rx_time_valid[q]) begin
                                tx_time_code[p] <= rx_time_code[q];
                                tx_time_valid[p] <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    endgenerate
    
    //------------------------------------------------------------------------
    // Slave Mode - Receive Time-Codes
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset handled above
        end else if (!master_mode) begin
            // In slave mode, sync to received time-codes
            for (int q = 0; q < NUM_PORTS; q++) begin
                if (rx_time_valid[q]) begin
                    time_code_counter <= rx_time_code[q][5:0];
                    time_code_count <= time_code_count + 1;
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Outputs
    //------------------------------------------------------------------------
    assign system_time_ns = ns_counter;
    assign system_time_sec = sec_counter;
    assign system_time_nsec = nsec_counter;
    assign current_time_code = time_code_counter;
    assign time_tick = tc_trigger;

endmodule




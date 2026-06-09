//============================================================================
// PhD Research: Watchdog Timer
// Author: Chandraboul
// Target: Space-Grade Fault-Tolerant Timer
//
// Description:
//   Watchdog timer with configurable timeout, warning threshold,
//   and automatic system reset on timeout.
//
// Features:
//   - Configurable timeout period
//   - Warning interrupt before timeout
//   - Software kick to reset counter
//   - Enable/disable control
//   - TMR-protected counter (optional)
//============================================================================

`timescale 1ns / 1ps

module watchdog_timer #(
    parameter TIMEOUT_CYCLES = 100_000_000,  // Default 1 second at 100MHz
    parameter WARNING_CYCLES = 10_000_000,   // Warning 100ms before timeout
    parameter ENABLE_TMR     = 0             // TMR protection
)(
    input  logic            clk,
    input  logic            rst_n,
    
    // Control
    input  logic            kick,       // Reset counter
    input  logic            enable,     // Enable watchdog
    
    // Status
    output logic            timeout,    // Timeout occurred
    output logic            warning,    // Warning before timeout
    output logic [31:0]     count       // Current counter value
);

    //------------------------------------------------------------------------
    // Counter Logic
    //------------------------------------------------------------------------
    logic [31:0] counter;
    logic [31:0] timeout_value;
    logic [31:0] warning_value;
    
    assign timeout_value = TIMEOUT_CYCLES;
    assign warning_value = TIMEOUT_CYCLES - WARNING_CYCLES;
    
    generate
        if (ENABLE_TMR) begin : gen_tmr
            // Triple Modular Redundancy
            logic [31:0] counter_a, counter_b, counter_c;
            
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    counter_a <= '0;
                    counter_b <= '0;
                    counter_c <= '0;
                end else if (kick) begin
                    counter_a <= '0;
                    counter_b <= '0;
                    counter_c <= '0;
                end else if (enable) begin
                    counter_a <= counter_a + 1;
                    counter_b <= counter_b + 1;
                    counter_c <= counter_c + 1;
                end
            end
            
            // Majority voter
            always_comb begin
                if (counter_a == counter_b)
                    counter = counter_a;
                else if (counter_a == counter_c)
                    counter = counter_a;
                else
                    counter = counter_b;
            end
            
        end else begin : gen_simple
            // Simple counter
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    counter <= '0;
                end else if (kick) begin
                    counter <= '0;
                end else if (enable) begin
                    counter <= counter + 1;
                end
            end
            
            assign count = counter;
        end
    endgenerate
    
    assign count = counter;
    
    //------------------------------------------------------------------------
    // Timeout and Warning Detection
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeout <= 1'b0;
            warning <= 1'b0;
        end else begin
            timeout <= (counter >= timeout_value) && enable;
            warning <= (counter >= warning_value) && (counter < timeout_value) && enable;
        end
    end

endmodule




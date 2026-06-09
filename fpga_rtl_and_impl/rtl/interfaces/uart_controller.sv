//============================================================================
// UART Controller - 115200 baud
//============================================================================

`timescale 1ns / 1ps

module uart_controller (
    input  logic        clk,
    input  logic        rst_n,
    
    // UART pins
    input  logic        rx,
    output logic        tx,
    
    // Configuration
    input  logic [15:0] baud_div,
    
    // Bus interface
    input  logic        bus_valid,
    input  logic [7:0]  bus_wdata,
    output logic [7:0]  bus_rdata,
    output logic        bus_ready
);

    // TX state machine
    typedef enum logic [2:0] {
        TX_IDLE,
        TX_START,
        TX_DATA,
        TX_STOP
    } tx_state_t;
    
    tx_state_t tx_state;
    logic [15:0] tx_counter;
    logic [7:0] tx_shift;
    logic [2:0] tx_bit;
    
    // RX state machine
    typedef enum logic [2:0] {
        RX_IDLE,
        RX_START,
        RX_DATA,
        RX_STOP
    } rx_state_t;
    
    rx_state_t rx_state;
    logic [15:0] rx_counter;
    logic [7:0] rx_shift;
    logic [2:0] rx_bit;
    logic [7:0] rx_data;
    logic rx_valid;
    
    // TX Logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_counter <= 16'd0;
            tx_shift <= 8'hFF;
            tx_bit <= 3'd0;
            tx <= 1'b1;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx <= 1'b1;
                    if (bus_valid) begin
                        tx_shift <= bus_wdata;
                        tx_state <= TX_START;
                        tx_counter <= baud_div;
                    end
                end
                
                TX_START: begin
                    tx <= 1'b0;  // Start bit
                    if (tx_counter == 0) begin
                        tx_state <= TX_DATA;
                        tx_counter <= baud_div;
                        tx_bit <= 3'd0;
                    end else begin
                        tx_counter <= tx_counter - 1;
                    end
                end
                
                TX_DATA: begin
                    tx <= tx_shift[tx_bit];
                    if (tx_counter == 0) begin
                        if (tx_bit == 3'd7) begin
                            tx_state <= TX_STOP;
                        end else begin
                            tx_bit <= tx_bit + 1;
                        end
                        tx_counter <= baud_div;
                    end else begin
                        tx_counter <= tx_counter - 1;
                    end
                end
                
                TX_STOP: begin
                    tx <= 1'b1;  // Stop bit
                    if (tx_counter == 0) begin
                        tx_state <= TX_IDLE;
                    end else begin
                        tx_counter <= tx_counter - 1;
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
    
    // RX Logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            rx_counter <= 16'd0;
            rx_shift <= 8'd0;
            rx_bit <= 3'd0;
            rx_data <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            
            case (rx_state)
                RX_IDLE: begin
                    if (!rx) begin  // Start bit detected
                        rx_state <= RX_START;
                        rx_counter <= {1'b0, baud_div[15:1]};  // Sample at middle
                    end
                end
                
                RX_START: begin
                    if (rx_counter == 0) begin
                        if (!rx) begin  // Verify start bit
                            rx_state <= RX_DATA;
                            rx_counter <= baud_div;
                            rx_bit <= 3'd0;
                        end else begin
                            rx_state <= RX_IDLE;
                        end
                    end else begin
                        rx_counter <= rx_counter - 1;
                    end
                end
                
                RX_DATA: begin
                    if (rx_counter == 0) begin
                        rx_shift[rx_bit] <= rx;
                        if (rx_bit == 3'd7) begin
                            rx_state <= RX_STOP;
                        end else begin
                            rx_bit <= rx_bit + 1;
                        end
                        rx_counter <= baud_div;
                    end else begin
                        rx_counter <= rx_counter - 1;
                    end
                end
                
                RX_STOP: begin
                    if (rx_counter == 0) begin
                        if (rx) begin  // Valid stop bit
                            rx_data <= rx_shift;
                            rx_valid <= 1'b1;
                        end
                        rx_state <= RX_IDLE;
                    end else begin
                        rx_counter <= rx_counter - 1;
                    end
                end
                
                default: rx_state <= RX_IDLE;
            endcase
        end
    end
    
    // Output
    assign bus_rdata = rx_data;
    assign bus_ready = (tx_state == TX_IDLE);

endmodule


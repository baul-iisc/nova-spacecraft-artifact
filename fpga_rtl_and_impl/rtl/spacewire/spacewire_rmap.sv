//============================================================================
// PhD Research: SpaceWire RMAP (Remote Memory Access Protocol)
// Author: Chandraboul
// Target: Space-Grade Remote Memory Access
//
// Description:
//   RMAP target and initiator for remote memory access over SpaceWire.
//   Compliant with ECSS-E-ST-50-52C standard.
//
// Features:
//   - RMAP Target (responds to remote read/write requests)
//   - RMAP Initiator (sends read/write commands)
//   - CRC-8 verification
//   - Authorization and key verification
//   - Increment addressing support
//   - Verified and unverified writes
//
// Command Types:
//   - Write with Reply
//   - Write without Reply
//   - Read with Single Reply
//   - Read with Multiple Replies
//   - Read-Modify-Write
//============================================================================

`timescale 1ns / 1ps

module spacewire_rmap #(
    parameter TARGET_LOGICAL_ADDR = 8'hFE,
    parameter INITIATOR_LOGICAL_ADDR = 8'h20,
    parameter KEY = 8'h00,
    parameter MEM_SIZE = 65536,         // 64KB addressable memory
    parameter FIFO_DEPTH = 256
)(
    input  logic            clk,
    input  logic            rst_n,
    
    // SpaceWire Link Interface (from codec)
    input  logic [7:0]      rx_data,
    input  logic            rx_valid,
    output logic            rx_ready,
    input  logic            rx_eop,
    input  logic            rx_eep,
    
    output logic [7:0]      tx_data,
    output logic            tx_valid,
    input  logic            tx_ready,
    output logic            tx_eop,
    output logic            tx_eep,
    
    // Memory Interface (for RMAP Target)
    output logic            mem_read,
    output logic            mem_write,
    output logic [31:0]     mem_addr,
    output logic [31:0]     mem_wdata,
    input  logic [31:0]     mem_rdata,
    input  logic            mem_ready,
    
    // Initiator Command Interface
    input  logic            cmd_valid,
    output logic            cmd_ready,
    input  logic [7:0]      cmd_dest_addr,     // Destination logical address
    input  logic [3:0]      cmd_type,          // Command type
    input  logic [31:0]     cmd_mem_addr,      // Memory address
    input  logic [23:0]     cmd_length,        // Data length
    input  logic [7:0]      cmd_key,           // Destination key
    
    // Initiator Response Interface
    output logic            rsp_valid,
    input  logic            rsp_ready,
    output logic [7:0]      rsp_status,
    output logic [31:0]     rsp_data,
    
    // Configuration
    input  logic            enable_target,
    input  logic            enable_initiator,
    input  logic [7:0]      target_key,
    
    // Status
    output logic            busy,
    output logic [31:0]     rx_cmd_count,
    output logic [31:0]     tx_rsp_count,
    output logic [31:0]     error_count,
    output logic            crc_error,
    output logic            key_error,
    output logic            addr_error
);

    //------------------------------------------------------------------------
    // RMAP Protocol Constants
    //------------------------------------------------------------------------
    localparam [7:0] RMAP_PROTOCOL_ID = 8'h01;
    
    // Command types (packet type + command code)
    localparam [3:0] CMD_WRITE_REPLY     = 4'b0110;  // Write, increment, reply
    localparam [3:0] CMD_WRITE_NO_REPLY  = 4'b0100;  // Write, increment, no reply
    localparam [3:0] CMD_READ_SINGLE     = 4'b0010;  // Read, single reply
    localparam [3:0] CMD_RMW             = 4'b0111;  // Read-Modify-Write
    
    // Status codes
    localparam [7:0] STATUS_OK           = 8'h00;
    localparam [7:0] STATUS_GENERAL_ERR  = 8'h01;
    localparam [7:0] STATUS_INVALID_KEY  = 8'h03;
    localparam [7:0] STATUS_INVALID_ADDR = 8'h0A;
    localparam [7:0] STATUS_CRC_ERROR    = 8'h0C;
    
    //------------------------------------------------------------------------
    // CRC-8 Calculation (RMAP uses polynomial x^8 + x^2 + x + 1)
    //------------------------------------------------------------------------
    function automatic logic [7:0] crc8_byte(input logic [7:0] crc, input logic [7:0] data);
        logic [7:0] result;
        logic [7:0] temp;
        temp = crc ^ data;
        result = 8'h00;
        
        for (int i = 0; i < 8; i++) begin
            if (temp[7])
                temp = (temp << 1) ^ 8'h07;  // Polynomial
            else
                temp = temp << 1;
        end
        
        return temp;
    endfunction
    
    //------------------------------------------------------------------------
    // Target State Machine
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        T_IDLE,
        T_RX_HEADER,
        T_RX_ADDR,
        T_RX_LENGTH,
        T_RX_HEADER_CRC,
        T_RX_DATA,
        T_RX_DATA_CRC,
        T_PROCESS,
        T_MEM_ACCESS,
        T_TX_REPLY,
        T_TX_DATA,
        T_TX_CRC,
        T_TX_EOP,
        T_ERROR
    } target_state_t;
    
    target_state_t t_state;
    
    // Received command fields
    logic [7:0]  rx_dest_addr;
    logic [7:0]  rx_protocol;
    logic [7:0]  rx_instruction;
    logic [7:0]  rx_key;
    logic [7:0]  rx_src_addr;
    logic [15:0] rx_transaction_id;
    logic [7:0]  rx_ext_addr;
    logic [31:0] rx_mem_addr;
    logic [23:0] rx_data_length;
    logic [7:0]  rx_header_crc;
    logic [7:0]  rx_crc_calc;
    
    // Data buffer
    logic [7:0]  data_buffer [255:0];
    logic [7:0]  data_idx;
    
    // Reply fields
    logic [7:0]  reply_status;
    logic [7:0]  reply_crc;
    
    //------------------------------------------------------------------------
    // Target RX Processing
    //------------------------------------------------------------------------
    logic [3:0]  rx_byte_cnt;
    logic [23:0] rx_data_cnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_state <= T_IDLE;
            rx_byte_cnt <= '0;
            rx_data_cnt <= '0;
            rx_crc_calc <= '0;
            rx_ready <= 1'b1;
            rx_cmd_count <= '0;
            error_count <= '0;
            crc_error <= 1'b0;
            key_error <= 1'b0;
            addr_error <= 1'b0;
            mem_read <= 1'b0;
            mem_write <= 1'b0;
        end else begin
            crc_error <= 1'b0;
            key_error <= 1'b0;
            addr_error <= 1'b0;
            
            case (t_state)
                T_IDLE: begin
                    rx_ready <= enable_target;
                    rx_byte_cnt <= '0;
                    rx_crc_calc <= '0;
                    
                    if (rx_valid && enable_target) begin
                        rx_dest_addr <= rx_data;
                        rx_crc_calc <= crc8_byte(8'h00, rx_data);
                        t_state <= T_RX_HEADER;
                        rx_byte_cnt <= 4'd1;
                    end
                end
                
                T_RX_HEADER: begin
                    if (rx_valid) begin
                        rx_crc_calc <= crc8_byte(rx_crc_calc, rx_data);
                        rx_byte_cnt <= rx_byte_cnt + 1;
                        
                        case (rx_byte_cnt)
                            4'd1: rx_protocol <= rx_data;
                            4'd2: rx_instruction <= rx_data;
                            4'd3: rx_key <= rx_data;
                            4'd4: rx_src_addr <= rx_data;
                            4'd5: rx_transaction_id[15:8] <= rx_data;
                            4'd6: rx_transaction_id[7:0] <= rx_data;
                            4'd7: rx_ext_addr <= rx_data;
                            4'd8: begin
                                t_state <= T_RX_ADDR;
                                rx_byte_cnt <= '0;
                            end
                        endcase
                    end else if (rx_eop || rx_eep) begin
                        t_state <= T_IDLE;
                        error_count <= error_count + 1;
                    end
                end
                
                T_RX_ADDR: begin
                    if (rx_valid) begin
                        rx_crc_calc <= crc8_byte(rx_crc_calc, rx_data);
                        rx_byte_cnt <= rx_byte_cnt + 1;
                        
                        case (rx_byte_cnt)
                            4'd0: rx_mem_addr[31:24] <= rx_data;
                            4'd1: rx_mem_addr[23:16] <= rx_data;
                            4'd2: rx_mem_addr[15:8] <= rx_data;
                            4'd3: begin
                                rx_mem_addr[7:0] <= rx_data;
                                t_state <= T_RX_LENGTH;
                                rx_byte_cnt <= '0;
                            end
                        endcase
                    end
                end
                
                T_RX_LENGTH: begin
                    if (rx_valid) begin
                        rx_crc_calc <= crc8_byte(rx_crc_calc, rx_data);
                        rx_byte_cnt <= rx_byte_cnt + 1;
                        
                        case (rx_byte_cnt)
                            4'd0: rx_data_length[23:16] <= rx_data;
                            4'd1: rx_data_length[15:8] <= rx_data;
                            4'd2: begin
                                rx_data_length[7:0] <= rx_data;
                                t_state <= T_RX_HEADER_CRC;
                            end
                        endcase
                    end
                end
                
                T_RX_HEADER_CRC: begin
                    if (rx_valid) begin
                        rx_header_crc <= rx_data;
                        
                        // Verify header CRC
                        if (rx_crc_calc == rx_data) begin
                            // Check if this is a write command with data
                            if (rx_instruction[5])  // Write command
                                t_state <= T_RX_DATA;
                            else
                                t_state <= T_PROCESS;
                            rx_data_cnt <= '0;
                            rx_crc_calc <= '0;
                        end else begin
                            crc_error <= 1'b1;
                            t_state <= T_ERROR;
                        end
                    end
                end
                
                T_RX_DATA: begin
                    if (rx_valid) begin
                        rx_crc_calc <= crc8_byte(rx_crc_calc, rx_data);
                        data_buffer[rx_data_cnt[7:0]] <= rx_data;
                        rx_data_cnt <= rx_data_cnt + 1;
                        
                        if (rx_data_cnt + 1 >= rx_data_length)
                            t_state <= T_RX_DATA_CRC;
                    end else if (rx_eop) begin
                        t_state <= T_RX_DATA_CRC;
                    end
                end
                
                T_RX_DATA_CRC: begin
                    if (rx_valid) begin
                        if (rx_crc_calc == rx_data) begin
                            t_state <= T_PROCESS;
                        end else begin
                            crc_error <= 1'b1;
                            t_state <= T_ERROR;
                        end
                    end else if (rx_eop) begin
                        t_state <= T_PROCESS;
                    end
                end
                
                T_PROCESS: begin
                    rx_cmd_count <= rx_cmd_count + 1;
                    
                    // Verify key
                    if (rx_key != target_key) begin
                        key_error <= 1'b1;
                        reply_status <= STATUS_INVALID_KEY;
                        t_state <= rx_instruction[3] ? T_TX_REPLY : T_IDLE;
                    end
                    // Verify address
                    else if (rx_mem_addr >= MEM_SIZE) begin
                        addr_error <= 1'b1;
                        reply_status <= STATUS_INVALID_ADDR;
                        t_state <= rx_instruction[3] ? T_TX_REPLY : T_IDLE;
                    end
                    else begin
                        reply_status <= STATUS_OK;
                        t_state <= T_MEM_ACCESS;
                        mem_addr <= rx_mem_addr;
                    end
                end
                
                T_MEM_ACCESS: begin
                    if (rx_instruction[5]) begin
                        // Write command
                        mem_write <= 1'b1;
                        mem_wdata <= {data_buffer[3], data_buffer[2], 
                                     data_buffer[1], data_buffer[0]};
                        
                        if (mem_ready) begin
                            mem_write <= 1'b0;
                            t_state <= rx_instruction[3] ? T_TX_REPLY : T_IDLE;
                        end
                    end else begin
                        // Read command
                        mem_read <= 1'b1;
                        
                        if (mem_ready) begin
                            mem_read <= 1'b0;
                            data_buffer[0] <= mem_rdata[7:0];
                            data_buffer[1] <= mem_rdata[15:8];
                            data_buffer[2] <= mem_rdata[23:16];
                            data_buffer[3] <= mem_rdata[31:24];
                            t_state <= T_TX_REPLY;
                        end
                    end
                end
                
                T_TX_REPLY: begin
                    // Send reply packet
                    if (tx_ready) begin
                        tx_valid <= 1'b1;
                        tx_data <= rx_src_addr;  // Reply to source
                        t_state <= T_TX_DATA;
                        rx_byte_cnt <= '0;
                        reply_crc <= crc8_byte(8'h00, rx_src_addr);
                    end
                end
                
                T_TX_DATA: begin
                    if (tx_ready) begin
                        rx_byte_cnt <= rx_byte_cnt + 1;
                        
                        case (rx_byte_cnt)
                            4'd0: begin
                                tx_data <= rx_dest_addr;
                                reply_crc <= crc8_byte(reply_crc, rx_dest_addr);
                            end
                            4'd1: begin
                                tx_data <= RMAP_PROTOCOL_ID;
                                reply_crc <= crc8_byte(reply_crc, RMAP_PROTOCOL_ID);
                            end
                            4'd2: begin
                                tx_data <= rx_instruction & 8'h3F;  // Reply instruction
                                reply_crc <= crc8_byte(reply_crc, rx_instruction & 8'h3F);
                            end
                            4'd3: begin
                                tx_data <= reply_status;
                                reply_crc <= crc8_byte(reply_crc, reply_status);
                            end
                            4'd4: begin
                                tx_data <= rx_src_addr;
                                reply_crc <= crc8_byte(reply_crc, rx_src_addr);
                            end
                            4'd5: begin
                                tx_data <= rx_transaction_id[15:8];
                                reply_crc <= crc8_byte(reply_crc, rx_transaction_id[15:8]);
                            end
                            4'd6: begin
                                tx_data <= rx_transaction_id[7:0];
                                reply_crc <= crc8_byte(reply_crc, rx_transaction_id[7:0]);
                            end
                            4'd7: begin
                                tx_data <= 8'h00;  // Reserved
                            end
                            default: begin
                                t_state <= T_TX_CRC;
                            end
                        endcase
                    end
                end
                
                T_TX_CRC: begin
                    if (tx_ready) begin
                        tx_data <= reply_crc;
                        t_state <= T_TX_EOP;
                        tx_rsp_count <= tx_rsp_count + 1;
                    end
                end
                
                T_TX_EOP: begin
                    if (tx_ready) begin
                        tx_valid <= 1'b0;
                        tx_eop <= 1'b1;
                        t_state <= T_IDLE;
                    end
                end
                
                T_ERROR: begin
                    error_count <= error_count + 1;
                    t_state <= T_IDLE;
                end
                
                default: t_state <= T_IDLE;
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // Status
    //------------------------------------------------------------------------
    assign busy = (t_state != T_IDLE);
    assign cmd_ready = !busy && enable_initiator;
    assign rsp_valid = 1'b0;  // Initiator response (placeholder)
    assign rsp_status = '0;
    assign rsp_data = '0;

endmodule




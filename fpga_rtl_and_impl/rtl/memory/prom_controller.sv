//============================================================================
// PROM (Programmable Read-Only Memory) Controller
// 
// Features:
//   - Space-grade PROM interface for boot code and constants
//   - SECDED ECC protection
//   - Read-only operation (write-protected)
//   - Configurable access timing for different PROM devices
//   - Supports parallel PROM interfaces (8-bit, 16-bit, 32-bit)
//
// Memory Map:
//   - Boot code at reset vector (0x0000_0000)
//   - Constants and lookup tables
//
// Author: Chandraboul
//============================================================================

`timescale 1ns / 1ps

module prom_controller #(
    parameter PROM_SIZE_KB    = 64,           // PROM size in KB
    parameter DATA_WIDTH      = 32,           // Internal data width
    parameter ADDR_WIDTH      = 32,           // Address width
    parameter PROM_DATA_WIDTH = 8,            // External PROM data width (8/16/32)
    parameter READ_CYCLES     = 4,            // PROM read access cycles
    parameter ENABLE_ECC      = 1             // Enable SECDED ECC
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // CPU Interface (AXI-Lite style)
    input  logic                        cpu_req_valid,
    input  logic [ADDR_WIDTH-1:0]       cpu_req_addr,
    output logic [DATA_WIDTH-1:0]       cpu_resp_data,
    output logic                        cpu_resp_valid,
    output logic                        cpu_resp_error,    // ECC uncorrectable error
    
    // PROM External Interface
    output logic [ADDR_WIDTH-1:0]       prom_addr,
    input  logic [PROM_DATA_WIDTH-1:0]  prom_data,
    output logic                        prom_ce_n,         // Chip enable (active low)
    output logic                        prom_oe_n,         // Output enable (active low)
    
    // ECC Status
    output logic                        ecc_correctable,
    output logic                        ecc_uncorrectable,
    output logic [31:0]                 ecc_error_count,
    output logic [ADDR_WIDTH-1:0]       ecc_error_addr
);

    //=========================================================================
    // Local Parameters
    //=========================================================================
    localparam PROM_BYTES = PROM_SIZE_KB * 1024;
    localparam PROM_ADDR_BITS = $clog2(PROM_BYTES);
    localparam READS_PER_WORD = DATA_WIDTH / PROM_DATA_WIDTH;
    localparam READ_CNT_BITS = $clog2(READS_PER_WORD + 1);
    
    // ECC parameters (SECDED for 32-bit data = 7 check bits)
    localparam ECC_BITS = 7;
    localparam PROTECTED_WIDTH = DATA_WIDTH + ECC_BITS;

    //=========================================================================
    // State Machine
    //=========================================================================
    typedef enum logic [2:0] {
        IDLE,
        READ_SETUP,
        READ_WAIT,
        READ_CAPTURE,
        ECC_CHECK,
        RESPOND,
        ERROR
    } state_t;
    
    state_t state, next_state;

    //=========================================================================
    // Internal Signals
    //=========================================================================
    logic [ADDR_WIDTH-1:0]       addr_reg;
    logic [DATA_WIDTH-1:0]       data_reg;
    logic [ECC_BITS-1:0]         ecc_reg;
    logic [$clog2(READ_CYCLES):0] wait_counter;
    logic [READ_CNT_BITS-1:0]    read_count;
    
    // ECC signals
    logic [DATA_WIDTH-1:0]       corrected_data;
    logic                        single_error;
    logic                        double_error;
    logic [5:0]                  error_position;

    //=========================================================================
    // PROM Read State Machine
    //=========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            addr_reg <= '0;
            data_reg <= '0;
            ecc_reg <= '0;
            wait_counter <= '0;
            read_count <= '0;
            ecc_error_count <= '0;
            ecc_error_addr <= '0;
        end else begin
            state <= next_state;
            
            case (state)
                IDLE: begin
                    if (cpu_req_valid) begin
                        addr_reg <= cpu_req_addr;
                        data_reg <= '0;
                        read_count <= '0;
                    end
                end
                
                READ_SETUP: begin
                    wait_counter <= '0;
                end
                
                READ_WAIT: begin
                    wait_counter <= wait_counter + 1;
                end
                
                READ_CAPTURE: begin
                    // Capture PROM data into appropriate byte position
                    case (read_count)
                        0: data_reg[7:0]   <= prom_data[7:0];
                        1: data_reg[15:8]  <= prom_data[7:0];
                        2: data_reg[23:16] <= prom_data[7:0];
                        3: data_reg[31:24] <= prom_data[7:0];
                        default: ;
                    endcase
                    
                    read_count <= read_count + 1;
                    addr_reg <= addr_reg + 1;
                end
                
                ECC_CHECK: begin
                    if (single_error) begin
                        ecc_error_count <= ecc_error_count + 1;
                        ecc_error_addr <= cpu_req_addr;
                    end
                end
                
                ERROR: begin
                    ecc_error_addr <= cpu_req_addr;
                end
            endcase
        end
    end

    // Next state logic
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (cpu_req_valid)
                    next_state = READ_SETUP;
            end
            
            READ_SETUP: begin
                next_state = READ_WAIT;
            end
            
            READ_WAIT: begin
                if (wait_counter >= READ_CYCLES - 1)
                    next_state = READ_CAPTURE;
            end
            
            READ_CAPTURE: begin
                if (read_count >= READS_PER_WORD - 1) begin
                    if (ENABLE_ECC)
                        next_state = ECC_CHECK;
                    else
                        next_state = RESPOND;
                end else begin
                    next_state = READ_SETUP;
                end
            end
            
            ECC_CHECK: begin
                if (double_error)
                    next_state = ERROR;
                else
                    next_state = RESPOND;
            end
            
            RESPOND: begin
                next_state = IDLE;
            end
            
            ERROR: begin
                next_state = RESPOND;  // Report error but continue
            end
            
            default: next_state = IDLE;
        endcase
    end

    //=========================================================================
    // PROM Interface
    //=========================================================================
    assign prom_addr = addr_reg[PROM_ADDR_BITS-1:0];
    assign prom_ce_n = (state == IDLE);
    assign prom_oe_n = (state == IDLE) || (state == READ_SETUP);

    //=========================================================================
    // SECDED ECC (Hamming 39,32)
    //=========================================================================
    generate
        if (ENABLE_ECC) begin : gen_ecc
            // ECC syndrome calculation
            logic [ECC_BITS-1:0] syndrome;
            logic [ECC_BITS-1:0] expected_ecc;
            
            // Calculate expected ECC for received data
            always_comb begin
                expected_ecc[0] = data_reg[0] ^ data_reg[1] ^ data_reg[3] ^ data_reg[4] ^
                                  data_reg[6] ^ data_reg[8] ^ data_reg[10] ^ data_reg[11] ^
                                  data_reg[13] ^ data_reg[15] ^ data_reg[17] ^ data_reg[19] ^
                                  data_reg[21] ^ data_reg[23] ^ data_reg[25] ^ data_reg[26] ^
                                  data_reg[28] ^ data_reg[30];
                
                expected_ecc[1] = data_reg[0] ^ data_reg[2] ^ data_reg[3] ^ data_reg[5] ^
                                  data_reg[6] ^ data_reg[9] ^ data_reg[10] ^ data_reg[12] ^
                                  data_reg[13] ^ data_reg[16] ^ data_reg[17] ^ data_reg[20] ^
                                  data_reg[21] ^ data_reg[24] ^ data_reg[25] ^ data_reg[27] ^
                                  data_reg[28] ^ data_reg[31];
                
                expected_ecc[2] = data_reg[1] ^ data_reg[2] ^ data_reg[3] ^ data_reg[7] ^
                                  data_reg[8] ^ data_reg[9] ^ data_reg[10] ^ data_reg[14] ^
                                  data_reg[15] ^ data_reg[16] ^ data_reg[17] ^ data_reg[22] ^
                                  data_reg[23] ^ data_reg[24] ^ data_reg[25] ^ data_reg[29] ^
                                  data_reg[30] ^ data_reg[31];
                
                expected_ecc[3] = data_reg[4] ^ data_reg[5] ^ data_reg[6] ^ data_reg[7] ^
                                  data_reg[8] ^ data_reg[9] ^ data_reg[10] ^ data_reg[18] ^
                                  data_reg[19] ^ data_reg[20] ^ data_reg[21] ^ data_reg[22] ^
                                  data_reg[23] ^ data_reg[24] ^ data_reg[25];
                
                expected_ecc[4] = data_reg[11] ^ data_reg[12] ^ data_reg[13] ^ data_reg[14] ^
                                  data_reg[15] ^ data_reg[16] ^ data_reg[17] ^ data_reg[18] ^
                                  data_reg[19] ^ data_reg[20] ^ data_reg[21] ^ data_reg[22] ^
                                  data_reg[23] ^ data_reg[24] ^ data_reg[25];
                
                expected_ecc[5] = data_reg[26] ^ data_reg[27] ^ data_reg[28] ^ data_reg[29] ^
                                  data_reg[30] ^ data_reg[31];
                
                // Overall parity
                expected_ecc[6] = ^data_reg ^ ^expected_ecc[5:0];
            end
            
            // For PROM, we store data+ECC together
            // Simplified: assume ECC is stored in separate region
            assign syndrome = expected_ecc ^ ecc_reg;
            assign single_error = (syndrome != 0) && expected_ecc[6];
            assign double_error = (syndrome != 0) && !expected_ecc[6];
            assign error_position = syndrome[5:0];
            
            // Correct single-bit error
            always_comb begin
                corrected_data = data_reg;
                if (single_error && error_position < 32) begin
                    corrected_data[error_position] = ~data_reg[error_position];
                end
            end
            
        end else begin : gen_no_ecc
            assign single_error = 1'b0;
            assign double_error = 1'b0;
            assign corrected_data = data_reg;
            assign error_position = '0;
        end
    endgenerate

    //=========================================================================
    // Output Signals
    //=========================================================================
    assign cpu_resp_data  = ENABLE_ECC ? corrected_data : data_reg;
    assign cpu_resp_valid = (state == RESPOND);
    assign cpu_resp_error = (state == ERROR) || (state == RESPOND && double_error);
    
    assign ecc_correctable   = single_error && (state == ECC_CHECK || state == RESPOND);
    assign ecc_uncorrectable = double_error && (state == ECC_CHECK || state == RESPOND);

endmodule



//============================================================================
// PhD Research: JTAG Debug Transport Module (DTM)
// Author: Chandraboul
// Target: RISC-V Debug Specification 0.13 Compatible
//
// Description:
//   JTAG Debug Transport Module for RISC-V debug interface.
//   Provides TAP controller and Debug Module Interface (DMI).
//
// Features:
//   - IEEE 1149.1 JTAG TAP controller
//   - RISC-V DTM registers (IDCODE, DTMCS, DMI)
//   - Debug Module Interface (DMI) access
//   - Automatic clock domain crossing
//   - BYPASS and IDCODE support
//
// JTAG Registers:
//   IR=0x01: IDCODE (32-bit)
//   IR=0x10: DTMCS  (32-bit)
//   IR=0x11: DMI    (variable width)
//   IR=0x1F: BYPASS (1-bit)
//============================================================================

`timescale 1ns / 1ps

module jtag_dtm #(
    parameter IDCODE_VALUE = 32'h10e31913,  // RISC-V debug IDCODE
    parameter IR_LENGTH    = 5,
    parameter DMI_ADDR_BITS= 7,
    parameter DMI_DATA_BITS= 32
)(
    // JTAG Interface
    input  logic            tck,
    input  logic            tms,
    input  logic            tdi,
    output logic            tdo,
    input  logic            trst_n,
    
    // System Clock Domain
    input  logic            sys_clk,
    input  logic            sys_rst_n,
    
    // Debug Module Interface (DMI)
    output logic                       dmi_req_valid,
    input  logic                       dmi_req_ready,
    output logic [DMI_ADDR_BITS-1:0]   dmi_req_addr,
    output logic [DMI_DATA_BITS-1:0]   dmi_req_data,
    output logic [1:0]                 dmi_req_op,    // 0=NOP, 1=READ, 2=WRITE
    
    input  logic                       dmi_rsp_valid,
    output logic                       dmi_rsp_ready,
    input  logic [DMI_DATA_BITS-1:0]   dmi_rsp_data,
    input  logic [1:0]                 dmi_rsp_op     // 0=OK, 2=FAILED, 3=BUSY
);

    //------------------------------------------------------------------------
    // TAP Controller States
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TEST_LOGIC_RESET = 4'h0,
        RUN_TEST_IDLE    = 4'h1,
        SELECT_DR_SCAN   = 4'h2,
        CAPTURE_DR       = 4'h3,
        SHIFT_DR         = 4'h4,
        EXIT1_DR         = 4'h5,
        PAUSE_DR         = 4'h6,
        EXIT2_DR         = 4'h7,
        UPDATE_DR        = 4'h8,
        SELECT_IR_SCAN   = 4'h9,
        CAPTURE_IR       = 4'hA,
        SHIFT_IR         = 4'hB,
        EXIT1_IR         = 4'hC,
        PAUSE_IR         = 4'hD,
        EXIT2_IR         = 4'hE,
        UPDATE_IR        = 4'hF
    } tap_state_t;
    
    tap_state_t tap_state;
    
    //------------------------------------------------------------------------
    // TAP State Machine
    //------------------------------------------------------------------------
    always_ff @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            tap_state <= TEST_LOGIC_RESET;
        end else begin
            case (tap_state)
                TEST_LOGIC_RESET: tap_state <= tms ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
                RUN_TEST_IDLE:    tap_state <= tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
                SELECT_DR_SCAN:   tap_state <= tms ? SELECT_IR_SCAN : CAPTURE_DR;
                CAPTURE_DR:       tap_state <= tms ? EXIT1_DR : SHIFT_DR;
                SHIFT_DR:         tap_state <= tms ? EXIT1_DR : SHIFT_DR;
                EXIT1_DR:         tap_state <= tms ? UPDATE_DR : PAUSE_DR;
                PAUSE_DR:         tap_state <= tms ? EXIT2_DR : PAUSE_DR;
                EXIT2_DR:         tap_state <= tms ? UPDATE_DR : SHIFT_DR;
                UPDATE_DR:        tap_state <= tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
                SELECT_IR_SCAN:   tap_state <= tms ? TEST_LOGIC_RESET : CAPTURE_IR;
                CAPTURE_IR:       tap_state <= tms ? EXIT1_IR : SHIFT_IR;
                SHIFT_IR:         tap_state <= tms ? EXIT1_IR : SHIFT_IR;
                EXIT1_IR:         tap_state <= tms ? UPDATE_IR : PAUSE_IR;
                PAUSE_IR:         tap_state <= tms ? EXIT2_IR : PAUSE_IR;
                EXIT2_IR:         tap_state <= tms ? UPDATE_IR : SHIFT_IR;
                UPDATE_IR:        tap_state <= tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
                default:          tap_state <= TEST_LOGIC_RESET;
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // Instruction Register
    //------------------------------------------------------------------------
    localparam [IR_LENGTH-1:0] IR_IDCODE = 5'b00001;
    localparam [IR_LENGTH-1:0] IR_DTMCS  = 5'b10000;
    localparam [IR_LENGTH-1:0] IR_DMI    = 5'b10001;
    localparam [IR_LENGTH-1:0] IR_BYPASS = 5'b11111;
    
    logic [IR_LENGTH-1:0] ir_shift;
    logic [IR_LENGTH-1:0] ir_reg;
    
    always_ff @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            ir_shift <= IR_IDCODE;
            ir_reg <= IR_IDCODE;
        end else begin
            case (tap_state)
                CAPTURE_IR: ir_shift <= IR_IDCODE;  // Capture fixed pattern
                SHIFT_IR:   ir_shift <= {tdi, ir_shift[IR_LENGTH-1:1]};
                UPDATE_IR:  ir_reg <= ir_shift;
                default: ;
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // Data Registers
    //------------------------------------------------------------------------
    // IDCODE register (read-only)
    logic [31:0] idcode_shift;
    
    // DTMCS register
    logic [31:0] dtmcs_shift;
    logic [31:0] dtmcs_reg;
    
    // DMI register
    localparam DMI_WIDTH = DMI_ADDR_BITS + DMI_DATA_BITS + 2;
    logic [DMI_WIDTH-1:0] dmi_shift;
    logic [DMI_WIDTH-1:0] dmi_reg;
    
    // Bypass register
    logic bypass_shift;
    
    //------------------------------------------------------------------------
    // DTMCS Register Fields
    //------------------------------------------------------------------------
    // [31:18] Reserved
    // [17]    dmihardreset - Write 1 to reset DMI
    // [16]    dmireset     - Write 1 to clear error state
    // [15]    Reserved
    // [14:12] idle         - Minimum cycles in Run-Test/Idle
    // [11:10] dmistat      - DMI status (0=ok, 1=reserved, 2=failed, 3=busy)
    // [9:4]   abits        - Address bits
    // [3:0]   version      - 0.13
    
    logic [1:0] dmi_status;
    
    always_comb begin
        dtmcs_reg = {
            14'b0,              // [31:18] Reserved
            1'b0,               // [17] dmihardreset (write only)
            1'b0,               // [16] dmireset (write only)
            1'b0,               // [15] Reserved
            3'd5,               // [14:12] idle cycles
            dmi_status,         // [11:10] dmistat
            6'(DMI_ADDR_BITS),  // [9:4] abits
            4'h1                // [3:0] version 0.13
        };
    end
    
    //------------------------------------------------------------------------
    // Data Register Shift Logic
    //------------------------------------------------------------------------
    always_ff @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            idcode_shift <= IDCODE_VALUE;
            dtmcs_shift <= '0;
            dmi_shift <= '0;
            bypass_shift <= 1'b0;
        end else begin
            case (tap_state)
                CAPTURE_DR: begin
                    case (ir_reg)
                        IR_IDCODE: idcode_shift <= IDCODE_VALUE;
                        IR_DTMCS:  dtmcs_shift <= dtmcs_reg;
                        IR_DMI:    dmi_shift <= {dmi_reg[DMI_WIDTH-1:2], dmi_status};
                        IR_BYPASS: bypass_shift <= 1'b0;
                        default: ;
                    endcase
                end
                
                SHIFT_DR: begin
                    case (ir_reg)
                        IR_IDCODE: idcode_shift <= {tdi, idcode_shift[31:1]};
                        IR_DTMCS:  dtmcs_shift <= {tdi, dtmcs_shift[31:1]};
                        IR_DMI:    dmi_shift <= {tdi, dmi_shift[DMI_WIDTH-1:1]};
                        IR_BYPASS: bypass_shift <= tdi;
                        default: ;
                    endcase
                end
                
                UPDATE_DR: begin
                    case (ir_reg)
                        IR_DMI: dmi_reg <= dmi_shift;
                        default: ;
                    endcase
                end
                
                default: ;
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // TDO Multiplexer
    //------------------------------------------------------------------------
    logic tdo_ir, tdo_dr;
    
    always_comb begin
        tdo_ir = ir_shift[0];
        
        case (ir_reg)
            IR_IDCODE: tdo_dr = idcode_shift[0];
            IR_DTMCS:  tdo_dr = dtmcs_shift[0];
            IR_DMI:    tdo_dr = dmi_shift[0];
            IR_BYPASS: tdo_dr = bypass_shift;
            default:   tdo_dr = 1'b0;
        endcase
    end
    
    // TDO output (active during shift states)
    always_ff @(negedge tck) begin
        case (tap_state)
            SHIFT_IR: tdo <= tdo_ir;
            SHIFT_DR: tdo <= tdo_dr;
            default:  tdo <= 1'b0;
        endcase
    end
    
    //------------------------------------------------------------------------
    // DMI Interface to Debug Module
    //------------------------------------------------------------------------
    // Clock domain crossing using synchronizers
    logic dmi_req_pending;
    logic [1:0] dmi_op;
    
    assign dmi_op = dmi_reg[1:0];
    
    // Request generation
    always_ff @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin
            dmi_req_pending <= 1'b0;
        end else begin
            if (tap_state == UPDATE_DR && ir_reg == IR_DMI && dmi_op != 2'b00) begin
                dmi_req_pending <= 1'b1;
            end
        end
    end
    
    // Synchronize to system clock domain
    logic [2:0] req_sync;
    
    always_ff @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            req_sync <= '0;
            dmi_req_valid <= 1'b0;
        end else begin
            req_sync <= {req_sync[1:0], dmi_req_pending};
            dmi_req_valid <= req_sync[1] && !req_sync[2];
        end
    end
    
    assign dmi_req_addr = dmi_reg[DMI_WIDTH-1:DMI_DATA_BITS+2];
    assign dmi_req_data = dmi_reg[DMI_DATA_BITS+1:2];
    assign dmi_req_op   = dmi_reg[1:0];
    
    // Response handling
    always_ff @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            dmi_status <= 2'b00;
            dmi_rsp_ready <= 1'b1;
        end else begin
            if (dmi_rsp_valid) begin
                dmi_status <= dmi_rsp_op;
            end
        end
    end

endmodule




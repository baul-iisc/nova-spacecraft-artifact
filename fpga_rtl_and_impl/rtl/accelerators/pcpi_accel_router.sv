//============================================================================
// PCPI Accelerator Router
//
// Routes custom RISC-V instructions from the core's PCPI interface to
// CORDIC and Systolic array accelerators.
//
// Custom Instruction Encoding (Custom-0 opcode = 7'b0001011):
//
//   CORDIC Operations (funct7[6:4] = 3'b000):
//     Encoding: [funct7=000_CCCC][rs2][rs1][funct3][rd][0001011]
//       CCCC = cordic_op[3:0]
//       funct3 selects which CORDIC input:
//         000: CORDIC start — x_in=rs1, y_in=rs2, z_in from CSR or prev
//         001: CORDIC set Z — z_in=rs1
//         010: CORDIC read result_x → rd
//         011: CORDIC read result_y → rd
//         100: CORDIC read result_z → rd
//
//   Systolic Operations (funct7[6:4] = 3'b001):
//     Encoding: [funct7=001_IIII][rs2][rs1][funct3][rd][0001011]
//       IIII = element index [3:0] (0-8 for 3x3 matrix)
//       funct3 selects operation:
//         000: Set A[index] = rs1
//         001: Set B[index] = rs1
//         010: Start multiply
//         011: Read C[index] → rd
//         100: Read status → rd
//
// Author: Chandraboul, IISc
//============================================================================

`timescale 1ns / 1ps

module pcpi_accel_router #(
    parameter XLEN = 64
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // PCPI interface from core
    input  logic                    pcpi_valid,
    input  logic [31:0]             pcpi_insn,
    input  logic [XLEN-1:0]         pcpi_rs1,
    input  logic [XLEN-1:0]         pcpi_rs2,
    output logic [XLEN-1:0]         pcpi_rd,
    output logic                    pcpi_ready,
    output logic                    pcpi_wait,

    // CORDIC interface
    output logic                    cordic_valid,
    output logic [3:0]              cordic_op,
    output logic [63:0]             cordic_x,
    output logic [63:0]             cordic_y,
    output logic [63:0]             cordic_z,
    input  logic [63:0]             cordic_result_x,
    input  logic [63:0]             cordic_result_y,
    input  logic [63:0]             cordic_result_z,
    input  logic                    cordic_ready,

    // Systolic array interface
    output logic                    systolic_valid,
    output logic [63:0]             systolic_a [0:8],
    output logic [63:0]             systolic_b [0:8],
    input  logic [63:0]             systolic_c [0:8],
    input  logic                    systolic_ready
);

    //=========================================================================
    // Instruction field extraction
    //=========================================================================
    wire [6:0] funct7 = pcpi_insn[31:25];
    wire [2:0] funct3 = pcpi_insn[14:12];
    wire [2:0] accel_sel = funct7[6:4];       // 000=CORDIC, 001=Systolic
    wire [3:0] accel_sub = funct7[3:0];       // Sub-function / index

    //=========================================================================
    // State machine
    //=========================================================================
    typedef enum logic [2:0] {
        IDLE,
        CORDIC_WAIT,
        CORDIC_DONE,
        SYSTOLIC_WAIT,
        SYSTOLIC_DONE,
        RESULT_READY
    } state_t;

    state_t state, next_state;

    //=========================================================================
    // CORDIC Z register (persists across instructions)
    //=========================================================================
    logic [63:0] cordic_z_reg;

    //=========================================================================
    // Systolic input registers (loaded element-by-element)
    //=========================================================================
    logic [63:0] sa_a_reg [0:8];
    logic [63:0] sa_b_reg [0:8];
    logic        sa_started;

    //=========================================================================
    // Result register
    //=========================================================================
    logic [XLEN-1:0] result_reg;

    //=========================================================================
    // FSM
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            cordic_z_reg <= 64'd0;
            sa_started <= 1'b0;
            result_reg <= '0;
            for (int i = 0; i < 9; i++) begin
                sa_a_reg[i] <= 64'd0;
                sa_b_reg[i] <= 64'd0;
            end
        end else begin
            case (state)
                IDLE: begin
                    if (pcpi_valid) begin
                        case (accel_sel)
                            3'b000: begin // CORDIC
                                case (funct3)
                                    3'b000: begin // CORDIC start
                                        cordic_z_reg <= cordic_z_reg; // Use previously set Z
                                        state <= CORDIC_WAIT;
                                    end
                                    3'b001: begin // Set Z
                                        cordic_z_reg <= pcpi_rs1;
                                        result_reg <= '0;
                                        state <= RESULT_READY;
                                    end
                                    3'b010: begin // Read result_x
                                        result_reg <= cordic_result_x;
                                        state <= RESULT_READY;
                                    end
                                    3'b011: begin // Read result_y
                                        result_reg <= cordic_result_y;
                                        state <= RESULT_READY;
                                    end
                                    3'b100: begin // Read result_z
                                        result_reg <= cordic_result_z;
                                        state <= RESULT_READY;
                                    end
                                    default: begin
                                        result_reg <= '0;
                                        state <= RESULT_READY;
                                    end
                                endcase
                            end
                            3'b001: begin // Systolic
                                case (funct3)
                                    3'b000: begin // Set A[index]
                                        if (accel_sub < 4'd9)
                                            sa_a_reg[accel_sub] <= pcpi_rs1;
                                        result_reg <= '0;
                                        state <= RESULT_READY;
                                    end
                                    3'b001: begin // Set B[index]
                                        if (accel_sub < 4'd9)
                                            sa_b_reg[accel_sub] <= pcpi_rs1;
                                        result_reg <= '0;
                                        state <= RESULT_READY;
                                    end
                                    3'b010: begin // Start multiply
                                        sa_started <= 1'b1;
                                        state <= SYSTOLIC_WAIT;
                                    end
                                    3'b011: begin // Read C[index]
                                        if (accel_sub < 4'd9)
                                            result_reg <= systolic_c[accel_sub];
                                        else
                                            result_reg <= '0;
                                        state <= RESULT_READY;
                                    end
                                    3'b100: begin // Read status
                                        result_reg <= {63'd0, systolic_ready};
                                        state <= RESULT_READY;
                                    end
                                    default: begin
                                        result_reg <= '0;
                                        state <= RESULT_READY;
                                    end
                                endcase
                            end
                            default: begin
                                result_reg <= '0;
                                state <= RESULT_READY;
                            end
                        endcase
                    end
                end

                CORDIC_WAIT: begin
                    if (cordic_ready) begin
                        // CORDIC done — default result is x_out
                        result_reg <= cordic_result_x;
                        state <= RESULT_READY;
                    end
                end

                SYSTOLIC_WAIT: begin
                    sa_started <= 1'b0; // Pulse valid for one cycle
                    if (systolic_ready) begin
                        result_reg <= systolic_c[0]; // Default: return C[0]
                        state <= RESULT_READY;
                    end
                end

                RESULT_READY: begin
                    // Hold result for one cycle, then return to IDLE
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

    //=========================================================================
    // CORDIC output signals
    //=========================================================================
    assign cordic_valid = (state == IDLE) && pcpi_valid && (accel_sel == 3'b000) && (funct3 == 3'b000);
    assign cordic_op    = accel_sub;
    assign cordic_x     = pcpi_rs1;
    assign cordic_y     = pcpi_rs2;
    assign cordic_z     = cordic_z_reg;

    //=========================================================================
    // Systolic output signals
    //=========================================================================
    assign systolic_valid = sa_started;

    // Drive systolic inputs from registers
    generate
        for (genvar i = 0; i < 9; i++) begin : gen_systolic_io
            assign systolic_a[i] = sa_a_reg[i];
            assign systolic_b[i] = sa_b_reg[i];
        end
    endgenerate

    //=========================================================================
    // PCPI response signals
    //=========================================================================
    assign pcpi_rd    = result_reg;
    assign pcpi_ready = (state == RESULT_READY);
    assign pcpi_wait  = (state == CORDIC_WAIT) || (state == SYSTOLIC_WAIT);

endmodule

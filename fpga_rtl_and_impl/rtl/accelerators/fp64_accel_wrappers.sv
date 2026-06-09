//============================================================================
// PhD Research: FP64 Accelerator PCPI Wrappers
// Author: Chandraboul, IISc
// Target: Kintex UltraScale KU060
//
// Description:
//   Simplified PCPI interface wrappers for FP64 accelerators.
//   These modules interface between the RISC-V core's PCPI and
//   the underlying FP64 compute units.
//============================================================================

`timescale 1ns / 1ps

//============================================================================
// FP64 Matrix Accelerator Wrapper
//============================================================================
module fp64_matrix_accel #(
    parameter MATRIX_SIZE = 3
)(
    input  logic        clk,
    input  logic        rst_n,
    
    input  logic        valid,
    input  logic [31:0] insn,
    input  logic [31:0] rs1,
    input  logic [31:0] rs2,
    
    output logic        ready,
    output logic [31:0] rd
);

    // Decode funct7 for operation
    wire [6:0] funct7 = insn[31:25];
    
    // Operations
    localparam OP_MATMUL   = 7'd0;   // Matrix multiply C = A × B
    localparam OP_MATADD   = 7'd1;   // Matrix add C = A + B
    localparam OP_MATSUB   = 7'd2;   // Matrix subtract C = A - B
    localparam OP_MATSCALE = 7'd3;   // Scale C = A × scalar
    localparam OP_DOTPROD  = 7'd4;   // Dot product
    localparam OP_NORM     = 7'd5;   // Vector norm
    
    // State machine
    typedef enum logic [2:0] {IDLE, COMPUTE, DONE} state_t;
    state_t state;
    logic [4:0] cycle_count;
    
    // FP64 accumulator for results
    logic [63:0] accum;
    logic [63:0] operand_a, operand_b;
    
    // Combine 32-bit inputs to form 64-bit FP values
    // In real implementation, would use memory-mapped FP registers
    assign operand_a = {rs1, rs1};  // Placeholder - use FP reg file
    assign operand_b = {rs2, rs2};  // Placeholder
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            ready <= 1'b0;
            rd <= 32'h0;
            cycle_count <= 5'd0;
            accum <= 64'd0;
        end else begin
            ready <= 1'b0;
            
            case (state)
                IDLE: begin
                    if (valid) begin
                        state <= COMPUTE;
                        cycle_count <= 5'd12;  // Pipeline latency
                        // Start computation based on operation
                        case (funct7)
                            OP_DOTPROD: accum <= operand_a * operand_b;  // Simplified
                            default: accum <= operand_a + operand_b;
                        endcase
                    end
                end
                
                COMPUTE: begin
                    if (cycle_count > 0) begin
                        cycle_count <= cycle_count - 1;
                    end else begin
                        state <= DONE;
                    end
                end
                
                DONE: begin
                    ready <= 1'b1;
                    rd <= accum[31:0];  // Return lower 32 bits
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule


//============================================================================
// FP64 CORDIC Accelerator Wrapper
//============================================================================
module fp64_cordic_accel (
    input  logic        clk,
    input  logic        rst_n,
    
    input  logic        valid,
    input  logic [31:0] insn,
    input  logic [31:0] rs1,
    input  logic [31:0] rs2,
    
    output logic        ready,
    output logic [31:0] rd
);

    // Decode funct7 for function selection
    wire [6:0] funct7 = insn[31:25];
    
    // Trigonometric functions
    localparam FUNC_SIN    = 7'd0;
    localparam FUNC_COS    = 7'd1;
    localparam FUNC_TAN    = 7'd2;
    localparam FUNC_ASIN   = 7'd3;
    localparam FUNC_ACOS   = 7'd4;
    localparam FUNC_ATAN   = 7'd5;
    localparam FUNC_ATAN2  = 7'd6;
    localparam FUNC_SQRT   = 7'd7;
    localparam FUNC_MAG    = 7'd8;   // Magnitude sqrt(x²+y²)
    localparam FUNC_PHASE  = 7'd9;   // Phase atan2(y,x)
    
    // CORDIC unit signals
    logic        cordic_valid, cordic_done;
    logic [63:0] cordic_x_out, cordic_y_out, cordic_z_out;
    logic [63:0] cordic_x_in, cordic_y_in, cordic_z_in;
    logic [3:0]  cordic_op;
    
    // Map funct7 to CORDIC operation
    always_comb begin
        case (funct7)
            FUNC_SIN, FUNC_COS: begin
                cordic_op = 4'd0;  // COS_SIN mode
                cordic_x_in = 64'h0;
                cordic_y_in = 64'h0;
                cordic_z_in = {rs2, rs1};  // Angle
            end
            FUNC_ATAN2, FUNC_MAG, FUNC_PHASE: begin
                cordic_op = 4'd1;  // ATAN mode
                cordic_x_in = {32'h0, rs1};  // X
                cordic_y_in = {32'h0, rs2};  // Y
                cordic_z_in = 64'h0;
            end
            FUNC_SQRT: begin
                cordic_op = 4'd3;  // SQRT mode
                cordic_x_in = {rs2, rs1};
                cordic_y_in = 64'h0;
                cordic_z_in = 64'h0;
            end
            default: begin
                cordic_op = 4'd0;
                cordic_x_in = {rs2, rs1};
                cordic_y_in = 64'h0;
                cordic_z_in = 64'h0;
            end
        endcase
    end
    
    // CORDIC compute unit
    fp64_cordic_unit u_cordic (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (valid),
        .op        (cordic_op),
        .ready     (),
        .x_in      (cordic_x_in),
        .y_in      (cordic_y_in),
        .z_in      (cordic_z_in),
        .x_out     (cordic_x_out),
        .y_out     (cordic_y_out),
        .z_out     (cordic_z_out),
        .valid_out (cordic_done),
        .overflow  (),
        .underflow ()
    );
    
    // Output selection based on function
    logic [31:0] result_select;
    
    always_comb begin
        case (funct7)
            FUNC_SIN:   result_select = cordic_y_out[31:0];
            FUNC_COS:   result_select = cordic_x_out[31:0];
            FUNC_ATAN, FUNC_ATAN2, FUNC_PHASE:
                        result_select = cordic_z_out[31:0];
            FUNC_SQRT, FUNC_MAG:
                        result_select = cordic_x_out[31:0];
            default:    result_select = cordic_x_out[31:0];
        endcase
    end
    
    // Pipeline latency counter
    logic [5:0] pipeline_count;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            rd <= 32'h0;
            pipeline_count <= 6'd0;
        end else begin
            ready <= 1'b0;
            
            if (valid && pipeline_count == 0) begin
                pipeline_count <= 6'd20;  // CORDIC pipeline depth
            end else if (pipeline_count > 1) begin
                pipeline_count <= pipeline_count - 1;
            end else if (pipeline_count == 1) begin
                ready <= 1'b1;
                rd <= result_select;
                pipeline_count <= 6'd0;
            end
        end
    end

endmodule


//============================================================================
// FP64 MAC Accelerator Wrapper
//============================================================================
module fp64_mac_accel (
    input  logic        clk,
    input  logic        rst_n,
    
    input  logic        valid,
    input  logic [31:0] insn,
    input  logic [31:0] rs1,
    input  logic [31:0] rs2,
    
    output logic        ready,
    output logic [31:0] rd
);

    // Decode funct7 for operation
    wire [6:0] funct7 = insn[31:25];
    wire [2:0] rm = insn[14:12];  // Rounding mode
    
    // MAC operations
    localparam OP_FMADD  = 7'd0;   // a×b + c
    localparam OP_FMSUB  = 7'd1;   // a×b - c
    localparam OP_FNMADD = 7'd2;   // -(a×b) + c
    localparam OP_FNMSUB = 7'd3;   // -(a×b) - c
    localparam OP_FMUL   = 7'd4;   // a×b
    localparam OP_FADD   = 7'd5;   // a+b
    localparam OP_FSUB   = 7'd6;   // a-b
    localparam OP_FDIV   = 7'd7;   // a/b
    
    // FP64 operands
    logic [63:0] operand_a, operand_b, operand_c;
    logic [63:0] mac_result;
    logic        mac_valid_out;
    
    // Combine register inputs (placeholder - real impl uses FP reg file)
    assign operand_a = {rs1, rs1};
    assign operand_b = {rs2, rs2};
    assign operand_c = 64'h0;  // Third operand from additional register
    
    // Map to MAC operation
    logic [2:0] mac_op;
    always_comb begin
        case (funct7)
            OP_FMADD:  mac_op = 3'b000;
            OP_FMUL:   mac_op = 3'b001;
            OP_FADD:   mac_op = 3'b010;
            OP_FSUB:   mac_op = 3'b011;
            default:   mac_op = 3'b000;
        endcase
    end
    
    // FP64 MAC unit
    fp64_mac_unit u_mac (
        .clk           (clk),
        .rst_n         (rst_n),
        .valid_in      (valid),
        .op            (mac_op),
        .rm            (rm),
        .ready         (),
        .a             (operand_a),
        .b             (operand_b),
        .c             (operand_c),
        .result        (mac_result),
        .valid_out     (mac_valid_out),
        .flag_invalid  (),
        .flag_overflow (),
        .flag_underflow(),
        .flag_inexact  (),
        .flag_divbyzero()
    );
    
    // Pipeline latency counter
    logic [3:0] pipeline_count;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ready <= 1'b0;
            rd <= 32'h0;
            pipeline_count <= 4'd0;
        end else begin
            ready <= 1'b0;
            
            if (valid && pipeline_count == 0) begin
                pipeline_count <= 4'd5;  // MAC pipeline depth
            end else if (pipeline_count > 1) begin
                pipeline_count <= pipeline_count - 1;
            end else if (pipeline_count == 1) begin
                ready <= 1'b1;
                rd <= mac_result[31:0];
                pipeline_count <= 4'd0;
            end
        end
    end

endmodule












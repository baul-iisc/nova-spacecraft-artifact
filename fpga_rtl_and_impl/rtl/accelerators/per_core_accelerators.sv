//============================================================================
// Per-Core Accelerators
// 
// Each core gets its own dedicated:
//   - FP64 CORDIC trigonometric unit
//   - FP64 3x3 Systolic Array for matrix multiplication
//
// This ensures DLS pairs can compare accelerator outputs
// and eliminates contention for shared accelerators.
//
// Author: Chandraboul
//============================================================================

`timescale 1ns / 1ps

module per_core_accelerators #(
    parameter CORE_ID   = 0,
    parameter DATA_WIDTH = 64,   // FP64
    parameter MATRIX_DIM = 3     // 3x3
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    //=========================================================================
    // CORDIC Interface
    //=========================================================================
    input  logic                    cordic_valid,
    input  logic [2:0]              cordic_op,      // 0=sin/cos, 1=atan, 2=sinh/cosh, etc.
    input  logic [63:0]             cordic_angle,   // Input angle (radians, FP64)
    output logic [63:0]             cordic_sin,     // sin(angle)
    output logic [63:0]             cordic_cos,     // cos(angle)
    output logic [63:0]             cordic_result,  // Generic result (for atan, etc.)
    output logic                    cordic_ready,
    output logic                    cordic_busy,
    
    //=========================================================================
    // Systolic Array Interface
    //=========================================================================
    input  logic                    matrix_valid,
    input  logic [DATA_WIDTH-1:0]   matrix_a [0:MATRIX_DIM*MATRIX_DIM-1],  // 3x3 = 9 elements
    input  logic [DATA_WIDTH-1:0]   matrix_b [0:MATRIX_DIM*MATRIX_DIM-1],
    output logic [DATA_WIDTH-1:0]   matrix_c [0:MATRIX_DIM*MATRIX_DIM-1],  // Result C = A * B
    output logic                    matrix_ready,
    output logic                    matrix_busy,
    
    //=========================================================================
    // FMA (Fused Multiply-Add) Interface - for FPU support
    //=========================================================================
    input  logic                    fma_valid,
    input  logic [63:0]             fma_a,
    input  logic [63:0]             fma_b,
    input  logic [63:0]             fma_c,
    output logic [63:0]             fma_result,     // a*b + c
    output logic                    fma_ready,
    
    // Status
    output logic                    accel_error
);

    //=========================================================================
    // Internal Signals
    //=========================================================================
    logic cordic_start;
    logic matrix_start;
    logic fma_start;
    
    // Cordic internal
    logic [63:0] cordic_x, cordic_y, cordic_z;
    logic        cordic_done;
    
    // Matrix internal
    logic [DATA_WIDTH-1:0] systolic_out [0:MATRIX_DIM*MATRIX_DIM-1];
    logic        systolic_done;
    
    //=========================================================================
    // FP64 CORDIC Unit Instance
    //=========================================================================
    fp64_cordic_trig #(
        .ITERATIONS(54),
        .PIPELINE_STAGES(20)
    ) u_cordic (
        .clk(clk),
        .rst_n(rst_n),
        .start(cordic_valid),
        .op(cordic_op),
        .angle(cordic_angle),
        .sin_out(cordic_sin),
        .cos_out(cordic_cos),
        .result(cordic_result),
        .done(cordic_ready),
        .busy(cordic_busy)
    );

    //=========================================================================
    // FP64 3x3 Systolic Array Instance
    //=========================================================================
    systolic_array_3x3 #(
        .DATA_WIDTH(DATA_WIDTH),
        .MATRIX_DIM(MATRIX_DIM)
    ) u_systolic (
        .clk(clk),
        .rst_n(rst_n),
        .start(matrix_valid),
        .a_matrix(matrix_a),
        .b_matrix(matrix_b),
        .c_matrix(matrix_c),
        .done(matrix_ready),
        .busy(matrix_busy)
    );

    //=========================================================================
    // FP64 Fused Multiply-Add Unit
    //=========================================================================
    fp64_mac_unit #(
        .PIPELINE_STAGES(5),
        .ENABLE_SUBNORMAL(1)
    ) u_fma (
        .clk(clk),
        .rst_n(rst_n),
        .valid(fma_valid),
        .op_a(fma_a),
        .op_b(fma_b),
        .op_c(fma_c),
        .result(fma_result),
        .ready(fma_ready),
        .overflow(),
        .underflow(),
        .invalid()
    );

    // Error detection (overflow, NaN, etc.)
    assign accel_error = 1'b0;  // Simplified - in real impl check FPU flags

endmodule

//============================================================================
// Octa-Core Accelerator Array
// Instantiates 8 sets of per-core accelerators
//============================================================================
module octa_core_accelerators #(
    parameter NUM_CORES  = 8,
    parameter DATA_WIDTH = 64,
    parameter MATRIX_DIM = 3
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Per-core CORDIC interfaces
    input  logic [NUM_CORES-1:0]    cordic_valid,
    input  logic [NUM_CORES-1:0][2:0]  cordic_op,
    input  logic [NUM_CORES-1:0][63:0] cordic_angle,
    output logic [NUM_CORES-1:0][63:0] cordic_sin,
    output logic [NUM_CORES-1:0][63:0] cordic_cos,
    output logic [NUM_CORES-1:0]    cordic_ready,
    output logic [NUM_CORES-1:0]    cordic_busy,
    
    // Per-core Systolic Array interfaces
    input  logic [NUM_CORES-1:0]    matrix_valid,
    input  logic [NUM_CORES-1:0][DATA_WIDTH-1:0] matrix_a [0:MATRIX_DIM*MATRIX_DIM-1],
    input  logic [NUM_CORES-1:0][DATA_WIDTH-1:0] matrix_b [0:MATRIX_DIM*MATRIX_DIM-1],
    output logic [NUM_CORES-1:0][DATA_WIDTH-1:0] matrix_c [0:MATRIX_DIM*MATRIX_DIM-1],
    output logic [NUM_CORES-1:0]    matrix_ready,
    output logic [NUM_CORES-1:0]    matrix_busy,
    
    // Per-core FMA interfaces
    input  logic [NUM_CORES-1:0]    fma_valid,
    input  logic [NUM_CORES-1:0][63:0] fma_a,
    input  logic [NUM_CORES-1:0][63:0] fma_b,
    input  logic [NUM_CORES-1:0][63:0] fma_c,
    output logic [NUM_CORES-1:0][63:0] fma_result,
    output logic [NUM_CORES-1:0]    fma_ready,
    
    // Status
    output logic [NUM_CORES-1:0]    accel_error
);

    //=========================================================================
    // Generate per-core accelerators
    //=========================================================================
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i++) begin : gen_core_accel
            // Local signals for matrix arrays
            logic [DATA_WIDTH-1:0] local_matrix_a [0:MATRIX_DIM*MATRIX_DIM-1];
            logic [DATA_WIDTH-1:0] local_matrix_b [0:MATRIX_DIM*MATRIX_DIM-1];
            logic [DATA_WIDTH-1:0] local_matrix_c [0:MATRIX_DIM*MATRIX_DIM-1];
            
            // Map 2D packed to 1D unpacked
            always_comb begin
                for (int j = 0; j < MATRIX_DIM*MATRIX_DIM; j++) begin
                    local_matrix_a[j] = matrix_a[j][i];
                    local_matrix_b[j] = matrix_b[j][i];
                    matrix_c[j][i] = local_matrix_c[j];
                end
            end
            
            per_core_accelerators #(
                .CORE_ID(i),
                .DATA_WIDTH(DATA_WIDTH),
                .MATRIX_DIM(MATRIX_DIM)
            ) u_accel (
                .clk(clk),
                .rst_n(rst_n),
                
                // CORDIC
                .cordic_valid(cordic_valid[i]),
                .cordic_op(cordic_op[i]),
                .cordic_angle(cordic_angle[i]),
                .cordic_sin(cordic_sin[i]),
                .cordic_cos(cordic_cos[i]),
                .cordic_result(),
                .cordic_ready(cordic_ready[i]),
                .cordic_busy(cordic_busy[i]),
                
                // Matrix
                .matrix_valid(matrix_valid[i]),
                .matrix_a(local_matrix_a),
                .matrix_b(local_matrix_b),
                .matrix_c(local_matrix_c),
                .matrix_ready(matrix_ready[i]),
                .matrix_busy(matrix_busy[i]),
                
                // FMA
                .fma_valid(fma_valid[i]),
                .fma_a(fma_a[i]),
                .fma_b(fma_b[i]),
                .fma_c(fma_c[i]),
                .fma_result(fma_result[i]),
                .fma_ready(fma_ready[i]),
                
                .accel_error(accel_error[i])
            );
        end
    endgenerate

endmodule



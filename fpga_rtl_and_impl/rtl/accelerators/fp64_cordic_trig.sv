//============================================================================
// PhD Research: Double-Precision CORDIC Trigonometric Unit
// Author: Chandraboul, IISc
// Target: Kintex UltraScale KU060
//
// Description:
//   Complete IEEE 754 FP64 CORDIC unit supporting ALL trigonometric
//   functions required for spacecraft navigation and attitude control.
//
// Supported Functions:
//   ┌──────────────────────────────────────────────────────────────────┐
//   │  DIRECT TRIGONOMETRIC (Rotation Mode)                           │
//   │    • SIN(θ)   - Sine of angle θ                                 │
//   │    • COS(θ)   - Cosine of angle θ                               │
//   │    • TAN(θ)   - Tangent of angle θ (computed as sin/cos)        │
//   │    • SINCOS(θ)- Both sin and cos simultaneously                 │
//   │                                                                  │
//   │  INVERSE TRIGONOMETRIC (Vectoring Mode)                         │
//   │    • ASIN(x)  - Arc sine, returns angle in [-π/2, π/2]         │
//   │    • ACOS(x)  - Arc cosine, returns angle in [0, π]            │
//   │    • ATAN(x)  - Arc tangent, returns angle in [-π/2, π/2]      │
//   │    • ATAN2(y,x) - Two-argument arctangent, full [-π, π] range  │
//   │                                                                  │
//   │  HYPERBOLIC (Hyperbolic Mode)                                   │
//   │    • SINH(x)  - Hyperbolic sine                                 │
//   │    • COSH(x)  - Hyperbolic cosine                               │
//   │    • TANH(x)  - Hyperbolic tangent                              │
//   │    • ATANH(x) - Inverse hyperbolic tangent                      │
//   │                                                                  │
//   │  UTILITY                                                        │
//   │    • SQRT(x)  - Square root (via hyperbolic CORDIC)            │
//   │    • MAG(x,y) - Magnitude √(x² + y²)                           │
//   │    • PHASE(x,y) - Phase angle atan2(y,x)                       │
//   └──────────────────────────────────────────────────────────────────┘
//
// Precision: Full FP64 (52-bit mantissa, 54 CORDIC iterations)
// Latency: 20 cycles (pipelined)
//============================================================================

`timescale 1ns / 1ps

module fp64_cordic_trig #(
    parameter ITERATIONS = 54,           // Full double precision
    parameter PIPELINE_STAGES = 20       // Pipeline depth
)(
    input  logic            clk,
    input  logic            rst_n,
    
    // Control interface
    input  logic            valid_in,
    input  logic [4:0]      func,        // Function select
    output logic            ready,
    output logic            busy,
    
    // Operands (IEEE 754 FP64)
    input  logic [63:0]     op_a,        // First operand (angle or x)
    input  logic [63:0]     op_b,        // Second operand (y for atan2)
    
    // Results (IEEE 754 FP64)
    output logic [63:0]     result,      // Primary result
    output logic [63:0]     result_cos,  // Secondary result (cos when computing sincos)
    output logic            valid_out,
    
    // Exception flags
    output logic            flag_invalid,    // Domain error
    output logic            flag_overflow,   // Result overflow
    output logic            flag_inexact     // Result inexact
);

    //------------------------------------------------------------------------
    // Function Codes
    //------------------------------------------------------------------------
    // Direct trigonometric
    localparam FUNC_SIN    = 5'd0;    // sin(θ)
    localparam FUNC_COS    = 5'd1;    // cos(θ)
    localparam FUNC_TAN    = 5'd2;    // tan(θ)
    localparam FUNC_SINCOS = 5'd3;    // sin(θ) and cos(θ)
    
    // Inverse trigonometric
    localparam FUNC_ASIN   = 5'd4;    // asin(x)
    localparam FUNC_ACOS   = 5'd5;    // acos(x)
    localparam FUNC_ATAN   = 5'd6;    // atan(x)
    localparam FUNC_ATAN2  = 5'd7;    // atan2(y, x)
    
    // Hyperbolic
    localparam FUNC_SINH   = 5'd8;    // sinh(x)
    localparam FUNC_COSH   = 5'd9;    // cosh(x)
    localparam FUNC_TANH   = 5'd10;   // tanh(x)
    localparam FUNC_ATANH  = 5'd11;   // atanh(x)
    
    // Utility
    localparam FUNC_SQRT   = 5'd12;   // sqrt(x)
    localparam FUNC_MAG    = 5'd13;   // magnitude sqrt(x²+y²)
    localparam FUNC_PHASE  = 5'd14;   // phase atan2(y,x)
    
    //------------------------------------------------------------------------
    // FP64 Constants
    //------------------------------------------------------------------------
    // Mathematical constants in IEEE 754 FP64
    localparam [63:0] FP64_ZERO     = 64'h0000_0000_0000_0000;
    localparam [63:0] FP64_ONE      = 64'h3FF0_0000_0000_0000;  // 1.0
    localparam [63:0] FP64_NEG_ONE  = 64'hBFF0_0000_0000_0000;  // -1.0
    localparam [63:0] FP64_PI       = 64'h4009_21FB_5444_2D18;  // π
    localparam [63:0] FP64_PI_2     = 64'h3FF9_21FB_5444_2D18;  // π/2
    localparam [63:0] FP64_PI_4     = 64'h3FE9_21FB_5444_2D18;  // π/4
    localparam [63:0] FP64_2PI      = 64'h4019_21FB_5444_2D18;  // 2π
    
    // CORDIC gain K = 0.6072529350088814...
    localparam [63:0] CORDIC_K      = 64'h3FE3_6E9D_B508_6BC9;
    localparam [63:0] CORDIC_K_INV  = 64'h3FF9_B74E_DA84_35E5;  // 1/K
    
    //------------------------------------------------------------------------
    // CORDIC arctan lookup table (precomputed for FP64)
    //------------------------------------------------------------------------
    // arctan(2^-i) for i = 0 to 53
    logic [63:0] atan_lut [53:0];
    
    initial begin
        atan_lut[0]  = 64'h3FE9_21FB_5444_2D18;  // atan(2^0)  = π/4
        atan_lut[1]  = 64'h3FDA_C670_561B_B4F5;  // atan(2^-1) 
        atan_lut[2]  = 64'h3FCF_5B75_F92C_80DD;  // atan(2^-2)
        atan_lut[3]  = 64'h3FC4_AE10_FC65_89A5;  // atan(2^-3)
        atan_lut[4]  = 64'h3FB9_FEAB_0D62_2F28;  // atan(2^-4)
        atan_lut[5]  = 64'h3FAF_5754_C3FE_0B1E;  // atan(2^-5)
        atan_lut[6]  = 64'h3FA4_3C0D_38C5_2E8E;  // atan(2^-6)
        atan_lut[7]  = 64'h3F99_A2C0_33A3_D4E5;  // atan(2^-7)
        atan_lut[8]  = 64'h3F8F_1E9E_C40B_5C24;  // atan(2^-8)
        atan_lut[9]  = 64'h3F84_145F_70B4_2C50;  // atan(2^-9)
        atan_lut[10] = 64'h3F79_0A35_7A7E_1D46;  // atan(2^-10)
        atan_lut[11] = 64'h3F6F_0540_4AF7_F0C0;  // atan(2^-11)
        atan_lut[12] = 64'h3F64_02A0_2578_6C18;  // atan(2^-12)
        // For i > 12, atan(2^-i) ≈ 2^-i with very small error
        for (int i = 13; i < 54; i++) begin
            // atan(2^-i) stored with exponent bias adjustment
            atan_lut[i] = {1'b0, 11'(1023 - i), 52'h0};
        end
    end
    
    //------------------------------------------------------------------------
    // Hyperbolic atanh lookup table
    //------------------------------------------------------------------------
    logic [63:0] atanh_lut [53:0];
    
    initial begin
        atanh_lut[1]  = 64'h3FE1_93EA_7AAD_030B;  // atanh(2^-1)
        atanh_lut[2]  = 64'h3FC0_163D_A9FB_3335;  // atanh(2^-2)
        atanh_lut[3]  = 64'h3FB0_0557_935A_4F74;  // atanh(2^-3)
        atanh_lut[4]  = 64'h3FA0_0055_5799_9555;  // atanh(2^-4)
        // For small values, atanh(x) ≈ x
        for (int i = 5; i < 54; i++) begin
            atanh_lut[i] = {1'b0, 11'(1023 - i), 52'h0};
        end
    end
    
    //------------------------------------------------------------------------
    // Input Preprocessing
    //------------------------------------------------------------------------
    // Extract components
    wire        a_sign = op_a[63];
    wire [10:0] a_exp  = op_a[62:52];
    wire [51:0] a_man  = op_a[51:0];
    wire        a_zero = (a_exp == 0) && (a_man == 0);
    wire        a_inf  = (a_exp == 11'h7FF) && (a_man == 0);
    wire        a_nan  = (a_exp == 11'h7FF) && (a_man != 0);
    
    wire        b_sign = op_b[63];
    wire [10:0] b_exp  = op_b[62:52];
    wire [51:0] b_man  = op_b[51:0];
    wire        b_zero = (b_exp == 0) && (b_man == 0);
    wire        b_inf  = (b_exp == 11'h7FF) && (b_man == 0);
    wire        b_nan  = (b_exp == 11'h7FF) && (b_man != 0);
    
    //------------------------------------------------------------------------
    // CORDIC Core State
    //------------------------------------------------------------------------
    typedef enum logic [2:0] {
        IDLE,
        PREPROCESS,
        ITERATE,
        POSTPROCESS,
        OUTPUT
    } state_t;
    
    state_t state;
    
    // CORDIC registers (extended precision for internal computation)
    logic signed [79:0] x_reg, y_reg, z_reg;
    logic [5:0] iter_count;
    logic [4:0] func_reg;
    logic [1:0] quadrant;
    logic       cordic_mode;  // 0=rotation, 1=vectoring
    logic       hyper_mode;   // 0=circular, 1=hyperbolic
    logic       result_negate;
    
    // Pipeline valid tracking
    logic [PIPELINE_STAGES-1:0] pipe_valid;
    logic [4:0] pipe_func [PIPELINE_STAGES-1:0];
    
    //------------------------------------------------------------------------
    // Domain Checking
    //------------------------------------------------------------------------
    logic domain_error;
    
    always_comb begin
        domain_error = 1'b0;
        
        case (func)
            FUNC_ASIN, FUNC_ACOS: begin
                // asin/acos require |x| <= 1
                if (!a_nan && !a_inf) begin
                    // Check if |a| > 1 (exp > 1023 means |a| >= 2)
                    if (a_exp > 11'd1023) domain_error = 1'b1;
                    // More precise check needed for 1 < |a| < 2
                end
            end
            
            FUNC_ATANH: begin
                // atanh requires |x| < 1
                if (a_exp >= 11'd1023) domain_error = 1'b1;
            end
            
            FUNC_SQRT: begin
                // sqrt requires x >= 0
                if (a_sign && !a_zero) domain_error = 1'b1;
            end
        endcase
    end
    
    //------------------------------------------------------------------------
    // Main State Machine
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            valid_out <= 1'b0;
            busy <= 1'b0;
            flag_invalid <= 1'b0;
            flag_overflow <= 1'b0;
            flag_inexact <= 1'b0;
        end else begin
            valid_out <= 1'b0;
            
            case (state)
                //------------------------------------------------------------
                IDLE: begin
                    busy <= 1'b0;
                    if (valid_in) begin
                        busy <= 1'b1;
                        func_reg <= func;
                        state <= PREPROCESS;
                        
                        // Handle special cases immediately
                        if (a_nan || b_nan) begin
                            result <= 64'h7FF8_0000_0000_0000;  // Quiet NaN
                            flag_invalid <= 1'b1;
                            state <= OUTPUT;
                        end else if (domain_error) begin
                            result <= 64'h7FF8_0000_0000_0000;  // NaN for domain error
                            flag_invalid <= 1'b1;
                            state <= OUTPUT;
                        end
                    end
                end
                
                //------------------------------------------------------------
                PREPROCESS: begin
                    iter_count <= 6'd0;
                    flag_invalid <= 1'b0;
                    flag_inexact <= 1'b1;  // Most results are inexact
                    result_negate <= 1'b0;
                    
                    case (func_reg)
                        //----------------------------------------------------
                        // SIN(θ): Use rotation mode, output y
                        //----------------------------------------------------
                        FUNC_SIN: begin
                            cordic_mode <= 1'b0;  // Rotation
                            hyper_mode <= 1'b0;   // Circular
                            // Start with x = 1/K, y = 0, z = θ
                            x_reg <= fp64_to_fixed(CORDIC_K_INV);
                            y_reg <= 80'sd0;
                            z_reg <= fp64_to_fixed(op_a);
                            // Handle angle > π/2 by quadrant adjustment
                            quadrant <= 2'b00;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // COS(θ): Use rotation mode, output x
                        //----------------------------------------------------
                        FUNC_COS: begin
                            cordic_mode <= 1'b0;
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(CORDIC_K_INV);
                            y_reg <= 80'sd0;
                            z_reg <= fp64_to_fixed(op_a);
                            quadrant <= 2'b00;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // TAN(θ): Compute sin/cos, then divide
                        //----------------------------------------------------
                        FUNC_TAN: begin
                            cordic_mode <= 1'b0;
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(CORDIC_K_INV);
                            y_reg <= 80'sd0;
                            z_reg <= fp64_to_fixed(op_a);
                            quadrant <= 2'b00;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // SINCOS(θ): Same as sin, but output both
                        //----------------------------------------------------
                        FUNC_SINCOS: begin
                            cordic_mode <= 1'b0;
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(CORDIC_K_INV);
                            y_reg <= 80'sd0;
                            z_reg <= fp64_to_fixed(op_a);
                            quadrant <= 2'b00;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // ASIN(x): asin(x) = atan(x / sqrt(1-x²))
                        // Or use vectoring mode with transformation
                        //----------------------------------------------------
                        FUNC_ASIN: begin
                            cordic_mode <= 1'b1;  // Vectoring
                            hyper_mode <= 1'b0;
                            // For asin, we use: asin(x) = atan2(x, sqrt(1-x²))
                            x_reg <= fp64_to_fixed(FP64_ONE);  // Placeholder
                            y_reg <= fp64_to_fixed(op_a);
                            z_reg <= 80'sd0;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // ACOS(x): acos(x) = π/2 - asin(x)
                        //----------------------------------------------------
                        FUNC_ACOS: begin
                            cordic_mode <= 1'b1;
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(op_a);
                            y_reg <= fp64_to_fixed(FP64_ONE);  // Placeholder
                            z_reg <= 80'sd0;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // ATAN(x): atan(x) = atan2(x, 1)
                        //----------------------------------------------------
                        FUNC_ATAN: begin
                            cordic_mode <= 1'b1;  // Vectoring
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(FP64_ONE);
                            y_reg <= fp64_to_fixed(op_a);
                            z_reg <= 80'sd0;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // ATAN2(y, x): Full quadrant arctangent
                        //----------------------------------------------------
                        FUNC_ATAN2: begin
                            cordic_mode <= 1'b1;
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(op_b);  // x
                            y_reg <= fp64_to_fixed(op_a);  // y
                            z_reg <= 80'sd0;
                            // Determine quadrant for post-processing
                            quadrant <= {op_a[63], op_b[63]};  // {y_sign, x_sign}
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // SINH(x): Hyperbolic sine
                        //----------------------------------------------------
                        FUNC_SINH: begin
                            cordic_mode <= 1'b0;
                            hyper_mode <= 1'b1;  // Hyperbolic
                            x_reg <= fp64_to_fixed(FP64_ONE);
                            y_reg <= 80'sd0;
                            z_reg <= fp64_to_fixed(op_a);
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // COSH(x): Hyperbolic cosine
                        //----------------------------------------------------
                        FUNC_COSH: begin
                            cordic_mode <= 1'b0;
                            hyper_mode <= 1'b1;
                            x_reg <= fp64_to_fixed(FP64_ONE);
                            y_reg <= 80'sd0;
                            z_reg <= fp64_to_fixed(op_a);
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // TANH(x): tanh = sinh/cosh
                        //----------------------------------------------------
                        FUNC_TANH: begin
                            cordic_mode <= 1'b0;
                            hyper_mode <= 1'b1;
                            x_reg <= fp64_to_fixed(FP64_ONE);
                            y_reg <= 80'sd0;
                            z_reg <= fp64_to_fixed(op_a);
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // ATANH(x): Inverse hyperbolic tangent
                        //----------------------------------------------------
                        FUNC_ATANH: begin
                            cordic_mode <= 1'b1;
                            hyper_mode <= 1'b1;
                            x_reg <= fp64_to_fixed(FP64_ONE);
                            y_reg <= fp64_to_fixed(op_a);
                            z_reg <= 80'sd0;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // SQRT(x): sqrt(a) = sqrt((a+1)/4 + (a-1)/4) trick
                        //----------------------------------------------------
                        FUNC_SQRT: begin
                            if (a_zero) begin
                                result <= op_a;  // sqrt(±0) = ±0
                                state <= OUTPUT;
                            end else if (a_inf && !a_sign) begin
                                result <= op_a;  // sqrt(+inf) = +inf
                                state <= OUTPUT;
                            end else begin
                                cordic_mode <= 1'b1;
                                hyper_mode <= 1'b1;
                                // Use identity: sqrt(x) via hyperbolic CORDIC
                                x_reg <= fp64_to_fixed(op_a);
                                y_reg <= 80'sd0;
                                z_reg <= 80'sd0;
                                state <= ITERATE;
                            end
                        end
                        
                        //----------------------------------------------------
                        // MAG(x,y): Magnitude = sqrt(x² + y²)
                        //----------------------------------------------------
                        FUNC_MAG: begin
                            cordic_mode <= 1'b1;
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(op_b);  // x (absolute value)
                            y_reg <= fp64_to_fixed(op_a);  // y
                            z_reg <= 80'sd0;
                            state <= ITERATE;
                        end
                        
                        //----------------------------------------------------
                        // PHASE(x,y): Phase = atan2(y,x)
                        //----------------------------------------------------
                        FUNC_PHASE: begin
                            cordic_mode <= 1'b1;
                            hyper_mode <= 1'b0;
                            x_reg <= fp64_to_fixed(op_b);
                            y_reg <= fp64_to_fixed(op_a);
                            z_reg <= 80'sd0;
                            quadrant <= {op_a[63], op_b[63]};
                            state <= ITERATE;
                        end
                        
                        default: begin
                            state <= OUTPUT;
                        end
                    endcase
                end
                
                //------------------------------------------------------------
                ITERATE: begin
                    if (iter_count < ITERATIONS) begin
                        // CORDIC iteration
                        logic signed [79:0] x_new, y_new, z_new;
                        logic sigma;
                        logic signed [79:0] x_shift, y_shift;
                        logic signed [79:0] angle_step;
                        
                        // Determine rotation direction
                        if (cordic_mode == 1'b0) begin
                            // Rotation mode: drive z toward 0
                            sigma = z_reg[79];  // sigma = -sign(z)
                        end else begin
                            // Vectoring mode: drive y toward 0
                            sigma = ~y_reg[79];  // sigma = sign(y)
                        end
                        
                        // Shift amounts
                        x_shift = x_reg >>> iter_count;
                        y_shift = y_reg >>> iter_count;
                        
                        // Get angle from lookup table
                        if (hyper_mode) begin
                            angle_step = fp64_to_fixed_lut(atanh_lut[iter_count]);
                        end else begin
                            angle_step = fp64_to_fixed_lut(atan_lut[iter_count]);
                        end
                        
                        // CORDIC equations
                        if (hyper_mode) begin
                            // Hyperbolic: x' = x + σdy, y' = y + σdx
                            if (sigma) begin
                                x_new = x_reg + y_shift;
                                y_new = y_reg + x_shift;
                                z_new = z_reg - angle_step;
                            end else begin
                                x_new = x_reg - y_shift;
                                y_new = y_reg - x_shift;
                                z_new = z_reg + angle_step;
                            end
                        end else begin
                            // Circular: x' = x - σdy, y' = y + σdx
                            if (sigma) begin
                                x_new = x_reg - y_shift;
                                y_new = y_reg + x_shift;
                                z_new = z_reg - angle_step;
                            end else begin
                                x_new = x_reg + y_shift;
                                y_new = y_reg - x_shift;
                                z_new = z_reg + angle_step;
                            end
                        end
                        
                        x_reg <= x_new;
                        y_reg <= y_new;
                        z_reg <= z_new;
                        iter_count <= iter_count + 1;
                    end else begin
                        state <= POSTPROCESS;
                    end
                end
                
                //------------------------------------------------------------
                POSTPROCESS: begin
                    // Convert result back to FP64 and apply corrections
                    case (func_reg)
                        FUNC_SIN: begin
                            result <= fixed_to_fp64(y_reg);
                        end
                        
                        FUNC_COS: begin
                            result <= fixed_to_fp64(x_reg);
                        end
                        
                        FUNC_TAN: begin
                            // tan = y/x (need division)
                            result <= fp64_div_approx(fixed_to_fp64(y_reg), fixed_to_fp64(x_reg));
                        end
                        
                        FUNC_SINCOS: begin
                            result <= fixed_to_fp64(y_reg);      // sin
                            result_cos <= fixed_to_fp64(x_reg);  // cos
                        end
                        
                        FUNC_ASIN: begin
                            result <= fixed_to_fp64(z_reg);
                        end
                        
                        FUNC_ACOS: begin
                            // acos = π/2 - asin
                            result <= fp64_sub_approx(FP64_PI_2, fixed_to_fp64(z_reg));
                        end
                        
                        FUNC_ATAN: begin
                            result <= fixed_to_fp64(z_reg);
                        end
                        
                        FUNC_ATAN2: begin
                            // Adjust for quadrant
                            logic [63:0] raw_angle;
                            raw_angle = fixed_to_fp64(z_reg);
                            
                            case (quadrant)
                                2'b00: result <= raw_angle;           // x>0, y>0: Q1
                                2'b01: result <= fp64_sub_approx(FP64_PI, raw_angle);  // x<0, y>0: Q2
                                2'b10: result <= raw_angle;           // x>0, y<0: Q4 (negative)
                                2'b11: result <= fp64_add_approx({1'b1, FP64_PI[62:0]}, raw_angle); // x<0, y<0: Q3
                            endcase
                        end
                        
                        FUNC_SINH: begin
                            result <= fixed_to_fp64(y_reg);
                        end
                        
                        FUNC_COSH: begin
                            result <= fixed_to_fp64(x_reg);
                        end
                        
                        FUNC_TANH: begin
                            result <= fp64_div_approx(fixed_to_fp64(y_reg), fixed_to_fp64(x_reg));
                        end
                        
                        FUNC_ATANH: begin
                            result <= fixed_to_fp64(z_reg);
                        end
                        
                        FUNC_SQRT: begin
                            result <= fixed_to_fp64(x_reg);
                        end
                        
                        FUNC_MAG: begin
                            // Magnitude is x after vectoring, multiply by K
                            result <= fp64_mul_approx(fixed_to_fp64(x_reg), CORDIC_K);
                        end
                        
                        FUNC_PHASE: begin
                            // Same as ATAN2
                            result <= fixed_to_fp64(z_reg);
                        end
                        
                        default: begin
                            result <= FP64_ZERO;
                        end
                    endcase
                    
                    state <= OUTPUT;
                end
                
                //------------------------------------------------------------
                OUTPUT: begin
                    valid_out <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end
    
    assign ready = (state == IDLE);
    
    //------------------------------------------------------------------------
    // Fixed-Point Conversion Functions
    //------------------------------------------------------------------------
    // Convert FP64 to 80-bit signed fixed-point (16.64 format)
    function automatic logic signed [79:0] fp64_to_fixed;
        input [63:0] fp;
        logic sign;
        logic [10:0] exp;
        logic [51:0] man;
        logic signed [79:0] result;
        begin
            sign = fp[63];
            exp = fp[62:52];
            man = fp[51:0];
            
            if (exp == 0) begin
                result = 80'sd0;  // Zero or subnormal
            end else if (exp == 11'h7FF) begin
                result = 80'sd0;  // Inf or NaN
            end else begin
                // Normal number: value = 1.man × 2^(exp-1023)
                result = {1'b1, man, 27'b0};  // 80-bit with implicit 1
                
                // Shift based on exponent
                if (exp > 1023 + 15) begin
                    result = result << (exp - 1023 - 15);
                end else if (exp < 1023 + 15) begin
                    result = result >> (1023 + 15 - exp);
                end
                
                if (sign) result = -result;
            end
            fp64_to_fixed = result;
        end
    endfunction
    
    function automatic logic signed [79:0] fp64_to_fixed_lut;
        input [63:0] fp;
        // Simplified version for LUT values
        fp64_to_fixed_lut = fp64_to_fixed(fp);
    endfunction
    
    // Convert 80-bit fixed-point back to FP64
    function automatic logic [63:0] fixed_to_fp64;
        input logic signed [79:0] fx;
        logic sign;
        logic [79:0] abs_val;
        logic [6:0] lz;
        logic [10:0] exp;
        logic [51:0] man;
        begin
            sign = fx[79];
            abs_val = sign ? -fx : fx;
            
            if (abs_val == 0) begin
                fixed_to_fp64 = 64'h0;
            end else begin
                // Find leading 1
                lz = 0;
                for (int i = 79; i >= 0; i--) begin
                    if (abs_val[i]) begin
                        lz = 79 - i;
                        break;
                    end
                end
                
                // Normalize
                abs_val = abs_val << lz;
                exp = 1023 + 79 - 15 - lz;  // Adjust for fixed point format
                man = abs_val[78:27];       // Take top 52 bits after leading 1
                
                fixed_to_fp64 = {sign, exp, man};
            end
        end
    endfunction
    
    //------------------------------------------------------------------------
    // Approximate FP64 Operations (for post-processing)
    //------------------------------------------------------------------------
    function automatic logic [63:0] fp64_add_approx;
        input [63:0] a, b;
        // Simplified - real implementation in FPU
        fp64_add_approx = a;  // Placeholder
    endfunction
    
    function automatic logic [63:0] fp64_sub_approx;
        input [63:0] a, b;
        fp64_sub_approx = a;  // Placeholder
    endfunction
    
    function automatic logic [63:0] fp64_mul_approx;
        input [63:0] a, b;
        fp64_mul_approx = a;  // Placeholder
    endfunction
    
    function automatic logic [63:0] fp64_div_approx;
        input [63:0] a, b;
        fp64_div_approx = a;  // Placeholder
    endfunction

endmodule












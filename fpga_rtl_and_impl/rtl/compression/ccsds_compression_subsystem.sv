//============================================================================
// CCSDS Compression Subsystem - Wrapper for TC, TM, and Image compression
//============================================================================

`timescale 1ns / 1ps

module ccsds_compression_subsystem (
    input  logic        clk,
    input  logic        rst_n,
    
    // Control interface
    input  logic        valid_in,
    input  logic [1:0]  mode,       // 0=TC, 1=TM, 2=Image
    input  logic [7:0]  data_in,
    output logic [7:0]  data_out,
    output logic        ready,
    output logic        valid_out
);

    // Internal compression cores
    logic tc_valid, tm_valid, img_valid;
    logic [7:0] tc_out, tm_out, img_out;
    logic tc_ready, tm_ready, img_ready;
    logic tc_done, tm_done, img_done;

    // TC Compression (simple run-length encoding)
    ccsds_tc_encoder u_tc (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (valid_in && mode == 2'd0),
        .data_in    (data_in),
        .data_out   (tc_out),
        .ready      (tc_ready),
        .valid_out  (tc_done)
    );
    
    // TM Compression (Rice coding)
    ccsds_tm_encoder u_tm (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (valid_in && mode == 2'd1),
        .data_in    (data_in),
        .data_out   (tm_out),
        .ready      (tm_ready),
        .valid_out  (tm_done)
    );
    
    // Image Compression (DWT-based)
    ccsds_image_compressor u_img (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (valid_in && mode == 2'd2),
        .data_in    (data_in),
        .data_out   (img_out),
        .ready      (img_ready),
        .valid_out  (img_done)
    );
    
    // Output mux
    always_comb begin
        case (mode)
            2'd0: begin
                data_out = tc_out;
                ready = tc_ready;
                valid_out = tc_done;
            end
            2'd1: begin
                data_out = tm_out;
                ready = tm_ready;
                valid_out = tm_done;
            end
            2'd2: begin
                data_out = img_out;
                ready = img_ready;
                valid_out = img_done;
            end
            default: begin
                data_out = 8'd0;
                ready = 1'b1;
                valid_out = 1'b0;
            end
        endcase
    end

endmodule

//============================================================================
// CCSDS TC Encoder - Telecommand encoding
//============================================================================
module ccsds_tc_encoder (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [7:0]  data_in,
    output logic [7:0]  data_out,
    output logic        ready,
    output logic        valid_out
);
    // Simple pass-through with frame marker (placeholder)
    logic [3:0] count;
    logic [7:0] data_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 4'd0;
            data_reg <= 8'd0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                data_reg <= data_in;
                count <= count + 1;
            end
        end
    end
    
    assign data_out = (count == 4'd0) ? 8'hEB : data_reg;  // Sync marker
    assign ready = 1'b1;
endmodule

//============================================================================
// CCSDS TM Encoder - Telemetry encoding with Rice coding
//============================================================================
module ccsds_tm_encoder (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [7:0]  data_in,
    output logic [7:0]  data_out,
    output logic        ready,
    output logic        valid_out
);
    // Rice coding parameters
    localparam K = 3;  // Split parameter
    
    logic [7:0] prev_sample;
    logic [7:0] prediction;
    logic [7:0] residual;
    logic [7:0] mapped;
    
    // Predictor
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_sample <= 8'd128;
        end else if (valid_in) begin
            prev_sample <= data_in;
        end
    end
    
    assign prediction = prev_sample;
    
    // Residual calculation
    assign residual = data_in - prediction;
    
    // Signed-to-unsigned mapping for Rice coding
    assign mapped = (residual[7]) ? (~(residual << 1)) : (residual << 1);
    
    // Output
    logic valid_reg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_reg <= 1'b0;
        end else begin
            valid_reg <= valid_in;
        end
    end
    
    assign data_out = mapped;
    assign valid_out = valid_reg;
    assign ready = 1'b1;
endmodule

//============================================================================
// CCSDS Image Compressor - DWT-based compression
//============================================================================
module ccsds_image_compressor (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [7:0]  data_in,
    output logic [7:0]  data_out,
    output logic        ready,
    output logic        valid_out
);
    // Simplified Haar DWT (2-tap)
    logic [7:0] prev_pixel;
    logic       pixel_pair;
    logic [7:0] low_pass;
    logic [7:0] high_pass;
    logic [8:0] sum, diff;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_pixel <= 8'd0;
            pixel_pair <= 1'b0;
        end else if (valid_in) begin
            prev_pixel <= data_in;
            pixel_pair <= ~pixel_pair;
        end
    end
    
    // DWT calculation
    assign sum = {1'b0, prev_pixel} + {1'b0, data_in};
    assign diff = {1'b0, prev_pixel} - {1'b0, data_in};
    
    assign low_pass = sum[8:1];  // Average
    assign high_pass = diff[7:0] + 8'd128;  // Difference (biased)
    
    // Output alternates between low and high pass
    logic output_select;
    logic [7:0] data_reg;
    logic valid_reg;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_select <= 1'b0;
            data_reg <= 8'd0;
            valid_reg <= 1'b0;
        end else begin
            valid_reg <= valid_in && pixel_pair;
            if (valid_in && pixel_pair) begin
                output_select <= ~output_select;
                data_reg <= output_select ? high_pass : low_pass;
            end
        end
    end
    
    assign data_out = data_reg;
    assign valid_out = valid_reg;
    assign ready = 1'b1;
endmodule


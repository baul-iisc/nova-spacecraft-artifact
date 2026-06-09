//============================================================================
// Main Memory ECC Controller
// SECDED protection for DDR4/CXL main memory interface
// Space-Grade: Handles memory ECC errors, provides scrubbing
//============================================================================

`timescale 1ns / 1ps

module mem_ecc_controller #(
    parameter ADDR_WIDTH   = 32,
    parameter DATA_WIDTH   = 512,  // Cache line width (64 bytes)
    parameter ECC_WIDTH    = 64    // 8 ECC bits per 64-bit word × 8 words
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // L2 Cache Interface
    input  logic                    l2_req_valid,
    input  logic                    l2_req_write,
    input  logic [ADDR_WIDTH-1:0]   l2_req_addr,
    input  logic [DATA_WIDTH-1:0]   l2_req_wdata,
    output logic [DATA_WIDTH-1:0]   l2_resp_rdata,
    output logic                    l2_resp_valid,
    output logic                    l2_resp_ecc_error,  // Uncorrectable error
    input  logic                    l2_req_ready,
    
    // Physical Memory Interface (DDR4/CXL)
    output logic                    mem_req_valid,
    output logic                    mem_req_write,
    output logic [ADDR_WIDTH-1:0]   mem_req_addr,
    output logic [DATA_WIDTH+ECC_WIDTH-1:0] mem_req_data,  // Data + ECC
    input  logic [DATA_WIDTH+ECC_WIDTH-1:0] mem_resp_data, // Data + ECC
    input  logic                    mem_resp_valid,
    output logic                    mem_req_ready,
    
    // Scrubber Interface
    input  logic                    scrub_enable,
    input  logic [ADDR_WIDTH-1:0]   scrub_start_addr,
    input  logic [ADDR_WIDTH-1:0]   scrub_end_addr,
    output logic                    scrub_active,
    output logic [ADDR_WIDTH-1:0]   scrub_current_addr,
    output logic                    scrub_error_found,
    output logic [ADDR_WIDTH-1:0]   scrub_error_addr,
    
    // Error Status
    output logic                    single_error_irq,
    output logic                    double_error_irq,
    output logic [31:0]             single_error_count,
    output logic [31:0]             double_error_count,
    output logic [ADDR_WIDTH-1:0]   last_error_addr
);

    localparam WORDS_PER_LINE = DATA_WIDTH / 64;  // 8 words
    
    //------------------------------------------------------------------------
    // State Machine
    //------------------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE,
        READ_REQ,
        READ_WAIT,
        READ_CHECK,
        READ_CORRECT,
        WRITE_ENCODE,
        WRITE_REQ,
        WRITE_WAIT,
        SCRUB_READ_REQ,
        SCRUB_READ_WAIT,
        SCRUB_CHECK,
        SCRUB_CORRECT_WRITE,
        SCRUB_NEXT
    } state_t;
    
    state_t state, next_state;
    
    // Registered request
    logic [ADDR_WIDTH-1:0] req_addr_r;
    logic [DATA_WIDTH-1:0] req_wdata_r;
    logic                  req_write_r;
    
    // Scrub state
    logic [ADDR_WIDTH-1:0] scrub_addr_r;
    
    //------------------------------------------------------------------------
    // ECC Decode for each 64-bit word
    //------------------------------------------------------------------------
    logic [63:0] decoded_words [WORDS_PER_LINE-1:0];
    logic [WORDS_PER_LINE-1:0] word_single_err;
    logic [WORDS_PER_LINE-1:0] word_double_err;
    logic any_single_error;
    logic any_double_error;
    
    generate
        for (genvar w = 0; w < WORDS_PER_LINE; w++) begin : gen_mem_dec
            ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_dec (
                .clk(clk),
                .rst_n(rst_n),
                .data_in('0),
                .encoded_out(),
                .encoded_in(mem_resp_data[w*72 +: 72]),
                .data_out(decoded_words[w]),
                .single_error(word_single_err[w]),
                .double_error(word_double_err[w]),
                .error_position()
            );
        end
    endgenerate
    
    assign any_single_error = |word_single_err;
    assign any_double_error = |word_double_err;
    
    // Reconstruct corrected line
    logic [DATA_WIDTH-1:0] corrected_line;
    always_comb begin
        for (int w = 0; w < WORDS_PER_LINE; w++) begin
            corrected_line[w*64 +: 64] = decoded_words[w];
        end
    end

    //------------------------------------------------------------------------
    // ECC Encode for write
    //------------------------------------------------------------------------
    logic [DATA_WIDTH+ECC_WIDTH-1:0] encoded_line;
    
    generate
        for (genvar w = 0; w < WORDS_PER_LINE; w++) begin : gen_mem_enc
            logic [71:0] enc_word;
            ecc_secded #(.DATA_WIDTH(64), .ECC_WIDTH(8)) u_enc (
                .clk(clk),
                .rst_n(rst_n),
                .data_in(req_wdata_r[w*64 +: 64]),
                .encoded_out(enc_word),
                .encoded_in('0),
                .data_out(),
                .single_error(),
                .double_error(),
                .error_position()
            );
            assign encoded_line[w*72 +: 72] = enc_word;
        end
    endgenerate

    //------------------------------------------------------------------------
    // State machine
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            req_addr_r <= '0;
            req_wdata_r <= '0;
            req_write_r <= '0;
            scrub_addr_r <= '0;
        end else begin
            state <= next_state;
            
            // Capture L2 request
            if (state == IDLE && l2_req_valid) begin
                req_addr_r <= l2_req_addr;
                req_wdata_r <= l2_req_wdata;
                req_write_r <= l2_req_write;
            end
            
            // Scrub address management
            if (state == IDLE && scrub_enable && !l2_req_valid) begin
                scrub_addr_r <= scrub_start_addr;
            end else if (state == SCRUB_NEXT) begin
                scrub_addr_r <= scrub_addr_r + 64;  // Next cache line
            end
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (l2_req_valid) begin
                    if (l2_req_write)
                        next_state = WRITE_ENCODE;
                    else
                        next_state = READ_REQ;
                end else if (scrub_enable) begin
                    next_state = SCRUB_READ_REQ;
                end
            end
            
            // Normal read path
            READ_REQ:    next_state = READ_WAIT;
            READ_WAIT:   next_state = mem_resp_valid ? READ_CHECK : READ_WAIT;
            READ_CHECK: begin
                if (any_double_error)
                    next_state = IDLE;  // Return error to L2
                else if (any_single_error)
                    next_state = READ_CORRECT;  // Correct and writeback
                else
                    next_state = IDLE;
            end
            READ_CORRECT: next_state = WRITE_REQ;  // Write corrected data back
            
            // Normal write path
            WRITE_ENCODE: next_state = WRITE_REQ;
            WRITE_REQ:    next_state = WRITE_WAIT;
            WRITE_WAIT:   next_state = mem_resp_valid ? IDLE : WRITE_WAIT;
            
            // Scrub path
            SCRUB_READ_REQ:  next_state = SCRUB_READ_WAIT;
            SCRUB_READ_WAIT: next_state = mem_resp_valid ? SCRUB_CHECK : SCRUB_READ_WAIT;
            SCRUB_CHECK: begin
                if (any_single_error)
                    next_state = SCRUB_CORRECT_WRITE;
                else
                    next_state = SCRUB_NEXT;
            end
            SCRUB_CORRECT_WRITE: next_state = WRITE_WAIT;
            SCRUB_NEXT: begin
                if (scrub_addr_r >= scrub_end_addr || !scrub_enable)
                    next_state = IDLE;
                else
                    next_state = SCRUB_READ_REQ;
            end
            
            default: next_state = IDLE;
        endcase
    end

    //------------------------------------------------------------------------
    // Error tracking
    //------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            single_error_count <= '0;
            double_error_count <= '0;
            last_error_addr <= '0;
            scrub_error_found <= 1'b0;
            scrub_error_addr <= '0;
        end else begin
            // Clear pulse signals
            single_error_irq <= 1'b0;
            double_error_irq <= 1'b0;
            scrub_error_found <= 1'b0;
            
            // Normal read errors
            if (state == READ_CHECK) begin
                if (any_single_error) begin
                    single_error_count <= single_error_count + 1;
                    single_error_irq <= 1'b1;
                    last_error_addr <= req_addr_r;
                end
                if (any_double_error) begin
                    double_error_count <= double_error_count + 1;
                    double_error_irq <= 1'b1;
                    last_error_addr <= req_addr_r;
                end
            end
            
            // Scrub errors
            if (state == SCRUB_CHECK && any_single_error) begin
                single_error_count <= single_error_count + 1;
                scrub_error_found <= 1'b1;
                scrub_error_addr <= scrub_addr_r;
            end
        end
    end

    //------------------------------------------------------------------------
    // Memory interface
    //------------------------------------------------------------------------
    assign mem_req_valid = (state == READ_REQ) || (state == WRITE_REQ) || 
                           (state == SCRUB_READ_REQ);
    assign mem_req_write = (state == WRITE_REQ) && (req_write_r || state == READ_CORRECT);
    assign mem_req_addr  = (state == SCRUB_READ_REQ || state == SCRUB_CHECK) ? scrub_addr_r : req_addr_r;
    assign mem_req_data  = encoded_line;
    assign mem_req_ready = (state == READ_WAIT) || (state == WRITE_WAIT) || (state == SCRUB_READ_WAIT);

    //------------------------------------------------------------------------
    // L2 interface
    //------------------------------------------------------------------------
    assign l2_resp_rdata = corrected_line;
    assign l2_resp_valid = (state == READ_CHECK) || 
                           (state == WRITE_WAIT && mem_resp_valid && req_write_r);
    assign l2_resp_ecc_error = (state == READ_CHECK) && any_double_error;

    //------------------------------------------------------------------------
    // Scrub status
    //------------------------------------------------------------------------
    assign scrub_active = (state == SCRUB_READ_REQ) || (state == SCRUB_READ_WAIT) || 
                          (state == SCRUB_CHECK) || (state == SCRUB_CORRECT_WRITE) ||
                          (state == SCRUB_NEXT);
    assign scrub_current_addr = scrub_addr_r;

endmodule









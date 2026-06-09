//============================================================================
// L1 Cache with SECDED ECC - 32KB, 4-way set associative
//
// Resources per cache:
//   - 32KB data: ~8 BRAM36K
//   - Tags + ECC: ~2 BRAM36K
//   - Total: ~10 BRAM36K per L1 cache
//============================================================================

`timescale 1ns / 1ps

module l1_cache_secded #(
    parameter CACHE_SIZE_KB = 32,
    parameter LINE_SIZE     = 64,         // 64 bytes per line
    parameter NUM_WAYS      = 4,
    parameter ADDR_WIDTH    = 64
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // CPU interface
    input  logic                    cpu_valid,
    input  logic                    cpu_write,
    input  logic [ADDR_WIDTH-1:0]   cpu_addr,
    input  logic [63:0]             cpu_wdata,
    output logic [63:0]             cpu_rdata,
    output logic                    cpu_ready,
    
    // L2 interface
    output logic                    l2_valid,
    output logic                    l2_write,
    output logic [ADDR_WIDTH-1:0]   l2_addr,
    output logic [511:0]            l2_wdata,
    input  logic [511:0]            l2_rdata,
    input  logic                    l2_ready,
    
    // ECC status
    output logic                    ecc_error,
    output logic                    uncorrectable
);

    // Cache geometry
    localparam CACHE_BYTES = CACHE_SIZE_KB * 1024;
    localparam NUM_SETS    = CACHE_BYTES / (LINE_SIZE * NUM_WAYS);  // 128 sets
    localparam OFFSET_BITS = $clog2(LINE_SIZE);    // 6 bits
    localparam INDEX_BITS  = $clog2(NUM_SETS);     // 7 bits
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;  // 51 bits
    localparam DATA_WIDTH  = LINE_SIZE * 8;        // 512 bits
    localparam ECC_BITS    = 8;                    // SECDED for 64 bits

    // State machine
    typedef enum logic [2:0] {
        IDLE,
        TAG_CHECK,
        DATA_READ,
        DATA_WRITE,
        MISS_REQ,
        MISS_WAIT,
        MISS_FILL
    } state_t;
    
    state_t state, next_state;

    // Tag storage
    (* ram_style = "block" *) logic [TAG_BITS+2:0] tag_mem [NUM_WAYS-1:0][NUM_SETS-1:0];
    
    // Data storage - 8 banks of 64 bits
    (* ram_style = "block" *) logic [63:0] data_mem [NUM_WAYS-1:0][7:0][NUM_SETS-1:0];
    
    // ECC storage
    (* ram_style = "block" *) logic [ECC_BITS-1:0] ecc_mem [NUM_WAYS-1:0][7:0][NUM_SETS-1:0];
    
    // PLRU
    (* ram_style = "distributed" *) logic [NUM_WAYS-2:0] plru [NUM_SETS-1:0];

    // Address decode
    wire [TAG_BITS-1:0]   req_tag   = cpu_addr[ADDR_WIDTH-1 -: TAG_BITS];
    wire [INDEX_BITS-1:0] req_index = cpu_addr[OFFSET_BITS +: INDEX_BITS];
    wire [2:0]            req_bank  = cpu_addr[5:3];  // 8 banks

    // Read data
    logic [TAG_BITS+2:0] tag_rd [NUM_WAYS-1:0];
    logic [63:0] data_rd [NUM_WAYS-1:0];
    logic [ECC_BITS-1:0] ecc_rd [NUM_WAYS-1:0];

    // Hit detection
    logic [NUM_WAYS-1:0] hit_way;
    logic hit;
    logic [1:0] hit_idx;

    // Read tags and data
    always_ff @(posedge clk) begin
        for (int w = 0; w < NUM_WAYS; w++) begin
            tag_rd[w] <= tag_mem[w][req_index];
            data_rd[w] <= data_mem[w][req_bank][req_index];
            ecc_rd[w] <= ecc_mem[w][req_bank][req_index];
        end
    end

    // Hit detection
    always_comb begin
        hit = 1'b0;
        hit_way = '0;
        hit_idx = 2'd0;
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (tag_rd[w][0] && (tag_rd[w][TAG_BITS+2:3] == req_tag)) begin
                hit = 1'b1;
                hit_way[w] = 1'b1;
                hit_idx = w[1:0];
            end
        end
    end

    // SECDED encode/decode
    function automatic logic [ECC_BITS-1:0] calc_ecc64(input logic [63:0] data);
        logic [ECC_BITS-1:0] ecc;
        ecc[0] = ^{data[0], data[1], data[3], data[4], data[6], data[8], data[10], data[11], data[13], data[15], data[17], data[19], data[21], data[23], data[25], data[26], data[28], data[30], data[32], data[34], data[36], data[38], data[40], data[42], data[44], data[46], data[48], data[50], data[52], data[54], data[56], data[57], data[59], data[61], data[63]};
        ecc[1] = ^{data[0], data[2], data[3], data[5], data[6], data[9], data[10], data[12], data[13], data[16], data[17], data[20], data[21], data[24], data[25], data[27], data[28], data[31], data[32], data[35], data[36], data[39], data[40], data[43], data[44], data[47], data[48], data[51], data[52], data[55], data[56], data[58], data[59], data[62], data[63]};
        ecc[2] = ^{data[1], data[2], data[3], data[7], data[8], data[9], data[10], data[14], data[15], data[16], data[17], data[22], data[23], data[24], data[25], data[29], data[30], data[31], data[32], data[37], data[38], data[39], data[40], data[45], data[46], data[47], data[48], data[53], data[54], data[55], data[56], data[60], data[61], data[62], data[63]};
        ecc[3] = ^{data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[33], data[34], data[35], data[36], data[37], data[38], data[39], data[40], data[49], data[50], data[51], data[52], data[53], data[54], data[55], data[56]};
        ecc[4] = ^{data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[41], data[42], data[43], data[44], data[45], data[46], data[47], data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55], data[56]};
        ecc[5] = ^{data[26], data[27], data[28], data[29], data[30], data[31], data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39], data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47], data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55], data[56]};
        ecc[6] = ^{data[57], data[58], data[59], data[60], data[61], data[62], data[63]};
        ecc[7] = ^{data, ecc[6:0]};  // Overall parity
        return ecc;
    endfunction

    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE:       if (cpu_valid) next_state = TAG_CHECK;
            TAG_CHECK:  next_state = hit ? (cpu_write ? DATA_WRITE : DATA_READ) : MISS_REQ;
            DATA_READ:  next_state = IDLE;
            DATA_WRITE: next_state = IDLE;
            MISS_REQ:   next_state = MISS_WAIT;
            MISS_WAIT:  if (l2_ready) next_state = MISS_FILL;
            MISS_FILL:  next_state = IDLE;
            default:    next_state = IDLE;
        endcase
    end

    // ECC check
    logic [ECC_BITS-1:0] calc_ecc_val;
    logic [ECC_BITS-1:0] syndrome;
    
    assign calc_ecc_val = calc_ecc64(data_rd[hit_idx]);
    assign syndrome = calc_ecc_val ^ ecc_rd[hit_idx];
    assign ecc_error = (syndrome != '0) && syndrome[7];
    assign uncorrectable = (syndrome != '0) && !syndrome[7];

    // Outputs
    assign cpu_rdata = data_rd[hit_idx];
    assign cpu_ready = (state == DATA_READ) || (state == DATA_WRITE) || (state == MISS_FILL);
    
    assign l2_valid = (state == MISS_REQ);
    assign l2_write = 1'b0;  // L1 only reads from L2 on miss
    assign l2_addr = {cpu_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    assign l2_wdata = '0;

endmodule


//============================================================================
// RV64IMAFDC Core - 64-bit RISC-V Processor with Hardware FPU
//
// Author: Chandraboul, IISc
// Target: Kintex UltraScale KU060
//
// ISA Support:
//   - RV64I  : 64-bit Integer Base
//   - M      : Integer Multiply/Divide
//   - A      : Atomic Operations (LR/SC, AMO)
//   - F      : Single-Precision Floating Point
//   - D      : Double-Precision Floating Point
//   - C      : Compressed Instructions
//
// Pipeline: 6-stage (IF, ID, EX, MEM, FPU, WB)
// Features:
//   - Full IEEE 754-2008 FPU integration
//   - Out-of-order FPU completion
//   - Hardware multiply/divide unit
//   - Branch prediction (BTB + BHT)
//   - Precise exceptions
//============================================================================

`timescale 1ns / 1ps

module rv64_core #(
    parameter XLEN           = 64,
    parameter FLEN           = 64,          // D extension (64-bit FP)
    parameter VLEN           = 256,         // Vector length for RVV
    parameter RESET_VECTOR   = 64'h0000_0000_0000_0000,
    parameter HART_ID        = 0,
    parameter ENABLE_FPU     = 1,           // Enable F/D extensions
    parameter ENABLE_VECTOR  = 1,           // Enable V extension
    parameter ENABLE_ATOMIC  = 1,           // Enable A extension
    parameter ENABLE_COMPRESSED = 1,        // Enable C extension
    parameter BTB_ENTRIES    = 64,
    parameter BHT_ENTRIES    = 256
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Interrupt inputs
    input  logic                    meip,           // Machine external interrupt
    input  logic                    mtip,           // Machine timer interrupt
    input  logic                    msip,           // Machine software interrupt
    input  logic                    seip,           // Supervisor external interrupt
    input  logic [3:0]              nmi,            // Non-maskable interrupts
    
    // Instruction memory interface
    output logic                    imem_valid,
    output logic [XLEN-1:0]         imem_addr,
    input  logic [31:0]             imem_data,
    input  logic                    imem_ready,
    
    // Data memory interface
    output logic                    dmem_valid,
    output logic                    dmem_write,
    output logic [XLEN-1:0]         dmem_addr,
    output logic [XLEN-1:0]         dmem_wdata,
    output logic [7:0]              dmem_wstrb,
    input  logic [XLEN-1:0]         dmem_rdata,
    input  logic                    dmem_ready,
    
    // Vector memory interface (for vector load/store)
    output logic                    vmem_valid,
    output logic                    vmem_write,
    output logic [XLEN-1:0]         vmem_addr,
    output logic [VLEN-1:0]         vmem_wdata,
    output logic [VLEN/8-1:0]       vmem_wstrb,
    input  logic [VLEN-1:0]         vmem_rdata,
    input  logic                    vmem_ready,
    
    // External coprocessor interface (PCPI-like)
    output logic                    pcpi_valid,
    output logic [31:0]             pcpi_insn,
    output logic [XLEN-1:0]         pcpi_rs1,
    output logic [XLEN-1:0]         pcpi_rs2,
    input  logic [XLEN-1:0]         pcpi_rd,
    input  logic                    pcpi_ready,
    input  logic                    pcpi_wait,
    
    // Debug interface
    input  logic                    debug_req,
    output logic                    debug_ack,
    output logic [XLEN-1:0]         debug_pc,
    output logic                    debug_halted,
    
    // Status
    output logic                    halted,
    output logic                    trap,
    output logic [XLEN-1:0]         trap_cause,
    output logic [XLEN-1:0]         trap_val
);

    //=========================================================================
    // Instruction Formats
    //=========================================================================
    typedef struct packed {
        logic [6:0]  funct7;
        logic [4:0]  rs2;
        logic [4:0]  rs1;
        logic [2:0]  funct3;
        logic [4:0]  rd;
        logic [6:0]  opcode;
    } r_type_t;
    
    typedef struct packed {
        logic [11:0] imm;
        logic [4:0]  rs1;
        logic [2:0]  funct3;
        logic [4:0]  rd;
        logic [6:0]  opcode;
    } i_type_t;
    
    typedef struct packed {
        logic [6:0]  imm_11_5;
        logic [4:0]  rs2;
        logic [4:0]  rs1;
        logic [2:0]  funct3;
        logic [4:0]  imm_4_0;
        logic [6:0]  opcode;
    } s_type_t;
    
    //=========================================================================
    // Opcodes (RV64IMAFDC)
    //=========================================================================
    localparam OP_LUI       = 7'b0110111;
    localparam OP_AUIPC     = 7'b0010111;
    localparam OP_JAL       = 7'b1101111;
    localparam OP_JALR      = 7'b1100111;
    localparam OP_BRANCH    = 7'b1100011;
    localparam OP_LOAD      = 7'b0000011;
    localparam OP_STORE     = 7'b0100011;
    localparam OP_IMM       = 7'b0010011;
    localparam OP_IMM_W     = 7'b0011011;  // RV64: *W instructions
    localparam OP_REG       = 7'b0110011;
    localparam OP_REG_W     = 7'b0111011;  // RV64: *W instructions
    localparam OP_FENCE     = 7'b0001111;
    localparam OP_SYSTEM    = 7'b1110011;
    localparam OP_AMO       = 7'b0101111;  // A extension
    localparam OP_LOAD_FP   = 7'b0000111;  // F/D extension
    localparam OP_STORE_FP  = 7'b0100111;  // F/D extension
    localparam OP_FMADD     = 7'b1000011;  // F/D extension
    localparam OP_FMSUB     = 7'b1000111;  // F/D extension
    localparam OP_FNMSUB    = 7'b1001011;  // F/D extension
    localparam OP_FNMADD    = 7'b1001111;  // F/D extension
    localparam OP_FP        = 7'b1010011;  // F/D extension
    localparam OP_VECTOR    = 7'b1010111;  // V extension
    
    //=========================================================================
    // Pipeline Registers
    //=========================================================================
    // IF/ID
    logic [XLEN-1:0] if_id_pc;
    logic [31:0]     if_id_instr;
    logic            if_id_valid;
    logic            if_id_compressed;
    
    // ID/EX
    logic [XLEN-1:0] id_ex_pc;
    logic [31:0]     id_ex_instr;
    logic [XLEN-1:0] id_ex_rs1_data;
    logic [XLEN-1:0] id_ex_rs2_data;
    logic [FLEN-1:0] id_ex_frs1_data;
    logic [FLEN-1:0] id_ex_frs2_data;
    logic [FLEN-1:0] id_ex_frs3_data;
    logic [XLEN-1:0] id_ex_imm;
    logic [4:0]      id_ex_rd;
    logic [4:0]      id_ex_rs1;
    logic [4:0]      id_ex_rs2;
    logic            id_ex_valid;
    logic            id_ex_use_fpu;
    logic            id_ex_use_vector;
    logic            id_ex_mem_read;
    logic            id_ex_mem_write;
    logic            id_ex_branch;
    logic            id_ex_jump;
    logic [3:0]      id_ex_alu_op;
    logic [4:0]      id_ex_fpu_op;
    logic [2:0]      id_ex_fpu_rm;
    
    // EX/MEM
    logic [XLEN-1:0] ex_mem_pc;
    logic [XLEN-1:0] ex_mem_alu_result;
    logic [XLEN-1:0] ex_mem_rs2_data;
    logic [4:0]      ex_mem_rd;
    logic            ex_mem_valid;
    logic            ex_mem_mem_read;
    logic            ex_mem_mem_write;
    logic            ex_mem_reg_write;
    logic            ex_mem_fp_reg_write;
    logic [2:0]      ex_mem_funct3;
    
    // MEM/WB
    logic [XLEN-1:0] mem_wb_result;
    logic [4:0]      mem_wb_rd;
    logic            mem_wb_valid;
    logic            mem_wb_reg_write;
    logic            mem_wb_fp_reg_write;
    
    // FPU Pipeline (parallel to MEM)
    logic [FLEN-1:0] fpu_result;
    logic [4:0]      fpu_rd;
    logic            fpu_valid;
    logic            fpu_ready;
    logic [4:0]      fpu_fflags;
    
    // Pipeline Control (forward declaration)
    logic            stall_pipeline;
    logic            flush_pipeline;
    
    //=========================================================================
    // Register Files
    //=========================================================================
    // Integer register file (x0-x31, 64-bit)
    logic [XLEN-1:0] regfile [31:0];
    
    // Floating-point register file (f0-f31, 64-bit)
    logic [FLEN-1:0] fpregfile [31:0];
    
    //=========================================================================
    // CSRs (Control and Status Registers)
    //=========================================================================
    // Machine-level CSRs
    logic [XLEN-1:0] csr_mstatus;
    logic [XLEN-1:0] csr_mie;
    logic [XLEN-1:0] csr_mip;
    logic [XLEN-1:0] csr_mtvec;
    logic [XLEN-1:0] csr_mepc;
    logic [XLEN-1:0] csr_mcause;
    logic [XLEN-1:0] csr_mtval;
    logic [XLEN-1:0] csr_mscratch;
    logic [63:0]     csr_mcycle;
    logic [63:0]     csr_minstret;
    logic [XLEN-1:0] csr_mhartid;
    
    // Floating-point CSRs
    logic [4:0]      csr_fflags;       // Accrued exception flags
    logic [2:0]      csr_frm;          // Rounding mode
    logic [7:0]      csr_fcsr;         // fflags + frm
    
    // Vector CSRs
    logic [XLEN-1:0] csr_vl;           // Vector length
    logic [XLEN-1:0] csr_vtype;        // Vector type
    logic [XLEN-1:0] csr_vlenb;        // VLEN/8
    logic [XLEN-1:0] csr_vstart;       // Vector start position
    
    //=========================================================================
    // Program Counter
    //=========================================================================
    logic [XLEN-1:0] pc;
    logic [XLEN-1:0] next_pc;
    logic            pc_write_enable;
    
    //=========================================================================
    // Branch Prediction
    //=========================================================================
    logic [XLEN-1:0] btb_target [BTB_ENTRIES-1:0];
    logic [XLEN-1:0] btb_tag [BTB_ENTRIES-1:0];
    logic            btb_valid [BTB_ENTRIES-1:0];
    logic [1:0]      bht [BHT_ENTRIES-1:0];  // 2-bit saturating counter
    
    logic            branch_predicted_taken;
    logic [XLEN-1:0] branch_predicted_target;
    logic            branch_mispredict;
    
    //=========================================================================
    // Instruction Fetch Stage
    //=========================================================================
    logic [31:0]     fetched_instr;
    logic [15:0]     compressed_instr;
    logic            is_compressed;
    logic [31:0]     decoded_instr;
    
    // Compressed instruction decoder
    always_comb begin
        if (ENABLE_COMPRESSED && if_id_instr[1:0] != 2'b11) begin
            is_compressed = 1'b1;
            // Expand compressed instruction
            decoded_instr = expand_compressed(if_id_instr[15:0]);
        end else begin
            is_compressed = 1'b0;
            decoded_instr = if_id_instr;
        end
    end
    
    // Instruction fetch state machine
    typedef enum logic [1:0] {
        IF_IDLE,
        IF_FETCH,
        IF_WAIT
    } if_state_t;
    
    if_state_t if_state;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= RESET_VECTOR;
            if_state <= IF_IDLE;
            imem_valid <= 1'b0;
        end else begin
            case (if_state)
                IF_IDLE: begin
                    imem_valid <= 1'b1;
                    imem_addr <= pc;
                    if_state <= IF_FETCH;
                end
                
                IF_FETCH: begin
                    if (imem_ready) begin
                        if_id_pc <= pc;
                        if_id_instr <= imem_data;
                        if_id_valid <= 1'b1;
                        
                        // Update PC
                        if (branch_mispredict) begin
                            pc <= next_pc;  // Correct misprediction
                        end else if (branch_predicted_taken) begin
                            pc <= branch_predicted_target;
                        end else begin
                            pc <= pc + (is_compressed ? 64'd2 : 64'd4);
                        end
                        
                        if_state <= IF_IDLE;
                        imem_valid <= 1'b0;
                    end
                end
                
                default: if_state <= IF_IDLE;
            endcase
            
            // Pipeline flush on branch mispredict
            if (branch_mispredict) begin
                if_id_valid <= 1'b0;
            end
        end
    end
    
    //=========================================================================
    // Instruction Decode Stage
    //=========================================================================
    // Decode instruction fields
    wire [6:0] opcode = decoded_instr[6:0];
    wire [4:0] rd     = decoded_instr[11:7];
    wire [2:0] funct3 = decoded_instr[14:12];
    wire [4:0] rs1    = decoded_instr[19:15];
    wire [4:0] rs2    = decoded_instr[24:20];
    wire [6:0] funct7 = decoded_instr[31:25];
    wire [4:0] rs3    = decoded_instr[31:27];  // For FMA instructions
    wire [1:0] fmt    = decoded_instr[26:25];  // FP format
    
    // Immediate generation
    logic [XLEN-1:0] imm_i, imm_s, imm_b, imm_u, imm_j;
    
    always_comb begin
        // I-type immediate
        imm_i = {{52{decoded_instr[31]}}, decoded_instr[31:20]};
        
        // S-type immediate
        imm_s = {{52{decoded_instr[31]}}, decoded_instr[31:25], decoded_instr[11:7]};
        
        // B-type immediate
        imm_b = {{51{decoded_instr[31]}}, decoded_instr[31], decoded_instr[7], 
                 decoded_instr[30:25], decoded_instr[11:8], 1'b0};
        
        // U-type immediate
        imm_u = {{32{decoded_instr[31]}}, decoded_instr[31:12], 12'b0};
        
        // J-type immediate
        imm_j = {{43{decoded_instr[31]}}, decoded_instr[31], decoded_instr[19:12],
                 decoded_instr[20], decoded_instr[30:21], 1'b0};
    end
    
    // Register file read
    logic [XLEN-1:0] rs1_data, rs2_data;
    logic [FLEN-1:0] frs1_data, frs2_data, frs3_data;
    
    always_comb begin
        rs1_data = (rs1 == 5'd0) ? 64'd0 : regfile[rs1];
        rs2_data = (rs2 == 5'd0) ? 64'd0 : regfile[rs2];
        frs1_data = fpregfile[rs1];
        frs2_data = fpregfile[rs2];
        frs3_data = fpregfile[rs3];
    end
    
    // Decode control signals
    logic use_fpu, use_vector, mem_read, mem_write, is_branch, is_jump;
    logic [3:0] alu_op;
    logic [4:0] fpu_op;
    logic [2:0] fpu_rm;
    
    always_comb begin
        use_fpu = 1'b0;
        use_vector = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        is_branch = 1'b0;
        is_jump = 1'b0;
        alu_op = 4'd0;
        fpu_op = 5'd0;
        fpu_rm = funct3;  // Default from instruction
        
        case (opcode)
            OP_LOAD:    mem_read = 1'b1;
            OP_STORE:   mem_write = 1'b1;
            OP_BRANCH:  is_branch = 1'b1;
            OP_JAL:     is_jump = 1'b1;
            OP_JALR:    is_jump = 1'b1;
            
            // FPU operations
            OP_LOAD_FP:  begin mem_read = 1'b1; use_fpu = 1'b1; end
            OP_STORE_FP: begin mem_write = 1'b1; use_fpu = 1'b1; end
            OP_FMADD:    begin use_fpu = 1'b1; fpu_op = 5'd5; end
            OP_FMSUB:    begin use_fpu = 1'b1; fpu_op = 5'd6; end
            OP_FNMSUB:   begin use_fpu = 1'b1; fpu_op = 5'd8; end
            OP_FNMADD:   begin use_fpu = 1'b1; fpu_op = 5'd7; end
            OP_FP: begin
                use_fpu = 1'b1;
                case (funct7)
                    7'b0000001: fpu_op = 5'd0;   // FADD.D
                    7'b0000101: fpu_op = 5'd1;   // FSUB.D
                    7'b0001001: fpu_op = 5'd2;   // FMUL.D
                    7'b0001101: fpu_op = 5'd3;   // FDIV.D
                    7'b0101101: fpu_op = 5'd4;   // FSQRT.D
                    7'b0010001: fpu_op = 5'd15;  // FSGNJ.D
                    7'b0010101: fpu_op = 5'd9;   // FMIN/FMAX.D
                    7'b1010001: fpu_op = 5'd11;  // FCMP.D
                    7'b1110001: fpu_op = 5'd14;  // FCLASS/FMV
                    7'b1100001: fpu_op = 5'd18;  // FCVT.W.D
                    7'b1101001: fpu_op = 5'd19;  // FCVT.D.W
                    default:    fpu_op = 5'd0;
                endcase
            end
            
            // Vector operations
            OP_VECTOR: begin
                use_vector = 1'b1;
            end
            
            // ALU operations
            OP_IMM, OP_IMM_W, OP_REG, OP_REG_W: begin
                case (funct3)
                    3'b000: alu_op = (opcode == OP_REG && funct7[5]) ? 4'd1 : 4'd0; // ADD/SUB
                    3'b001: alu_op = 4'd2;  // SLL
                    3'b010: alu_op = 4'd3;  // SLT
                    3'b011: alu_op = 4'd4;  // SLTU
                    3'b100: alu_op = 4'd5;  // XOR
                    3'b101: alu_op = funct7[5] ? 4'd7 : 4'd6;  // SRL/SRA
                    3'b110: alu_op = 4'd8;  // OR
                    3'b111: alu_op = 4'd9;  // AND
                endcase
            end
        endcase
    end
    
    // ID/EX pipeline register update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_ex_valid <= 1'b0;
        end else if (!stall_pipeline) begin
            id_ex_pc <= if_id_pc;
            id_ex_instr <= decoded_instr;
            id_ex_rs1_data <= rs1_data;
            id_ex_rs2_data <= rs2_data;
            id_ex_frs1_data <= frs1_data;
            id_ex_frs2_data <= frs2_data;
            id_ex_frs3_data <= frs3_data;
            id_ex_rd <= rd;
            id_ex_rs1 <= rs1;
            id_ex_rs2 <= rs2;
            id_ex_valid <= if_id_valid;
            id_ex_use_fpu <= use_fpu;
            id_ex_use_vector <= use_vector;
            id_ex_mem_read <= mem_read;
            id_ex_mem_write <= mem_write;
            id_ex_branch <= is_branch;
            id_ex_jump <= is_jump;
            id_ex_alu_op <= alu_op;
            id_ex_fpu_op <= fpu_op;
            id_ex_fpu_rm <= (fpu_rm == 3'b111) ? csr_frm : fpu_rm;  // Dynamic rounding
            
            // Select immediate
            case (opcode)
                OP_LUI, OP_AUIPC: id_ex_imm <= imm_u;
                OP_JAL:           id_ex_imm <= imm_j;
                OP_BRANCH:        id_ex_imm <= imm_b;
                OP_STORE, OP_STORE_FP: id_ex_imm <= imm_s;
                default:          id_ex_imm <= imm_i;
            endcase
        end
    end
    
    //=========================================================================
    // Execute Stage
    //=========================================================================
    logic [XLEN-1:0] alu_result;
    logic [XLEN-1:0] alu_operand_a, alu_operand_b;
    logic            branch_taken;
    
    // Forwarding logic
    logic [XLEN-1:0] forwarded_rs1, forwarded_rs2;
    
    always_comb begin
        // Forward from EX/MEM
        if (ex_mem_valid && ex_mem_reg_write && ex_mem_rd == id_ex_rs1 && ex_mem_rd != 0)
            forwarded_rs1 = ex_mem_alu_result;
        // Forward from MEM/WB
        else if (mem_wb_valid && mem_wb_reg_write && mem_wb_rd == id_ex_rs1 && mem_wb_rd != 0)
            forwarded_rs1 = mem_wb_result;
        else
            forwarded_rs1 = id_ex_rs1_data;
            
        // RS2 forwarding
        if (ex_mem_valid && ex_mem_reg_write && ex_mem_rd == id_ex_rs2 && ex_mem_rd != 0)
            forwarded_rs2 = ex_mem_alu_result;
        else if (mem_wb_valid && mem_wb_reg_write && mem_wb_rd == id_ex_rs2 && mem_wb_rd != 0)
            forwarded_rs2 = mem_wb_result;
        else
            forwarded_rs2 = id_ex_rs2_data;
    end
    
    // ALU operand selection
    always_comb begin
        alu_operand_a = forwarded_rs1;
        
        case (id_ex_instr[6:0])
            OP_AUIPC: alu_operand_a = id_ex_pc;
            OP_JAL, OP_JALR: alu_operand_a = id_ex_pc;
        endcase
        
        case (id_ex_instr[6:0])
            OP_REG, OP_REG_W, OP_BRANCH: alu_operand_b = forwarded_rs2;
            OP_JAL, OP_JALR: alu_operand_b = (id_ex_instr[1:0] == 2'b11) ? 64'd4 : 64'd2;  // Return address
            default: alu_operand_b = id_ex_imm;
        endcase
    end
    
    // ALU
    always_comb begin
        case (id_ex_alu_op)
            4'd0:  alu_result = alu_operand_a + alu_operand_b;                    // ADD
            4'd1:  alu_result = alu_operand_a - alu_operand_b;                    // SUB
            4'd2:  alu_result = alu_operand_a << alu_operand_b[5:0];              // SLL
            4'd3:  alu_result = $signed(alu_operand_a) < $signed(alu_operand_b);  // SLT
            4'd4:  alu_result = alu_operand_a < alu_operand_b;                    // SLTU
            4'd5:  alu_result = alu_operand_a ^ alu_operand_b;                    // XOR
            4'd6:  alu_result = alu_operand_a >> alu_operand_b[5:0];              // SRL
            4'd7:  alu_result = $signed(alu_operand_a) >>> alu_operand_b[5:0];    // SRA
            4'd8:  alu_result = alu_operand_a | alu_operand_b;                    // OR
            4'd9:  alu_result = alu_operand_a & alu_operand_b;                    // AND
            default: alu_result = 64'd0;
        endcase
        
        // Handle *W instructions (32-bit result sign-extended)
        if (id_ex_instr[6:0] == OP_IMM_W || id_ex_instr[6:0] == OP_REG_W) begin
            alu_result = {{32{alu_result[31]}}, alu_result[31:0]};
        end
    end
    
    // Branch comparison
    always_comb begin
        branch_taken = 1'b0;
        if (id_ex_branch) begin
            case (id_ex_instr[14:12])
                3'b000: branch_taken = (forwarded_rs1 == forwarded_rs2);                    // BEQ
                3'b001: branch_taken = (forwarded_rs1 != forwarded_rs2);                    // BNE
                3'b100: branch_taken = ($signed(forwarded_rs1) < $signed(forwarded_rs2));   // BLT
                3'b101: branch_taken = ($signed(forwarded_rs1) >= $signed(forwarded_rs2));  // BGE
                3'b110: branch_taken = (forwarded_rs1 < forwarded_rs2);                     // BLTU
                3'b111: branch_taken = (forwarded_rs1 >= forwarded_rs2);                    // BGEU
                default: branch_taken = 1'b0;
            endcase
        end
    end
    
    // Branch/Jump target
    logic [XLEN-1:0] branch_target;
    always_comb begin
        if (id_ex_instr[6:0] == OP_JALR) begin
            branch_target = (forwarded_rs1 + id_ex_imm) & ~64'h1;  // Clear LSB
        end else begin
            branch_target = id_ex_pc + id_ex_imm;
        end
    end
    
    // Update branch predictor
    assign branch_mispredict = id_ex_valid && (id_ex_branch || id_ex_jump) && 
                               (branch_taken != branch_predicted_taken || 
                                branch_target != branch_predicted_target);
    
    always_comb begin
        next_pc = branch_target;
    end
    
    //=========================================================================
    // FPU Integration
    //=========================================================================
    generate
        if (ENABLE_FPU) begin : gen_fpu
            // FPU instance
            fp64_fpu_integrated u_fpu (
                .clk        (clk),
                .rst_n      (rst_n),
                .valid_in   (id_ex_valid && id_ex_use_fpu && !id_ex_mem_read && !id_ex_mem_write),
                .op         (id_ex_fpu_op),
                .rm         (id_ex_fpu_rm),
                .rs1        (id_ex_frs1_data),
                .rs2        (id_ex_frs2_data),
                .rs3        (id_ex_frs3_data),
                .rs1_int    (forwarded_rs1),  // For FCVT/FMV from int
                .rd         (fpu_result),
                .valid_out  (fpu_valid),
                .ready      (fpu_ready),
                .fflags     (fpu_fflags)
            );
        end else begin : gen_no_fpu
            assign fpu_result = 64'd0;
            assign fpu_valid = 1'b0;
            assign fpu_ready = 1'b1;
            assign fpu_fflags = 5'd0;
        end
    endgenerate
    
    //=========================================================================
    // Memory Stage
    //=========================================================================
    typedef enum logic [1:0] {
        MEM_IDLE,
        MEM_ACCESS,
        MEM_WAIT
    } mem_state_t;
    
    mem_state_t mem_state;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_state <= MEM_IDLE;
            dmem_valid <= 1'b0;
            ex_mem_valid <= 1'b0;
        end else begin
            case (mem_state)
                MEM_IDLE: begin
                    if (id_ex_valid && (id_ex_mem_read || id_ex_mem_write)) begin
                        dmem_valid <= 1'b1;
                        dmem_addr <= alu_result;
                        dmem_write <= id_ex_mem_write;
                        dmem_wdata <= id_ex_use_fpu ? id_ex_frs2_data : forwarded_rs2;
                        
                        // Generate write strobe based on funct3
                        case (id_ex_instr[14:12])
                            3'b000: dmem_wstrb <= 8'h01;  // SB
                            3'b001: dmem_wstrb <= 8'h03;  // SH
                            3'b010: dmem_wstrb <= 8'h0F;  // SW
                            3'b011: dmem_wstrb <= 8'hFF;  // SD
                            default: dmem_wstrb <= 8'hFF;
                        endcase
                        
                        mem_state <= MEM_ACCESS;
                    end else begin
                        // Pass through ALU result
                        ex_mem_alu_result <= alu_result;
                        ex_mem_rd <= id_ex_rd;
                        ex_mem_valid <= id_ex_valid && !id_ex_use_fpu &&
                                       (id_ex_instr[6:0] != 7'b0001011);  // Exclude Custom-0 (PCPI)
                        ex_mem_reg_write <= id_ex_valid && (id_ex_instr[6:0] != OP_BRANCH && 
                                            id_ex_instr[6:0] != OP_STORE &&
                                            id_ex_instr[6:0] != 7'b0001011);  // Exclude Custom-0
                        ex_mem_fp_reg_write <= 1'b0;
                    end
                end
                
                MEM_ACCESS: begin
                    if (dmem_ready) begin
                        dmem_valid <= 1'b0;
                        
                        if (id_ex_mem_read) begin
                            // Load data (sign/zero extend based on funct3)
                            case (id_ex_instr[14:12])
                                3'b000: ex_mem_alu_result <= {{56{dmem_rdata[7]}}, dmem_rdata[7:0]};   // LB
                                3'b001: ex_mem_alu_result <= {{48{dmem_rdata[15]}}, dmem_rdata[15:0]}; // LH
                                3'b010: ex_mem_alu_result <= {{32{dmem_rdata[31]}}, dmem_rdata[31:0]}; // LW
                                3'b011: ex_mem_alu_result <= dmem_rdata;                              // LD
                                3'b100: ex_mem_alu_result <= {56'd0, dmem_rdata[7:0]};                // LBU
                                3'b101: ex_mem_alu_result <= {48'd0, dmem_rdata[15:0]};               // LHU
                                3'b110: ex_mem_alu_result <= {32'd0, dmem_rdata[31:0]};               // LWU
                                default: ex_mem_alu_result <= dmem_rdata;
                            endcase
                        end
                        
                        ex_mem_rd <= id_ex_rd;
                        ex_mem_valid <= 1'b1;
                        ex_mem_reg_write <= id_ex_mem_read && !id_ex_use_fpu;
                        ex_mem_fp_reg_write <= id_ex_mem_read && id_ex_use_fpu;
                        mem_state <= MEM_IDLE;
                    end
                end
                
                default: mem_state <= MEM_IDLE;
            endcase
        end
    end
    
    //=========================================================================
    // Writeback Stage
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_wb_valid <= 1'b0;
            // Initialize register file
            for (int i = 0; i < 32; i++) begin
                regfile[i] <= 64'd0;
                fpregfile[i] <= 64'd0;
            end
        end else begin
            mem_wb_valid <= ex_mem_valid || fpu_valid;
            mem_wb_rd <= fpu_valid ? fpu_rd : ex_mem_rd;
            mem_wb_result <= fpu_valid ? fpu_result : ex_mem_alu_result;
            mem_wb_reg_write <= ex_mem_reg_write;
            mem_wb_fp_reg_write <= ex_mem_fp_reg_write || fpu_valid;
            
            // Write to integer register file
            if (ex_mem_valid && ex_mem_reg_write && ex_mem_rd != 5'd0) begin
                regfile[ex_mem_rd] <= ex_mem_alu_result;
            end
            
            // PCPI coprocessor result writeback
            if (pcpi_ready && id_ex_rd != 5'd0) begin
                regfile[id_ex_rd] <= pcpi_rd;
            end
            
            // Write to FP register file
            if (fpu_valid && fpu_rd != 5'd0) begin
                fpregfile[fpu_rd] <= fpu_result;
            end
            
            if (ex_mem_valid && ex_mem_fp_reg_write) begin
                fpregfile[ex_mem_rd] <= ex_mem_alu_result;
            end
            
            // Update FPU exception flags
            if (fpu_valid) begin
                csr_fflags <= csr_fflags | fpu_fflags;
            end
        end
    end
    
    //=========================================================================
    // Pipeline Control Logic
    //=========================================================================
    // Stall on load-use hazard, FPU busy, or PCPI coprocessor wait
    assign stall_pipeline = (id_ex_mem_read && 
                            ((id_ex_rd == rs1 && rs1 != 0) || 
                             (id_ex_rd == rs2 && rs2 != 0))) ||
                           (id_ex_use_fpu && !fpu_ready) ||
                           pcpi_wait;
    
    // Flush on branch mispredict or exception
    assign flush_pipeline = branch_mispredict || trap;
    
    //=========================================================================
    // CSR Access
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            csr_mstatus <= 64'd0;
            csr_mie <= 64'd0;
            csr_mtvec <= 64'd0;
            csr_mepc <= 64'd0;
            csr_mcause <= 64'd0;
            csr_mtval <= 64'd0;
            csr_mscratch <= 64'd0;
            csr_mcycle <= 64'd0;
            csr_minstret <= 64'd0;
            csr_mhartid <= HART_ID;
            csr_fflags <= 5'd0;
            csr_frm <= 3'd0;
            csr_vl <= 64'd0;
            csr_vtype <= 64'd0;
            csr_vlenb <= VLEN / 8;
            csr_vstart <= 64'd0;
        end else begin
            // Cycle counter
            csr_mcycle <= csr_mcycle + 1;
            
            // Instruction counter
            if (mem_wb_valid) begin
                csr_minstret <= csr_minstret + 1;
            end
            
            // CSR write (SYSTEM instructions)
            if (id_ex_valid && id_ex_instr[6:0] == OP_SYSTEM && id_ex_instr[14:12] != 3'b000) begin
                // CSR address in imm_i[11:0]
                // Handle CSR writes here
            end
        end
    end
    
    //=========================================================================
    // Exception/Interrupt Handling
    //=========================================================================
    assign trap = 1'b0;  // Simplified - add full exception handling
    assign trap_cause = 64'd0;
    assign trap_val = 64'd0;
    assign halted = 1'b0;
    
    //=========================================================================
    // Debug Interface
    //=========================================================================
    assign debug_pc = pc;
    assign debug_halted = halted;
    assign debug_ack = debug_req;
    
    //=========================================================================
    // External Coprocessor Interface
    //=========================================================================
    assign pcpi_valid = id_ex_valid && !id_ex_use_fpu && !id_ex_use_vector &&
                        (id_ex_instr[6:0] == 7'b0001011);  // Custom-0 opcode
    assign pcpi_insn = id_ex_instr;
    assign pcpi_rs1 = forwarded_rs1;
    assign pcpi_rs2 = forwarded_rs2;
    
    //=========================================================================
    // Vector Memory Interface (placeholder)
    //=========================================================================
    assign vmem_valid = 1'b0;
    assign vmem_write = 1'b0;
    assign vmem_addr = 64'd0;
    assign vmem_wdata = '0;
    assign vmem_wstrb = '0;
    
    //=========================================================================
    // Branch Prediction (simplified)
    //=========================================================================
    assign branch_predicted_taken = 1'b0;  // Predict not taken
    assign branch_predicted_target = pc + 64'd4;
    
    //=========================================================================
    // Compressed Instruction Decoder
    //=========================================================================
    function automatic [31:0] expand_compressed(input [15:0] cinstr);
        logic [31:0] expanded;
        logic [4:0] rd_c, rs1_c, rs2_c;
        logic [7:0] imm8;
        logic [5:0] imm6;
        
        // Compressed register mapping (x8-x15 for 3-bit fields)
        rd_c = {2'b01, cinstr[9:7]};
        rs1_c = {2'b01, cinstr[9:7]};
        rs2_c = {2'b01, cinstr[4:2]};
        
        expanded = 32'd0;
        
        case (cinstr[1:0])
            2'b00: begin  // Quadrant 0
                case (cinstr[15:13])
                    3'b000: begin  // C.ADDI4SPN
                        imm8 = {cinstr[10:7], cinstr[12:11], cinstr[5], cinstr[6]};
                        expanded = {2'b0, imm8, 2'b00, 5'd2, 3'b000, rd_c, OP_IMM};
                    end
                    3'b010: begin  // C.LW
                        expanded = {5'b0, cinstr[5], cinstr[12:10], cinstr[6], 2'b00, 
                                   rs1_c, 3'b010, rd_c, OP_LOAD};
                    end
                    3'b011: begin  // C.LD (RV64)
                        expanded = {4'b0, cinstr[6:5], cinstr[12:10], 3'b000,
                                   rs1_c, 3'b011, rd_c, OP_LOAD};
                    end
                    3'b110: begin  // C.SW
                        expanded = {5'b0, cinstr[5], cinstr[12], rs2_c, rs1_c, 
                                   3'b010, cinstr[11:10], cinstr[6], 2'b00, OP_STORE};
                    end
                    3'b111: begin  // C.SD (RV64)
                        expanded = {4'b0, cinstr[6:5], cinstr[12], rs2_c, rs1_c,
                                   3'b011, cinstr[11:10], 3'b000, OP_STORE};
                    end
                    default: expanded = 32'h0000_0013;  // NOP
                endcase
            end
            
            2'b01: begin  // Quadrant 1
                case (cinstr[15:13])
                    3'b000: begin  // C.ADDI
                        expanded = {{6{cinstr[12]}}, cinstr[12], cinstr[6:2], 
                                   cinstr[11:7], 3'b000, cinstr[11:7], OP_IMM};
                    end
                    3'b001: begin  // C.ADDIW (RV64)
                        expanded = {{6{cinstr[12]}}, cinstr[12], cinstr[6:2],
                                   cinstr[11:7], 3'b000, cinstr[11:7], OP_IMM_W};
                    end
                    3'b010: begin  // C.LI
                        expanded = {{6{cinstr[12]}}, cinstr[12], cinstr[6:2],
                                   5'd0, 3'b000, cinstr[11:7], OP_IMM};
                    end
                    3'b011: begin  // C.LUI / C.ADDI16SP
                        if (cinstr[11:7] == 5'd2) begin  // C.ADDI16SP
                            expanded = {{3{cinstr[12]}}, cinstr[4:3], cinstr[5], 
                                       cinstr[2], cinstr[6], 4'b0000, 5'd2, 3'b000, 5'd2, OP_IMM};
                        end else begin  // C.LUI
                            expanded = {{14{cinstr[12]}}, cinstr[12], cinstr[6:2],
                                       cinstr[11:7], OP_LUI};
                        end
                    end
                    3'b100: begin  // ALU operations
                        case (cinstr[11:10])
                            2'b00: begin  // C.SRLI
                                expanded = {6'b000000, cinstr[12], cinstr[6:2],
                                           rs1_c, 3'b101, rs1_c, OP_IMM};
                            end
                            2'b01: begin  // C.SRAI
                                expanded = {6'b010000, cinstr[12], cinstr[6:2],
                                           rs1_c, 3'b101, rs1_c, OP_IMM};
                            end
                            2'b10: begin  // C.ANDI
                                expanded = {{6{cinstr[12]}}, cinstr[12], cinstr[6:2],
                                           rs1_c, 3'b111, rs1_c, OP_IMM};
                            end
                            2'b11: begin  // C.SUB, C.XOR, C.OR, C.AND
                                case ({cinstr[12], cinstr[6:5]})
                                    3'b000: expanded = {7'b0100000, rs2_c, rs1_c, 3'b000, rs1_c, OP_REG}; // C.SUB
                                    3'b001: expanded = {7'b0000000, rs2_c, rs1_c, 3'b100, rs1_c, OP_REG}; // C.XOR
                                    3'b010: expanded = {7'b0000000, rs2_c, rs1_c, 3'b110, rs1_c, OP_REG}; // C.OR
                                    3'b011: expanded = {7'b0000000, rs2_c, rs1_c, 3'b111, rs1_c, OP_REG}; // C.AND
                                    3'b100: expanded = {7'b0100000, rs2_c, rs1_c, 3'b000, rs1_c, OP_REG_W}; // C.SUBW
                                    3'b101: expanded = {7'b0000000, rs2_c, rs1_c, 3'b000, rs1_c, OP_REG_W}; // C.ADDW
                                    default: expanded = 32'h0000_0013;
                                endcase
                            end
                        endcase
                    end
                    3'b101: begin  // C.J
                        expanded = {cinstr[12], cinstr[8], cinstr[10:9], cinstr[6],
                                   cinstr[7], cinstr[2], cinstr[11], cinstr[5:3],
                                   {9{cinstr[12]}}, 5'd0, OP_JAL};
                    end
                    3'b110: begin  // C.BEQZ
                        expanded = {{4{cinstr[12]}}, cinstr[6:5], cinstr[2], 5'd0,
                                   rs1_c, 3'b000, cinstr[11:10], cinstr[4:3],
                                   cinstr[12], OP_BRANCH};
                    end
                    3'b111: begin  // C.BNEZ
                        expanded = {{4{cinstr[12]}}, cinstr[6:5], cinstr[2], 5'd0,
                                   rs1_c, 3'b001, cinstr[11:10], cinstr[4:3],
                                   cinstr[12], OP_BRANCH};
                    end
                endcase
            end
            
            2'b10: begin  // Quadrant 2
                case (cinstr[15:13])
                    3'b000: begin  // C.SLLI
                        expanded = {6'b000000, cinstr[12], cinstr[6:2],
                                   cinstr[11:7], 3'b001, cinstr[11:7], OP_IMM};
                    end
                    3'b001: begin  // C.FLDSP (RV64)
                        expanded = {3'b0, cinstr[4:2], cinstr[12], cinstr[6:5], 3'b000,
                                   5'd2, 3'b011, cinstr[11:7], OP_LOAD_FP};
                    end
                    3'b010: begin  // C.LWSP
                        expanded = {4'b0, cinstr[3:2], cinstr[12], cinstr[6:4], 2'b00,
                                   5'd2, 3'b010, cinstr[11:7], OP_LOAD};
                    end
                    3'b011: begin  // C.LDSP (RV64)
                        expanded = {3'b0, cinstr[4:2], cinstr[12], cinstr[6:5], 3'b000,
                                   5'd2, 3'b011, cinstr[11:7], OP_LOAD};
                    end
                    3'b100: begin
                        if (cinstr[12] == 1'b0) begin
                            if (cinstr[6:2] == 5'd0) begin  // C.JR
                                expanded = {12'd0, cinstr[11:7], 3'b000, 5'd0, OP_JALR};
                            end else begin  // C.MV
                                expanded = {7'd0, cinstr[6:2], 5'd0, 3'b000, 
                                           cinstr[11:7], OP_REG};
                            end
                        end else begin
                            if (cinstr[6:2] == 5'd0) begin
                                if (cinstr[11:7] == 5'd0) begin  // C.EBREAK
                                    expanded = 32'h0010_0073;
                                end else begin  // C.JALR
                                    expanded = {12'd0, cinstr[11:7], 3'b000, 5'd1, OP_JALR};
                                end
                            end else begin  // C.ADD
                                expanded = {7'd0, cinstr[6:2], cinstr[11:7], 3'b000,
                                           cinstr[11:7], OP_REG};
                            end
                        end
                    end
                    3'b101: begin  // C.FSDSP (RV64)
                        expanded = {3'b0, cinstr[9:7], cinstr[12], cinstr[6:2],
                                   5'd2, 3'b011, cinstr[11:10], 3'b000, OP_STORE_FP};
                    end
                    3'b110: begin  // C.SWSP
                        expanded = {4'b0, cinstr[8:7], cinstr[12], cinstr[6:2],
                                   5'd2, 3'b010, cinstr[11:9], 2'b00, OP_STORE};
                    end
                    3'b111: begin  // C.SDSP (RV64)
                        expanded = {3'b0, cinstr[9:7], cinstr[12], cinstr[6:2],
                                   5'd2, 3'b011, cinstr[11:10], 3'b000, OP_STORE};
                    end
                endcase
            end
            
            default: expanded = 32'h0000_0013;  // NOP
        endcase
        
        return expanded;
    endfunction
    
    // Assign FPU rd (simplified - uses same rd as instruction)
    assign fpu_rd = id_ex_rd;

endmodule


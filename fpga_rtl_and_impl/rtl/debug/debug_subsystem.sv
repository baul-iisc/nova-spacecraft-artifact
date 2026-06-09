//============================================================================
// PhD Research: Complete Debug Subsystem
// Author: Chandraboul
// Target: Octa-Core Space Processor Debug System
//
// Description:
//   Complete debug subsystem integrating JTAG DTM, Debug Module,
//   trace buffer, and performance counters for the space processor.
//
// Features:
//   - JTAG Debug Access (IEEE 1149.1)
//   - RISC-V Debug Spec 0.13 compliant
//   - Hardware breakpoints (4 per hart)
//   - Trace buffer (instruction/data)
//   - Performance counters
//   - DLS error capture
//   - System bus access
//   - Multi-hart debugging
//
// Debug Memory Map:
//   0x000 - 0x0FF : Debug Module registers
//   0x100 - 0x1FF : Trace buffer
//   0x200 - 0x2FF : Performance counters
//   0x300 - 0x3FF : Error logs
//============================================================================

`timescale 1ns / 1ps

module debug_subsystem #(
    parameter NUM_HARTS       = 8,
    parameter NUM_BREAKPOINTS = 4,
    parameter TRACE_DEPTH     = 256,
    parameter NUM_PERF_CTRS   = 8
)(
    // System Interface
    input  logic            clk,
    input  logic            rst_n,
    
    // JTAG Interface
    input  logic            tck,
    input  logic            tms,
    input  logic            tdi,
    output logic            tdo,
    input  logic            trst_n,
    
    // Hart Interfaces
    output logic [NUM_HARTS-1:0]    halt_req,
    output logic [NUM_HARTS-1:0]    resume_req,
    input  logic [NUM_HARTS-1:0]    halted,
    input  logic [NUM_HARTS-1:0]    running,
    input  logic [NUM_HARTS-1:0]    unavailable,
    
    // Trace Interface (from cores)
    input  logic [31:0]             trace_pc     [NUM_HARTS-1:0],
    input  logic [31:0]             trace_instr  [NUM_HARTS-1:0],
    input  logic [NUM_HARTS-1:0]    trace_valid,
    
    // Performance Events (from cores)
    input  logic [NUM_HARTS-1:0]    event_instr_commit,
    input  logic [NUM_HARTS-1:0]    event_branch,
    input  logic [NUM_HARTS-1:0]    event_branch_miss,
    input  logic [NUM_HARTS-1:0]    event_load,
    input  logic [NUM_HARTS-1:0]    event_store,
    input  logic [NUM_HARTS-1:0]    event_exception,
    
    // DLS Error Interface
    input  logic [3:0]              dls_errors,
    input  logic [31:0]             dls_error_pc   [3:0],
    input  logic [31:0]             dls_error_data [3:0],
    
    // Breakpoint Interface
    output logic [31:0]             bp_addr [NUM_BREAKPOINTS-1:0],
    output logic [NUM_BREAKPOINTS-1:0] bp_enable,
    input  logic [NUM_BREAKPOINTS-1:0] bp_hit,
    
    // System Bus Interface
    output logic            sb_req,
    output logic            sb_write,
    output logic [31:0]     sb_addr,
    output logic [31:0]     sb_wdata,
    input  logic [31:0]     sb_rdata,
    input  logic            sb_ready,
    input  logic            sb_error,
    
    // Status
    output logic            debug_active,
    output logic            ndmreset,
    output logic [NUM_HARTS-1:0] debug_irq
);

    //------------------------------------------------------------------------
    // DMI Signals (between DTM and DM)
    //------------------------------------------------------------------------
    logic        dmi_req_valid;
    logic        dmi_req_ready;
    logic [6:0]  dmi_req_addr;
    logic [31:0] dmi_req_data;
    logic [1:0]  dmi_req_op;
    
    logic        dmi_rsp_valid;
    logic        dmi_rsp_ready;
    logic [31:0] dmi_rsp_data;
    logic [1:0]  dmi_rsp_op;
    
    //------------------------------------------------------------------------
    // JTAG Debug Transport Module
    //------------------------------------------------------------------------
    jtag_dtm #(
        .IDCODE_VALUE   (32'h10e31913),
        .IR_LENGTH      (5),
        .DMI_ADDR_BITS  (7),
        .DMI_DATA_BITS  (32)
    ) dtm (
        // JTAG
        .tck            (tck),
        .tms            (tms),
        .tdi            (tdi),
        .tdo            (tdo),
        .trst_n         (trst_n),
        
        // System clock
        .sys_clk        (clk),
        .sys_rst_n      (rst_n),
        
        // DMI
        .dmi_req_valid  (dmi_req_valid),
        .dmi_req_ready  (dmi_req_ready),
        .dmi_req_addr   (dmi_req_addr),
        .dmi_req_data   (dmi_req_data),
        .dmi_req_op     (dmi_req_op),
        .dmi_rsp_valid  (dmi_rsp_valid),
        .dmi_rsp_ready  (dmi_rsp_ready),
        .dmi_rsp_data   (dmi_rsp_data),
        .dmi_rsp_op     (dmi_rsp_op)
    );
    
    //------------------------------------------------------------------------
    // Debug Module
    //------------------------------------------------------------------------
    logic        dm_reg_req;
    logic        dm_reg_write;
    logic [15:0] dm_reg_addr;
    logic [31:0] dm_reg_wdata;
    logic [31:0] dm_reg_rdata;
    logic        dm_reg_ready;
    logic [2:0]  dm_reg_hartsel;
    
    debug_module #(
        .NUM_HARTS      (NUM_HARTS),
        .NUM_BREAKPOINTS(NUM_BREAKPOINTS),
        .PROGBUF_SIZE   (16),
        .DATA_COUNT     (2),
        .DMI_ADDR_BITS  (7),
        .DMI_DATA_BITS  (32)
    ) dm (
        .clk            (clk),
        .rst_n          (rst_n),
        
        // DMI
        .dmi_req_valid  (dmi_req_valid),
        .dmi_req_ready  (dmi_req_ready),
        .dmi_req_addr   (dmi_req_addr),
        .dmi_req_data   (dmi_req_data),
        .dmi_req_op     (dmi_req_op),
        .dmi_rsp_valid  (dmi_rsp_valid),
        .dmi_rsp_ready  (dmi_rsp_ready),
        .dmi_rsp_data   (dmi_rsp_data),
        .dmi_rsp_op     (dmi_rsp_op),
        
        // Hart control
        .halt_req       (halt_req),
        .resume_req     (resume_req),
        .halted         (halted),
        .running        (running),
        .unavailable    (unavailable),
        
        // Register access
        .reg_req        (dm_reg_req),
        .reg_write      (dm_reg_write),
        .reg_addr       (dm_reg_addr),
        .reg_wdata      (dm_reg_wdata),
        .reg_rdata      (dm_reg_rdata),
        .reg_ready      (dm_reg_ready),
        .reg_hartsel    (dm_reg_hartsel),
        
        // Breakpoints
        .bp_addr        (bp_addr),
        .bp_enable      (bp_enable),
        .bp_hit         (bp_hit),
        
        // System bus
        .sb_req         (sb_req),
        .sb_write       (sb_write),
        .sb_addr        (sb_addr),
        .sb_wdata       (sb_wdata),
        .sb_rdata       (sb_rdata),
        .sb_ready       (sb_ready),
        .sb_error       (sb_error),
        
        // Status
        .dm_active      (debug_active),
        .ndmreset_req   (ndmreset),
        .debug_irq      (debug_irq)
    );
    
    //------------------------------------------------------------------------
    // Trace Buffer - SYNCHRONOUS reset, NO reset for buffer RAM
    //------------------------------------------------------------------------
    typedef struct packed {
        logic [31:0] pc;
        logic [31:0] instr;
        logic [2:0]  hart;
        logic [28:0] timestamp;
    } trace_entry_t;
    
    // Split trace buffer for BRAM inference
    (* ram_style = "block" *) logic [31:0] trace_buffer_pc [TRACE_DEPTH-1:0];
    (* ram_style = "block" *) logic [31:0] trace_buffer_instr [TRACE_DEPTH-1:0];
    (* ram_style = "block" *) logic [31:0] trace_buffer_meta [TRACE_DEPTH-1:0]; // hart + timestamp
    
    logic [$clog2(TRACE_DEPTH)-1:0] trace_wr_ptr;
    logic [31:0] trace_timestamp;
    logic trace_enable;
    logic trace_wrap;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            trace_wr_ptr <= '0;
            trace_timestamp <= '0;
            trace_wrap <= 1'b0;
        end else begin
            trace_timestamp <= trace_timestamp + 1;
            
            if (trace_enable) begin
                for (int h = 0; h < NUM_HARTS; h++) begin
                    if (trace_valid[h]) begin
                        trace_buffer_pc[trace_wr_ptr] <= trace_pc[h];
                        trace_buffer_instr[trace_wr_ptr] <= trace_instr[h];
                        trace_buffer_meta[trace_wr_ptr] <= {h[2:0], trace_timestamp[28:0]};
                        
                        if (trace_wr_ptr == TRACE_DEPTH-1) begin
                            trace_wr_ptr <= '0;
                            trace_wrap <= 1'b1;
                        end else begin
                            trace_wr_ptr <= trace_wr_ptr + 1;
                        end
                    end
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Performance Counters - SYNCHRONOUS reset
    //------------------------------------------------------------------------
    logic [63:0] perf_counters [NUM_PERF_CTRS-1:0];
    logic [NUM_PERF_CTRS-1:0] perf_enable;
    
    // Counter 0: Total cycles
    // Counter 1: Instructions committed
    // Counter 2: Branches
    // Counter 3: Branch misses
    // Counter 4: Loads
    // Counter 5: Stores
    // Counter 6: Exceptions
    // Counter 7: Reserved
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < NUM_PERF_CTRS; i++)
                perf_counters[i] <= '0;
        end else begin
            // Cycles
            if (perf_enable[0])
                perf_counters[0] <= perf_counters[0] + 1;
            
            // Instructions
            if (perf_enable[1])
                perf_counters[1] <= perf_counters[1] + $countones(event_instr_commit);
            
            // Branches
            if (perf_enable[2])
                perf_counters[2] <= perf_counters[2] + $countones(event_branch);
            
            // Branch misses
            if (perf_enable[3])
                perf_counters[3] <= perf_counters[3] + $countones(event_branch_miss);
            
            // Loads
            if (perf_enable[4])
                perf_counters[4] <= perf_counters[4] + $countones(event_load);
            
            // Stores
            if (perf_enable[5])
                perf_counters[5] <= perf_counters[5] + $countones(event_store);
            
            // Exceptions
            if (perf_enable[6])
                perf_counters[6] <= perf_counters[6] + $countones(event_exception);
        end
    end
    
    //------------------------------------------------------------------------
    // DLS Error Capture - SYNCHRONOUS reset, NO reset for log RAM
    //------------------------------------------------------------------------
    typedef struct packed {
        logic [31:0] pc;
        logic [31:0] data;
        logic [1:0]  pair;
        logic [29:0] timestamp;
    } error_entry_t;
    
    // Split error log for better inference
    logic [31:0] error_log_pc [15:0];
    logic [31:0] error_log_data [15:0];
    logic [31:0] error_log_meta [15:0]; // pair + timestamp
    logic [3:0] error_wr_ptr;
    logic [31:0] error_timestamp;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            error_wr_ptr <= '0;
            error_timestamp <= '0;
        end else begin
            error_timestamp <= error_timestamp + 1;
            
            for (int p = 0; p < 4; p++) begin
                if (dls_errors[p]) begin
                    error_log_pc[error_wr_ptr] <= dls_error_pc[p];
                    error_log_data[error_wr_ptr] <= dls_error_data[p];
                    error_log_meta[error_wr_ptr] <= {p[1:0], error_timestamp[29:0]};
                    error_wr_ptr <= error_wr_ptr + 1;
                end
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Register Access (placeholder)
    //------------------------------------------------------------------------
    assign dm_reg_rdata = 32'h0;
    assign dm_reg_ready = 1'b1;
    assign trace_enable = debug_active;
    assign perf_enable = 8'hFF;  // Enable all by default

endmodule




//============================================================================
// PhD Research: RISC-V Debug Module
// Author: Chandraboul
// Target: RISC-V Debug Specification 0.13 Compatible
//
// Description:
//   Debug Module for multi-hart RISC-V systems with hardware breakpoints,
//   single-stepping, and system bus access.
//
// Features:
//   - Multi-hart support (up to 8 cores)
//   - Hardware breakpoints (4 per hart)
//   - Single-step support
//   - Abstract commands (access register, quick access)
//   - Program buffer (16 bytes)
//   - System bus access
//   - Core halt/resume control
//   - DLS pair debugging
//
// Debug Memory Map (0x0 - 0x3FF):
//   0x04: data0        - Abstract Data 0
//   0x05: data1        - Abstract Data 1
//   ...
//   0x10: dmcontrol    - Debug Module Control
//   0x11: dmstatus     - Debug Module Status
//   0x12: hartinfo     - Hart Info
//   0x16: abstractcs   - Abstract Control and Status
//   0x17: command      - Abstract Command
//   0x18: abstractauto - Abstract Command Autoexec
//   0x20: progbuf0     - Program Buffer 0
//   ...
//   0x38: sbcs         - System Bus Access Control
//   0x39: sbaddress0   - System Bus Address [31:0]
//   0x3C: sbdata0      - System Bus Data [31:0]
//============================================================================

`timescale 1ns / 1ps

module debug_module #(
    parameter NUM_HARTS      = 8,
    parameter NUM_BREAKPOINTS= 4,
    parameter PROGBUF_SIZE   = 16,
    parameter DATA_COUNT     = 2,
    parameter DMI_ADDR_BITS  = 7,
    parameter DMI_DATA_BITS  = 32
)(
    input  logic            clk,
    input  logic            rst_n,
    
    // DMI Interface (from DTM)
    input  logic                       dmi_req_valid,
    output logic                       dmi_req_ready,
    input  logic [DMI_ADDR_BITS-1:0]   dmi_req_addr,
    input  logic [DMI_DATA_BITS-1:0]   dmi_req_data,
    input  logic [1:0]                 dmi_req_op,
    
    output logic                       dmi_rsp_valid,
    input  logic                       dmi_rsp_ready,
    output logic [DMI_DATA_BITS-1:0]   dmi_rsp_data,
    output logic [1:0]                 dmi_rsp_op,
    
    // Hart Interfaces (to cores)
    output logic [NUM_HARTS-1:0]       halt_req,
    output logic [NUM_HARTS-1:0]       resume_req,
    input  logic [NUM_HARTS-1:0]       halted,
    input  logic [NUM_HARTS-1:0]       running,
    input  logic [NUM_HARTS-1:0]       unavailable,
    
    // Register Access Interface
    output logic                       reg_req,
    output logic                       reg_write,
    output logic [15:0]                reg_addr,
    output logic [31:0]                reg_wdata,
    input  logic [31:0]                reg_rdata,
    input  logic                       reg_ready,
    output logic [2:0]                 reg_hartsel,
    
    // Breakpoint Interface
    output logic [31:0]                bp_addr [NUM_BREAKPOINTS-1:0],
    output logic [NUM_BREAKPOINTS-1:0] bp_enable,
    input  logic [NUM_BREAKPOINTS-1:0] bp_hit,
    
    // System Bus Interface
    output logic                       sb_req,
    output logic                       sb_write,
    output logic [31:0]                sb_addr,
    output logic [31:0]                sb_wdata,
    input  logic [31:0]                sb_rdata,
    input  logic                       sb_ready,
    input  logic                       sb_error,
    
    // Status
    output logic                       dm_active,
    output logic                       ndmreset_req,
    output logic [NUM_HARTS-1:0]       debug_irq
);

    //------------------------------------------------------------------------
    // Debug Module Register Addresses
    //------------------------------------------------------------------------
    localparam [6:0] ADDR_DATA0       = 7'h04;
    localparam [6:0] ADDR_DATA1       = 7'h05;
    localparam [6:0] ADDR_DMCONTROL   = 7'h10;
    localparam [6:0] ADDR_DMSTATUS    = 7'h11;
    localparam [6:0] ADDR_HARTINFO    = 7'h12;
    localparam [6:0] ADDR_HALTSUM0    = 7'h13;
    localparam [6:0] ADDR_HAWINDOWSEL = 7'h14;
    localparam [6:0] ADDR_HAWINDOW    = 7'h15;
    localparam [6:0] ADDR_ABSTRACTCS  = 7'h16;
    localparam [6:0] ADDR_COMMAND     = 7'h17;
    localparam [6:0] ADDR_ABSTRACTAUTO= 7'h18;
    localparam [6:0] ADDR_PROGBUF0    = 7'h20;
    localparam [6:0] ADDR_SBCS        = 7'h38;
    localparam [6:0] ADDR_SBADDRESS0  = 7'h39;
    localparam [6:0] ADDR_SBDATA0     = 7'h3C;
    
    //------------------------------------------------------------------------
    // Internal Registers
    //------------------------------------------------------------------------
    // Abstract data registers
    logic [31:0] data_regs [DATA_COUNT-1:0];
    
    // Program buffer
    logic [31:0] progbuf [PROGBUF_SIZE/4-1:0];
    
    // Control registers
    logic [31:0] dmcontrol;
    logic [31:0] abstractcs;
    logic [31:0] command_reg;
    logic [31:0] abstractauto;
    
    // System bus control
    logic [31:0] sbcs;
    logic [31:0] sbaddress0;
    logic [31:0] sbdata0;
    
    //------------------------------------------------------------------------
    // DMCONTROL Fields
    //------------------------------------------------------------------------
    logic        dmactive;
    logic        ndmreset;
    logic        clrresethaltreq;
    logic        setresethaltreq;
    logic        hartselhi;
    logic        hartsello;
    logic        hasel;
    logic        ackhavereset;
    logic        hartreset;
    logic        resumereq;
    logic        haltreq;
    
    logic [9:0]  hartsel;
    
    always_comb begin
        dmactive        = dmcontrol[0];
        ndmreset        = dmcontrol[1];
        clrresethaltreq = dmcontrol[2];
        setresethaltreq = dmcontrol[3];
        hartselhi       = dmcontrol[6];
        hartsello       = dmcontrol[16];
        hasel           = dmcontrol[26];
        hartreset       = dmcontrol[29];
        resumereq       = dmcontrol[30];
        haltreq         = dmcontrol[31];
        
        hartsel = {dmcontrol[25:16]};
    end
    
    //------------------------------------------------------------------------
    // DMSTATUS (Read-Only)
    //------------------------------------------------------------------------
    logic [31:0] dmstatus;
    logic        allhalted, anyhalted;
    logic        allrunning, anyrunning;
    logic        allunavail, anyunavail;
    logic        allresumeack, anyresumeack;
    logic        allhavereset, anyhavereset;
    
    always_comb begin
        allhalted    = &halted;
        anyhalted    = |halted;
        allrunning   = &running;
        anyrunning   = |running;
        allunavail   = &unavailable;
        anyunavail   = |unavailable;
        allresumeack = 1'b0;
        anyresumeack = 1'b0;
        allhavereset = 1'b0;
        anyhavereset = 1'b0;
        
        dmstatus = {
            9'b0,                   // [31:23] Reserved
            1'b0,                   // [22] impebreak
            1'b0,                   // [21] Reserved
            1'b0,                   // [20] allhavereset
            1'b0,                   // [19] anyhavereset
            allresumeack,           // [18] allresumeack
            anyresumeack,           // [17] anyresumeack
            1'b0,                   // [16] allnonexistent
            1'b0,                   // [15] anynonexistent
            allunavail,             // [14] allunavail
            anyunavail,             // [13] anyunavail
            allrunning,             // [12] allrunning
            anyrunning,             // [11] anyrunning
            allhalted,              // [10] allhalted
            anyhalted,              // [9]  anyhalted
            1'b1,                   // [8]  authenticated
            1'b0,                   // [7]  authbusy
            1'b1,                   // [6]  hasresethaltreq
            1'b0,                   // [5]  confstrptrvalid
            4'h2                    // [3:0] version (0.13)
        };
    end
    
    //------------------------------------------------------------------------
    // ABSTRACTCS Fields
    //------------------------------------------------------------------------
    logic [2:0]  cmderr;
    logic        busy;
    
    always_comb begin
        abstractcs = {
            3'b0,                   // [31:29] Reserved
            5'(PROGBUF_SIZE/4),     // [28:24] progbufsize
            11'b0,                  // [23:13] Reserved
            busy,                   // [12] busy
            1'b0,                   // [11] Reserved
            cmderr,                 // [10:8] cmderr
            4'b0,                   // [7:4] Reserved
            4'(DATA_COUNT)          // [3:0] datacount
        };
    end
    
    //------------------------------------------------------------------------
    // SBCS Fields (System Bus Control and Status)
    //------------------------------------------------------------------------
    logic        sbbusy;
    logic        sberror;
    logic [2:0]  sbaccess;
    logic        sbreadonaddr;
    logic        sbreadondata;
    
    always_comb begin
        sbcs = {
            3'h1,                   // [31:29] sbversion
            6'b0,                   // [28:23] Reserved
            1'b0,                   // [22] sbbusyerror
            sbbusy,                 // [21] sbbusy
            sbreadonaddr,           // [20] sbreadonaddr
            sbaccess,               // [19:17] sbaccess
            1'b0,                   // [16] sbautoincrement
            sbreadondata,           // [15] sbreadondata
            sberror ? 3'h1 : 3'h0,  // [14:12] sberror
            7'h20,                  // [11:5] sbasize (32-bit)
            1'b1,                   // [4] sbaccess128
            1'b1,                   // [3] sbaccess64
            1'b1,                   // [2] sbaccess32
            1'b1,                   // [1] sbaccess16
            1'b1                    // [0] sbaccess8
        };
    end
    
    //------------------------------------------------------------------------
    // DMI Transaction Handling
    //------------------------------------------------------------------------
    typedef enum logic [2:0] {
        DM_IDLE,
        DM_READ,
        DM_WRITE,
        DM_EXEC_CMD,
        DM_SBUS_READ,
        DM_SBUS_WRITE,
        DM_RESPOND
    } dm_state_t;
    
    dm_state_t state;
    
    logic [6:0] req_addr;
    logic [31:0] req_data;
    logic [1:0] req_op;
    logic [31:0] rsp_data;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= DM_IDLE;
            dmi_req_ready <= 1'b1;
            dmi_rsp_valid <= 1'b0;
            dmi_rsp_op <= 2'b00;
            
            // Reset registers
            dmcontrol <= '0;
            for (int i = 0; i < DATA_COUNT; i++)
                data_regs[i] <= '0;
            for (int i = 0; i < PROGBUF_SIZE/4; i++)
                progbuf[i] <= 32'h00100073;  // EBREAK
            
            command_reg <= '0;
            abstractauto <= '0;
            sbaddress0 <= '0;
            sbdata0 <= '0;
            sbreadonaddr <= 1'b0;
            sbreadondata <= 1'b0;
            sbaccess <= 3'b010;  // 32-bit
            
            cmderr <= '0;
            busy <= 1'b0;
            sbbusy <= 1'b0;
            sberror <= 1'b0;
            
        end else begin
            dmi_rsp_valid <= 1'b0;
            
            case (state)
                DM_IDLE: begin
                    dmi_req_ready <= 1'b1;
                    
                    if (dmi_req_valid && dmi_req_ready) begin
                        req_addr <= dmi_req_addr[6:0];
                        req_data <= dmi_req_data;
                        req_op <= dmi_req_op;
                        dmi_req_ready <= 1'b0;
                        
                        if (dmi_req_op == 2'b01)      // Read
                            state <= DM_READ;
                        else if (dmi_req_op == 2'b10) // Write
                            state <= DM_WRITE;
                        else
                            state <= DM_RESPOND;
                    end
                end
                
                DM_READ: begin
                    case (req_addr)
                        ADDR_DATA0:       rsp_data <= data_regs[0];
                        ADDR_DATA1:       rsp_data <= data_regs[1];
                        ADDR_DMCONTROL:   rsp_data <= dmcontrol;
                        ADDR_DMSTATUS:    rsp_data <= dmstatus;
                        ADDR_HARTINFO:    rsp_data <= 32'h00000000;
                        ADDR_HALTSUM0:    rsp_data <= {24'b0, halted};
                        ADDR_ABSTRACTCS:  rsp_data <= abstractcs;
                        ADDR_COMMAND:     rsp_data <= command_reg;
                        ADDR_ABSTRACTAUTO:rsp_data <= abstractauto;
                        ADDR_SBCS:        rsp_data <= sbcs;
                        ADDR_SBADDRESS0:  rsp_data <= sbaddress0;
                        ADDR_SBDATA0: begin
                            rsp_data <= sbdata0;
                            if (sbreadondata && !sbbusy) begin
                                state <= DM_SBUS_READ;
                            end
                        end
                        default: begin
                            if (req_addr >= ADDR_PROGBUF0 && 
                                req_addr < ADDR_PROGBUF0 + PROGBUF_SIZE/4)
                                rsp_data <= progbuf[req_addr - ADDR_PROGBUF0];
                            else
                                rsp_data <= '0;
                        end
                    endcase
                    
                    if (state == DM_READ)
                        state <= DM_RESPOND;
                end
                
                DM_WRITE: begin
                    case (req_addr)
                        ADDR_DATA0:       data_regs[0] <= req_data;
                        ADDR_DATA1:       data_regs[1] <= req_data;
                        ADDR_DMCONTROL:   dmcontrol <= req_data;
                        ADDR_ABSTRACTCS: begin
                            // Write 1 to clear cmderr
                            if (req_data[10:8] != 0)
                                cmderr <= '0;
                        end
                        ADDR_COMMAND: begin
                            command_reg <= req_data;
                            if (!busy)
                                state <= DM_EXEC_CMD;
                        end
                        ADDR_ABSTRACTAUTO: abstractauto <= req_data;
                        ADDR_SBCS: begin
                            sbreadonaddr <= req_data[20];
                            sbreadondata <= req_data[15];
                            sbaccess <= req_data[19:17];
                            if (req_data[14:12] != 0)
                                sberror <= 1'b0;
                        end
                        ADDR_SBADDRESS0: begin
                            sbaddress0 <= req_data;
                            if (sbreadonaddr && !sbbusy)
                                state <= DM_SBUS_READ;
                        end
                        ADDR_SBDATA0: begin
                            sbdata0 <= req_data;
                            if (!sbbusy)
                                state <= DM_SBUS_WRITE;
                        end
                        default: begin
                            if (req_addr >= ADDR_PROGBUF0 && 
                                req_addr < ADDR_PROGBUF0 + PROGBUF_SIZE/4)
                                progbuf[req_addr - ADDR_PROGBUF0] <= req_data;
                        end
                    endcase
                    
                    if (state == DM_WRITE)
                        state <= DM_RESPOND;
                end
                
                DM_EXEC_CMD: begin
                    // Execute abstract command
                    busy <= 1'b1;
                    
                    // Simplified: just complete immediately
                    // Real implementation would handle register access
                    busy <= 1'b0;
                    state <= DM_RESPOND;
                end
                
                DM_SBUS_READ: begin
                    sbbusy <= 1'b1;
                    sb_req <= 1'b1;
                    sb_write <= 1'b0;
                    sb_addr <= sbaddress0;
                    
                    if (sb_ready) begin
                        sb_req <= 1'b0;
                        sbdata0 <= sb_rdata;
                        sberror <= sb_error;
                        sbbusy <= 1'b0;
                        state <= DM_RESPOND;
                    end
                end
                
                DM_SBUS_WRITE: begin
                    sbbusy <= 1'b1;
                    sb_req <= 1'b1;
                    sb_write <= 1'b1;
                    sb_addr <= sbaddress0;
                    sb_wdata <= sbdata0;
                    
                    if (sb_ready) begin
                        sb_req <= 1'b0;
                        sberror <= sb_error;
                        sbbusy <= 1'b0;
                        state <= DM_RESPOND;
                    end
                end
                
                DM_RESPOND: begin
                    dmi_rsp_valid <= 1'b1;
                    dmi_rsp_data <= rsp_data;
                    dmi_rsp_op <= 2'b00;  // OK
                    
                    if (dmi_rsp_ready) begin
                        dmi_rsp_valid <= 1'b0;
                        state <= DM_IDLE;
                    end
                end
                
                default: state <= DM_IDLE;
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // Hart Control
    //------------------------------------------------------------------------
    always_comb begin
        halt_req = '0;
        resume_req = '0;
        
        if (dmactive) begin
            if (hartsel < NUM_HARTS) begin
                halt_req[hartsel] = haltreq;
                resume_req[hartsel] = resumereq;
            end
        end
    end
    
    //------------------------------------------------------------------------
    // Debug IRQ Generation
    //------------------------------------------------------------------------
    always_comb begin
        debug_irq = '0;
        for (int i = 0; i < NUM_HARTS; i++) begin
            debug_irq[i] = bp_hit != '0;  // Any breakpoint hit
        end
    end
    
    //------------------------------------------------------------------------
    // Outputs
    //------------------------------------------------------------------------
    assign dm_active = dmactive;
    assign ndmreset_req = ndmreset;
    assign reg_hartsel = hartsel[2:0];
    
    // Register access (placeholder)
    assign reg_req = 1'b0;
    assign reg_write = 1'b0;
    assign reg_addr = '0;
    assign reg_wdata = '0;
    
    // Breakpoints (placeholder)
    always_comb begin
        for (int i = 0; i < NUM_BREAKPOINTS; i++) begin
            bp_addr[i] = data_regs[0];  // Simplified
            bp_enable[i] = 1'b0;
        end
    end

endmodule




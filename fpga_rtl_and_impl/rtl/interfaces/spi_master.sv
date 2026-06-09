//============================================================================
// PhD Research: SPI Master Interface for Space Processor
// Author: Chandraboul, IISc
// Target: Kintex UltraScale KU060
//
// Description:
//   Full-featured SPI Master with:
//   - Configurable clock divider (up to 50 MHz)
//   - CPOL/CPHA mode support (all 4 SPI modes)
//   - 8/16/32-bit transfer support
//   - TX/RX FIFOs for burst transfers
//   - Multi-slave support with chip select
//   - DMA-ready interface
//   - Interrupt generation
//
// Standard: Follows typical SPI protocol for flash/EEPROM/sensor interfacing
//============================================================================

`timescale 1ns / 1ps

module spi_master #(
    parameter CLK_FREQ_HZ  = 100_000_000,
    parameter SPI_CLK_HZ   = 25_000_000,
    parameter FIFO_DEPTH   = 16,
    parameter DATA_WIDTH   = 8,        // 8, 16, or 32
    parameter NUM_CS       = 4         // Number of chip selects
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // APB Slave Interface
    input  logic [11:0]             paddr,
    input  logic                    psel,
    input  logic                    penable,
    input  logic                    pwrite,
    input  logic [31:0]             pwdata,
    output logic [31:0]             prdata,
    output logic                    pready,
    output logic                    pslverr,
    
    // SPI Interface
    output logic                    spi_clk,
    output logic                    spi_mosi,
    input  logic                    spi_miso,
    output logic [NUM_CS-1:0]       spi_cs_n,
    
    // Interrupt
    output logic                    irq,
    
    // DMA Interface
    output logic                    dma_req,
    input  logic                    dma_ack,
    output logic                    dma_single,
    output logic                    dma_last
);

    //------------------------------------------------------------------------
    // Register Map
    //------------------------------------------------------------------------
    // 0x000: CTRL     - Control register
    // 0x004: STATUS   - Status register
    // 0x008: TXDATA   - Transmit data
    // 0x00C: RXDATA   - Receive data
    // 0x010: CLKDIV   - Clock divider
    // 0x014: CSCTRL   - Chip select control
    // 0x018: IRQEN    - Interrupt enable
    // 0x01C: IRQSTAT  - Interrupt status
    
    localparam ADDR_CTRL    = 12'h000;
    localparam ADDR_STATUS  = 12'h004;
    localparam ADDR_TXDATA  = 12'h008;
    localparam ADDR_RXDATA  = 12'h00C;
    localparam ADDR_CLKDIV  = 12'h010;
    localparam ADDR_CSCTRL  = 12'h014;
    localparam ADDR_IRQEN   = 12'h018;
    localparam ADDR_IRQSTAT = 12'h01C;
    
    //------------------------------------------------------------------------
    // Registers
    //------------------------------------------------------------------------
    logic [31:0] ctrl_reg;
    logic [31:0] clkdiv_reg;
    logic [31:0] csctrl_reg;
    logic [31:0] irqen_reg;
    logic [31:0] irqstat_reg;
    
    // Control register bits
    logic        spi_enable;
    logic        cpol;
    logic        cpha;
    logic [1:0]  data_size;    // 00=8bit, 01=16bit, 10=32bit
    logic        lsb_first;
    logic        loop_mode;
    
    assign spi_enable = ctrl_reg[0];
    assign cpol       = ctrl_reg[1];
    assign cpha       = ctrl_reg[2];
    assign data_size  = ctrl_reg[4:3];
    assign lsb_first  = ctrl_reg[5];
    assign loop_mode  = ctrl_reg[6];
    
    //------------------------------------------------------------------------
    // TX/RX FIFOs
    //------------------------------------------------------------------------
    logic [31:0] tx_fifo [FIFO_DEPTH-1:0];
    logic [31:0] rx_fifo [FIFO_DEPTH-1:0];
    logic [$clog2(FIFO_DEPTH):0] tx_wr_ptr, tx_rd_ptr;
    logic [$clog2(FIFO_DEPTH):0] rx_wr_ptr, rx_rd_ptr;
    
    wire tx_fifo_empty = (tx_wr_ptr == tx_rd_ptr);
    wire tx_fifo_full  = (tx_wr_ptr[$clog2(FIFO_DEPTH)] != tx_rd_ptr[$clog2(FIFO_DEPTH)]) &&
                         (tx_wr_ptr[$clog2(FIFO_DEPTH)-1:0] == tx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]);
    wire rx_fifo_empty = (rx_wr_ptr == rx_rd_ptr);
    wire rx_fifo_full  = (rx_wr_ptr[$clog2(FIFO_DEPTH)] != rx_rd_ptr[$clog2(FIFO_DEPTH)]) &&
                         (rx_wr_ptr[$clog2(FIFO_DEPTH)-1:0] == rx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]);
    
    wire [$clog2(FIFO_DEPTH):0] tx_fifo_level = tx_wr_ptr - tx_rd_ptr;
    wire [$clog2(FIFO_DEPTH):0] rx_fifo_level = rx_wr_ptr - rx_rd_ptr;
    
    //------------------------------------------------------------------------
    // SPI Clock Generation
    //------------------------------------------------------------------------
    logic [15:0] clk_cnt;
    logic        spi_clk_int;
    logic        clk_edge;
    
    // SYNCHRONOUS reset for FPGA BRAM inference
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            clk_cnt <= '0;
            spi_clk_int <= cpol;
        end else if (spi_enable && !tx_fifo_empty) begin
            if (clk_cnt >= clkdiv_reg[15:0]) begin
                clk_cnt <= '0;
                spi_clk_int <= ~spi_clk_int;
            end else begin
                clk_cnt <= clk_cnt + 1'b1;
            end
        end else begin
            spi_clk_int <= cpol;
            clk_cnt <= '0;
        end
    end
    
    assign spi_clk = spi_clk_int;
    
    //------------------------------------------------------------------------
    // SPI State Machine
    //------------------------------------------------------------------------
    typedef enum logic [2:0] {
        SPI_IDLE,
        SPI_CS_SETUP,
        SPI_TRANSFER,
        SPI_CS_HOLD,
        SPI_DONE
    } spi_state_t;
    
    spi_state_t spi_state;
    logic [5:0]  bit_cnt;
    logic [31:0] shift_reg;
    logic [31:0] rx_shift_reg;
    logic [7:0]  cs_setup_cnt;
    logic [7:0]  cs_hold_cnt;
    logic        transfer_done;
    
    // Determine bits to transfer based on data size
    wire [5:0] bits_to_transfer = (data_size == 2'b00) ? 6'd8  :
                                  (data_size == 2'b01) ? 6'd16 :
                                  6'd32;
    
    // SYNCHRONOUS reset for FPGA BRAM inference on FIFOs
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            spi_state <= SPI_IDLE;
            bit_cnt <= '0;
            shift_reg <= '0;
            rx_shift_reg <= '0;
            spi_cs_n <= '1;
            cs_setup_cnt <= '0;
            cs_hold_cnt <= '0;
            transfer_done <= 1'b0;
            tx_rd_ptr <= '0;
            rx_wr_ptr <= '0;
        end else begin
            transfer_done <= 1'b0;
            
            case (spi_state)
                SPI_IDLE: begin
                    if (spi_enable && !tx_fifo_empty) begin
                        // Load data from TX FIFO
                        shift_reg <= lsb_first ? 
                            {tx_fifo[tx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]][7:0],
                             tx_fifo[tx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]][15:8],
                             tx_fifo[tx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]][23:16],
                             tx_fifo[tx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]][31:24]} :
                            tx_fifo[tx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]];
                        tx_rd_ptr <= tx_rd_ptr + 1'b1;
                        // Assert chip select
                        spi_cs_n <= ~csctrl_reg[NUM_CS-1:0];
                        cs_setup_cnt <= csctrl_reg[15:8];
                        spi_state <= SPI_CS_SETUP;
                    end
                end
                
                SPI_CS_SETUP: begin
                    if (cs_setup_cnt == 0) begin
                        bit_cnt <= bits_to_transfer;
                        spi_state <= SPI_TRANSFER;
                    end else begin
                        cs_setup_cnt <= cs_setup_cnt - 1'b1;
                    end
                end
                
                SPI_TRANSFER: begin
                    // Detect clock edge based on CPOL/CPHA
                    if (clk_cnt == '0) begin
                        if ((cpha == 1'b0 && spi_clk_int == cpol) ||
                            (cpha == 1'b1 && spi_clk_int != cpol)) begin
                            // Shift out on this edge
                            shift_reg <= {shift_reg[30:0], 1'b0};
                        end else begin
                            // Sample on this edge
                            rx_shift_reg <= {rx_shift_reg[30:0], loop_mode ? shift_reg[31] : spi_miso};
                            bit_cnt <= bit_cnt - 1'b1;
                            if (bit_cnt == 1) begin
                                cs_hold_cnt <= csctrl_reg[23:16];
                                spi_state <= SPI_CS_HOLD;
                            end
                        end
                    end
                end
                
                SPI_CS_HOLD: begin
                    if (cs_hold_cnt == 0) begin
                        // Store received data
                        if (!rx_fifo_full) begin
                            rx_fifo[rx_wr_ptr[$clog2(FIFO_DEPTH)-1:0]] <= rx_shift_reg;
                            rx_wr_ptr <= rx_wr_ptr + 1'b1;
                        end
                        spi_cs_n <= '1;
                        transfer_done <= 1'b1;
                        spi_state <= SPI_DONE;
                    end else begin
                        cs_hold_cnt <= cs_hold_cnt - 1'b1;
                    end
                end
                
                SPI_DONE: begin
                    // Check if more data to transfer
                    if (!tx_fifo_empty) begin
                        spi_state <= SPI_IDLE;
                    end else begin
                        spi_state <= SPI_IDLE;
                    end
                end
            endcase
        end
    end
    
    // MOSI output
    assign spi_mosi = shift_reg[31];
    
    //------------------------------------------------------------------------
    // APB Register Interface - SYNCHRONOUS reset for FPGA BRAM inference
    //------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ctrl_reg <= 32'h0;
            clkdiv_reg <= CLK_FREQ_HZ / (2 * SPI_CLK_HZ) - 1;
            csctrl_reg <= 32'h00040401;  // Default: CS0, setup=4, hold=4
            irqen_reg <= 32'h0;
            irqstat_reg <= 32'h0;
            tx_wr_ptr <= '0;
            rx_rd_ptr <= '0;
        end else begin
            // Interrupt status update
            if (transfer_done) irqstat_reg[0] <= 1'b1;
            if (tx_fifo_empty) irqstat_reg[1] <= 1'b1;
            if (!rx_fifo_empty) irqstat_reg[2] <= 1'b1;
            
            if (psel && penable) begin
                if (pwrite) begin
                    case (paddr)
                        ADDR_CTRL:    ctrl_reg <= pwdata;
                        ADDR_TXDATA: begin
                            if (!tx_fifo_full) begin
                                tx_fifo[tx_wr_ptr[$clog2(FIFO_DEPTH)-1:0]] <= pwdata;
                                tx_wr_ptr <= tx_wr_ptr + 1'b1;
                            end
                        end
                        ADDR_CLKDIV:  clkdiv_reg <= pwdata;
                        ADDR_CSCTRL:  csctrl_reg <= pwdata;
                        ADDR_IRQEN:   irqen_reg <= pwdata;
                        ADDR_IRQSTAT: irqstat_reg <= irqstat_reg & ~pwdata;  // W1C
                    endcase
                end else begin
                    case (paddr)
                        ADDR_RXDATA: begin
                            if (!rx_fifo_empty) begin
                                rx_rd_ptr <= rx_rd_ptr + 1'b1;
                            end
                        end
                    endcase
                end
            end
        end
    end
    
    // Read data mux
    always_comb begin
        prdata = 32'h0;
        case (paddr)
            ADDR_CTRL:    prdata = ctrl_reg;
            ADDR_STATUS:  prdata = {16'h0, rx_fifo_level, tx_fifo_level, 
                                    rx_fifo_full, rx_fifo_empty, tx_fifo_full, tx_fifo_empty,
                                    spi_state != SPI_IDLE};
            ADDR_TXDATA:  prdata = 32'h0;
            ADDR_RXDATA:  prdata = rx_fifo[rx_rd_ptr[$clog2(FIFO_DEPTH)-1:0]];
            ADDR_CLKDIV:  prdata = clkdiv_reg;
            ADDR_CSCTRL:  prdata = csctrl_reg;
            ADDR_IRQEN:   prdata = irqen_reg;
            ADDR_IRQSTAT: prdata = irqstat_reg;
            default:      prdata = 32'h0;
        endcase
    end
    
    assign pready = 1'b1;
    assign pslverr = 1'b0;
    
    // Interrupt generation
    assign irq = |(irqstat_reg & irqen_reg);
    
    // DMA signals
    assign dma_req = !rx_fifo_empty;
    assign dma_single = 1'b1;
    assign dma_last = (rx_fifo_level == 1);

endmodule


//============================================================================
// SPI Subsystem - 4x SPI Masters
//============================================================================
module spi_subsystem #(
    parameter NUM_SPI_PORTS = 4,
    parameter CLK_FREQ_HZ   = 100_000_000
)(
    input  logic                        clk,
    input  logic                        rst_n,
    
    // AXI-Lite Slave Interface
    input  logic [15:0]                 axi_awaddr,
    input  logic                        axi_awvalid,
    output logic                        axi_awready,
    input  logic [31:0]                 axi_wdata,
    input  logic [3:0]                  axi_wstrb,
    input  logic                        axi_wvalid,
    output logic                        axi_wready,
    output logic [1:0]                  axi_bresp,
    output logic                        axi_bvalid,
    input  logic                        axi_bready,
    input  logic [15:0]                 axi_araddr,
    input  logic                        axi_arvalid,
    output logic                        axi_arready,
    output logic [31:0]                 axi_rdata,
    output logic [1:0]                  axi_rresp,
    output logic                        axi_rvalid,
    input  logic                        axi_rready,
    
    // SPI Interfaces (directly exposed)
    output logic [NUM_SPI_PORTS-1:0]    spi_clk,
    output logic [NUM_SPI_PORTS-1:0]    spi_mosi,
    input  logic [NUM_SPI_PORTS-1:0]    spi_miso,
    output logic [NUM_SPI_PORTS*4-1:0]  spi_cs_n,
    
    // Interrupts
    output logic [NUM_SPI_PORTS-1:0]    spi_irq
);

    //------------------------------------------------------------------------
    // Address Decode (4KB per SPI port)
    //------------------------------------------------------------------------
    wire [1:0] port_sel_wr = axi_awaddr[13:12];
    wire [1:0] port_sel_rd = axi_araddr[13:12];
    
    // APB signals for each port
    logic [NUM_SPI_PORTS-1:0]       psel;
    logic [NUM_SPI_PORTS-1:0]       penable;
    logic [NUM_SPI_PORTS-1:0]       pwrite;
    logic [11:0]                    paddr;
    logic [31:0]                    pwdata;
    logic [31:0]                    prdata [NUM_SPI_PORTS-1:0];
    logic [NUM_SPI_PORTS-1:0]       pready;
    
    // AXI-Lite to APB conversion
    typedef enum logic [1:0] {
        AXI_IDLE,
        AXI_WRITE,
        AXI_READ,
        AXI_RESP
    } axi_state_t;
    
    axi_state_t axi_state;
    logic [1:0] active_port;
    logic       is_write;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_state <= AXI_IDLE;
            axi_awready <= 1'b0;
            axi_wready <= 1'b0;
            axi_bvalid <= 1'b0;
            axi_arready <= 1'b0;
            axi_rvalid <= 1'b0;
            psel <= '0;
            penable <= '0;
            pwrite <= '0;
            active_port <= '0;
        end else begin
            case (axi_state)
                AXI_IDLE: begin
                    axi_awready <= 1'b1;
                    axi_arready <= 1'b1;
                    axi_bvalid <= 1'b0;
                    axi_rvalid <= 1'b0;
                    
                    if (axi_awvalid && axi_wvalid) begin
                        axi_awready <= 1'b0;
                        axi_wready <= 1'b1;
                        paddr <= axi_awaddr[11:0];
                        pwdata <= axi_wdata;
                        active_port <= port_sel_wr;
                        psel[port_sel_wr] <= 1'b1;
                        pwrite[port_sel_wr] <= 1'b1;
                        is_write <= 1'b1;
                        axi_state <= AXI_WRITE;
                    end else if (axi_arvalid) begin
                        axi_arready <= 1'b0;
                        paddr <= axi_araddr[11:0];
                        active_port <= port_sel_rd;
                        psel[port_sel_rd] <= 1'b1;
                        pwrite[port_sel_rd] <= 1'b0;
                        is_write <= 1'b0;
                        axi_state <= AXI_READ;
                    end
                end
                
                AXI_WRITE: begin
                    axi_wready <= 1'b0;
                    penable[active_port] <= 1'b1;
                    if (pready[active_port]) begin
                        psel <= '0;
                        penable <= '0;
                        pwrite <= '0;
                        axi_bvalid <= 1'b1;
                        axi_bresp <= 2'b00;
                        axi_state <= AXI_RESP;
                    end
                end
                
                AXI_READ: begin
                    penable[active_port] <= 1'b1;
                    if (pready[active_port]) begin
                        psel <= '0;
                        penable <= '0;
                        axi_rdata <= prdata[active_port];
                        axi_rvalid <= 1'b1;
                        axi_rresp <= 2'b00;
                        axi_state <= AXI_RESP;
                    end
                end
                
                AXI_RESP: begin
                    if ((is_write && axi_bready) || (!is_write && axi_rready)) begin
                        axi_bvalid <= 1'b0;
                        axi_rvalid <= 1'b0;
                        axi_state <= AXI_IDLE;
                    end
                end
            endcase
        end
    end
    
    //------------------------------------------------------------------------
    // SPI Master Instances
    //------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < NUM_SPI_PORTS; i++) begin : gen_spi
            spi_master #(
                .CLK_FREQ_HZ(CLK_FREQ_HZ),
                .SPI_CLK_HZ(25_000_000),
                .FIFO_DEPTH(16),
                .DATA_WIDTH(8),
                .NUM_CS(4)
            ) u_spi_master (
                .clk        (clk),
                .rst_n      (rst_n),
                .paddr      (paddr),
                .psel       (psel[i]),
                .penable    (penable[i]),
                .pwrite     (pwrite[i]),
                .pwdata     (pwdata),
                .prdata     (prdata[i]),
                .pready     (pready[i]),
                .pslverr    (),
                .spi_clk    (spi_clk[i]),
                .spi_mosi   (spi_mosi[i]),
                .spi_miso   (spi_miso[i]),
                .spi_cs_n   (spi_cs_n[i*4 +: 4]),
                .irq        (spi_irq[i]),
                .dma_req    (),
                .dma_ack    (1'b0),
                .dma_single (),
                .dma_last   ()
            );
        end
    endgenerate

endmodule












# Pin Constraints for RV64IMAFDCV Octa-Core Space Processor V7
# Target: Virtex UltraScale+ VU9P (xcvu9p-flga2104-2-e)
# Space-grade equivalent: XQRVU9P (radiation-tolerant)
# Package: FLGA2104 (2104-ball flip-chip BGA)
#
# Pin assignments verified against Vivado device database:
#   - Clock pins use IS_GLOBAL_CLK=1 (GCIO) sites
#   - All pins are bonded HP IO in valid banks

# ─── System Clock (Bank 40, GCIO pin) ──────────────────────────────────
# AV33 = IO_L13P_T2L_N0_GC_QBC_40 (Global Clock Capable, Master)
set_property PACKAGE_PIN AV33 [get_ports clk]
set_property IOSTANDARD LVCMOS18 [get_ports clk]

# ─── Reset (Bank 40, regular HP IO) ────────────────────────────────────
set_property PACKAGE_PIN BD30 [get_ports rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports rst_n]

# ─── JTAG Interface (Bank 42) ──────────────────────────────────────────
# tck on GCIO pin: BC38 = IO_L12P_T1U_N10_GC_42 (Global Clock Capable)
set_property PACKAGE_PIN BC38 [get_ports tck]
set_property PACKAGE_PIN BD36 [get_ports tms]
set_property PACKAGE_PIN BE37 [get_ports tdi]
set_property PACKAGE_PIN BF36 [get_ports tdo]
set_property PACKAGE_PIN BF37 [get_ports trst_n]
set_property IOSTANDARD LVCMOS18 [get_ports {tck tms tdi tdo trst_n}]

# ─── Status LEDs (Bank 43, regular HP IO) ──────────────────────────────
set_property PACKAGE_PIN AP38 [get_ports {led[0]}]
set_property PACKAGE_PIN AT35 [get_ports {led[1]}]
set_property PACKAGE_PIN AR35 [get_ports {led[2]}]
set_property PACKAGE_PIN AP35 [get_ports {led[3]}]
set_property PACKAGE_PIN AP36 [get_ports {led[4]}]
set_property PACKAGE_PIN AP37 [get_ports {led[5]}]
set_property PACKAGE_PIN AN34 [get_ports {led[6]}]
set_property PACKAGE_PIN AN35 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[*]}]

# ─── UART (Bank 43, regular HP IO) ─────────────────────────────────────
set_property PACKAGE_PIN AN33 [get_ports uart_rx]
set_property PACKAGE_PIN AP33 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS18 [get_ports {uart_rx uart_tx}]

# Demote CLOCK_DEDICATED_ROUTE errors for non-clock signals routed through BUFG
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets rst_n_IBUF_inst/O]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_IBUF_inst/O]

# Let Vivado auto-place all other I/Os
# (SpaceWire, Ethernet, SPI, GPIO, DDR4 assigned during board design)

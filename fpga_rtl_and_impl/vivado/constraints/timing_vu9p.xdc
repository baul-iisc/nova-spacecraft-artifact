# Timing Constraints for RV64IMAFDCV Octa-Core Space Processor V7
# Target: Virtex UltraScale+ VU9P (space-grade: XQRVU9P)
# Migration from VU095 (UltraScale 20nm) to VU9P (UltraScale+ 16nm)
#
# Revision 2: Clock reduced to 100 MHz and multicycle path constraints added
#   - 125 MHz failed with WNS = -3.977ns (critical path ~12ns in L2 cache)
#   - L2 cache tag BRAM → compare → data BRAM WE crosses all 3 SLRs
#   - Tag lookup + hit decision + data write is architecturally 2 pipeline stages

# Primary system clock - 100 MHz (10ns period)
# VU095 met timing at 80 MHz with WNS = +3.016ns (critical path ~9.5ns)
# VU9P SSI adds inter-SLR delay; 100 MHz gives margin for SLR crossings
create_clock -period 10.000 -name sys_clk [get_ports clk]

# JTAG clock - 10 MHz (unchanged)
create_clock -period 100.000 -name tck [get_ports tck]

# Clock domain crossing
set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks tck]

# Input delays
set_input_delay -clock sys_clk -max 2.0 [get_ports rst_n]
set_input_delay -clock sys_clk -min 0.5 [get_ports rst_n]

set_input_delay -clock tck -max 5.0 [get_ports {tdi tms}]
set_input_delay -clock tck -min 1.0 [get_ports {tdi tms}]

# Output delays
set_output_delay -clock tck -max 5.0 [get_ports tdo]
set_output_delay -clock tck -min 1.0 [get_ports tdo]

# False paths for async signals
set_false_path -from [get_ports rst_n]
set_false_path -to [get_ports led[*]]

# ==============================================================================
# Multicycle Path Constraints
# ==============================================================================
# The L2 cache tag-read → compare → hit/miss → data-write-enable path is
# architecturally a 2-cycle operation:
#   Cycle 1: Tag BRAM read + tag comparison + hit vector generation
#   Cycle 2: Data BRAM write/read gated by hit result
#
# Critical path at 125 MHz was:
#   tag_mem_way_reg_bram → CASDOUT cascade (3 BRAMs) → LUT6 compare →
#   CARRY8 → hit detect → LUT6 WE gen → data_mem_bank_reg_bram/WEA
#   Total: 10.93ns data path + SLR crossings

# L2 cache: tag BRAM output → data BRAM write enable (2-cycle setup)
set_multicycle_path 2 -setup \
    -from [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*tag_mem_way*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*data_mem_bank*}]
set_multicycle_path 1 -hold \
    -from [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*tag_mem_way*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*data_mem_bank*}]

# L2 cache: tag BRAM output → tag BRAM write (for replacement/eviction, 2-cycle)
set_multicycle_path 2 -setup \
    -from [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*tag_mem_way*reg_bram*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*tag_mem_way*reg_bram*}]
set_multicycle_path 1 -hold \
    -from [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*tag_mem_way*reg_bram*}] \
    -to   [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*tag_mem_way*reg_bram*}]

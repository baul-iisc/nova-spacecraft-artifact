#============================================================================
# Synthesis + Implementation: Integrated SoC with PCPI + CXL + JESD204B
#
# Top module: octa_core_rv64_soc_integrated
# Target: Virtex UltraScale+ VU9P (space-grade: XQRVU9P)
# Migration from VU095 (20nm UltraScale) → VU9P (16nm UltraScale+)
#
# Key improvements over VU095:
#   - 16nm FinFET+ fabric (20-30% faster than 20nm UltraScale)
#   - 2.2× more LUTs (1,182,240 vs 537,600)
#   - 8.9× more DSPs (6,840 vs 768)
#   - UltraRAM (960 blocks, 270 Mb on-chip)
#   - Target clock: 100 MHz (vs 80 MHz on VU095, reduced from 125 MHz)
#
# - PCPI router connects CORDIC and Systolic Array via custom instructions
# - Peripheral bus bridge connects TinyML, CCSDS, Debug via memory-mapped I/O
# - Full cache hierarchy (L1 I/D + MOESI + L2 4MB)
# - All peripherals (SpaceWire, TSN×2, CXL×2, JESD204B×2, SPI, UART, JTAG)
#============================================================================

set part xcvu9p-flga2104-2-e
set top  octa_core_rv64_soc_integrated

puts "============================================"
puts " PCPI+CXL+JESD204B SoC Synthesis (v7 - VU9P)"
puts " Part: $part"
puts " Top:  $top"
puts " Target Clock: 100 MHz (10ns)"
puts "============================================"

# Create project in memory
create_project -in_memory -part $part

# Add RTL sources
set rtl_dir "../rtl"

foreach subdir {core fpu vector dls octa_core cache accelerators compression \
               spacewire tsn_ethernet interfaces debug memory fault_tolerance \
               interconnect integration} {
    set files [glob -nocomplain $rtl_dir/$subdir/*.sv]
    if {[llength $files] > 0} {
        add_files -fileset sources_1 $files
    }
}

# Add constraints
add_files -fileset constrs_1 [glob -nocomplain constraints/*.xdc]

# Set top module
set_property top $top [current_fileset]

# ---- SYNTHESIS ----
puts "Starting synthesis..."

# Use Performance_ExplorePostRoutePhysOpt for maximum Fmax on UltraScale+
# Retiming enabled to allow register balancing across pipeline stages
synth_design -top $top -part $part \
    -directive PerformanceOptimized \
    -retiming \
    -flatten_hierarchy rebuilt

write_checkpoint -force build/${top}_synth.dcp

# Synthesis reports
file mkdir reports/synth
report_utilization -file reports/synth/utilization_synth.rpt
report_utilization -hierarchical -hierarchical_depth 5 \
    -file reports/synth/utilization_hierarchical_synth.rpt
report_timing_summary -file reports/synth/timing_synth.rpt

puts "Synthesis complete. Checking post-synth timing..."
set wns_synth [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
puts "Post-synthesis WNS: ${wns_synth} ns"

# ---- IMPLEMENTATION ----
puts "Starting implementation..."

# Optimization: ExploreWithRemap for UltraScale+ (takes advantage of LUT remapping)
opt_design -directive ExploreWithRemap

# ---- SLR FLOORPLANNING ----
# Constrain L2 cache to SLR1 (middle die) to eliminate inter-SLR crossings
# on the critical tag→data path that caused -3.977ns violation at 125 MHz
if {[llength [get_cells -quiet -hierarchical -filter {NAME =~ *u_l2_cache*}]] > 0} {
    puts "Creating SLR1 Pblock for L2 cache..."
    create_pblock pblock_l2_cache
    add_cells_to_pblock pblock_l2_cache [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*}]
    resize_pblock pblock_l2_cache -add {SLR1}
    set_property IS_SOFT TRUE [get_pblocks pblock_l2_cache]
}

# Placement: SSI-balanced + timing-focused
# SSI_SpreadSLLs balances logic across SLRs while respecting Pblock constraints
place_design -directive SSI_SpreadSLLs

# Post-placement physical optimization
phys_opt_design -directive AggressiveExplore

write_checkpoint -force build/${top}_placed.dcp

# Routing: AggressiveExplore explores more routing solutions
route_design -directive AggressiveExplore

# Final physical opt after routing for maximum performance
phys_opt_design -directive AggressiveExplore

write_checkpoint -force build/${top}_impl.dcp

# ---- REPORTS ----
file mkdir reports/impl
report_utilization -file reports/impl/utilization_impl.rpt
report_utilization -hierarchical -hierarchical_depth 5 \
    -file reports/impl/utilization_hierarchical.rpt
report_timing_summary -file reports/impl/timing_impl.rpt
report_timing_summary -delay_type min_max -max_paths 20 \
    -file reports/impl/timing_detailed.rpt
report_power -file reports/impl/power_impl.rpt
report_drc -file reports/impl/drc_impl.rpt
report_route_status -file reports/impl/route_status.rpt
report_clock_utilization -file reports/impl/clock_utilization.rpt
report_methodology -file reports/impl/methodology.rpt

# Print final timing summary
set wns_impl [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
set whs_impl [get_property SLACK [get_timing_paths -max_paths 1 -hold]]

puts "============================================"
puts " Implementation complete (VU9P @ 100 MHz)"
puts " Post-route WNS: ${wns_impl} ns"
puts " Post-route WHS: ${whs_impl} ns"
puts " Reports in: reports/synth/ and reports/impl/"
puts "============================================"

if {$wns_impl < 0} {
    puts "WARNING: Timing not met. Consider:"
    puts "  1. Reduce clock to 100 MHz (set period to 10.0 ns)"
    puts "  2. Enable multicycle paths in timing constraints"
    puts "  3. Add pipeline registers in critical paths"
}

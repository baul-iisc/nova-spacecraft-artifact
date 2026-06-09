#============================================================================
# Implementation Only: Read synthesis checkpoint and run P&R
# Skips re-synthesis — uses existing synth.dcp
# Target: Virtex UltraScale+ VU9P (XQRVU9P)
#============================================================================

set part xcvu9p-flga2104-2-e
set top  octa_core_rv64_soc_integrated

puts "============================================"
puts " Implementation Only (from synth checkpoint)"
puts " Part: $part  (VU9P UltraScale+)"
puts " Top:  $top"
puts "============================================"

# Read synthesis checkpoint
open_checkpoint build/${top}_synth.dcp

# ---- IMPLEMENTATION ----
puts "Starting implementation..."

# ExploreWithRemap takes advantage of UltraScale+ LUT remapping
opt_design -directive ExploreWithRemap

# SLR Floorplanning: constrain L2 cache to SLR1 (middle die) to avoid
# inter-SLR crossings on the critical tag→data path
if {[llength [get_cells -quiet -hierarchical -filter {NAME =~ *u_l2_cache*}]] > 0} {
    puts "Creating SLR1 Pblock for L2 cache..."
    create_pblock pblock_l2_cache
    add_cells_to_pblock pblock_l2_cache [get_cells -hierarchical -filter {NAME =~ *u_l2_cache*}]
    resize_pblock pblock_l2_cache -add {SLR1}
    set_property IS_SOFT TRUE [get_pblocks pblock_l2_cache]
}

# SSI-balanced placement with timing focus
place_design -directive SSI_SpreadSLLs

# Physical optimization before routing
phys_opt_design -directive AggressiveExplore

write_checkpoint -force build/${top}_placed.dcp

# AggressiveExplore routing for maximum Fmax
route_design -directive AggressiveExplore

# Post-route physical optimization
phys_opt_design -directive AggressiveExplore

write_checkpoint -force build/${top}_impl.dcp

# Implementation reports
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

puts "============================================"
puts " Implementation complete (VU9P @ 100 MHz)"
puts " Reports in: reports/impl/"
puts "============================================"

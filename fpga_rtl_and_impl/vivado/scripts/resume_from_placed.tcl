#========================================================================
# Resume Implementation from Placed Checkpoint
# Skips: synthesis, opt_design, place_design, phys_opt_design (pre-route)
# Starts from: route_design (AggressiveExplore)
#
# Target: Virtex UltraScale+ VU9P (xcvu9p-flga2104-2-e)
# Clock: 125 MHz (8 ns)
#========================================================================

set top octa_core_rv64_soc_integrated

puts "============================================"
puts " Resuming from placed checkpoint (VU9P)"
puts " Checkpoint: build/${top}_placed.dcp"
puts " Starting at: route_design"
puts "============================================"

# Open the placed checkpoint
open_checkpoint build/${top}_placed.dcp

# ---- ROUTING ----
puts "Starting route_design -directive AggressiveExplore ..."
route_design -directive AggressiveExplore

# Post-route physical optimization
puts "Starting post-route phys_opt_design ..."
phys_opt_design -directive AggressiveExplore

# Save implementation checkpoint
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
puts " Implementation complete (VU9P @ 125 MHz)"
puts " Post-route WNS: ${wns_impl} ns"
puts " Post-route WHS: ${whs_impl} ns"
puts " Reports in: reports/impl/"
puts "============================================"

if {$wns_impl < 0} {
    puts "WARNING: Timing not met. Consider:"
    puts "  1. Reduce clock to 100 MHz (set period to 10.0 ns)"
    puts "  2. Enable multicycle paths in timing constraints"
    puts "  3. Add pipeline registers in critical paths"
}

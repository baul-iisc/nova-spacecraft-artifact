# Implementation script for Octa-Core RV64IMAFDCV Space Processor V7 (VU9P)

set part [lindex $argv 0]
set top [lindex $argv 1]

puts "============================================"
puts " Implementation: RV64IMAFDCV Octa-Core SoC V7"
puts " Part: $part  (VU9P UltraScale+)"
puts " Top: $top"
puts "============================================"

# Open synthesis checkpoint
open_checkpoint build/${top}_synth.dcp

# Optimize design - ExploreWithRemap for UltraScale+ LUT remapping
opt_design -directive ExploreWithRemap

# Place design - ExtraTimingOpt for best timing on UltraScale+ SSI
place_design -directive ExtraTimingOpt

# Post-placement physical optimization
phys_opt_design -directive AggressiveExplore

# Write post-place checkpoint
write_checkpoint -force build/${top}_placed.dcp

# Route design - AggressiveExplore for maximum performance
route_design -directive AggressiveExplore

# Post-route physical optimization
phys_opt_design -directive AggressiveExplore

# Write implementation checkpoint
write_checkpoint -force build/${top}_impl.dcp

# Generate reports
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

puts "Implementation completed successfully"

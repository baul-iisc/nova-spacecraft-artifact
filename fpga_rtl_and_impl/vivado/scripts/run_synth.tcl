# Synthesis script for Octa-Core RV64IMAFDCV Space Processor V7 (VU9P)

set part [lindex $argv 0]
set top [lindex $argv 1]

puts "============================================"
puts " Synthesis: RV64IMAFDCV Octa-Core SoC V7"
puts " Part: $part  (VU9P UltraScale+)"
puts " Top: $top"
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

# Run synthesis - performance-optimized for UltraScale+
synth_design -top $top -part $part \
    -directive PerformanceOptimized \
    -retiming \
    -flatten_hierarchy rebuilt

# Save checkpoint
file mkdir build
write_checkpoint -force build/${top}_synth.dcp

# Reports
file mkdir reports/synth
report_utilization -file reports/synth/utilization_synth.rpt
report_utilization -hierarchical -hierarchical_depth 5 \
    -file reports/synth/utilization_hierarchical_synth.rpt
report_timing_summary -file reports/synth/timing_synth.rpt

puts "Synthesis completed successfully"

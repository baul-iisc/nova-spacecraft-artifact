#!/usr/bin/env python3
"""
Spacecraft Accelerator Study: Dedicated vs Hybrid vs Fully Shared
PhD Research: Chandraboul

Based on the working spacecraft_se_simple.py configuration.
Compares three accelerator allocation strategies.
"""

import argparse
import json
import sys
from os import path

import m5
from m5.objects import *
from m5.util import addToPath

# =============================================================================
# Configuration
# =============================================================================

# Area costs (relative units)
AREA_COST = {
    'matrix': 1.0,
    'cordic': 0.3,
}

# Power costs (Watts)
POWER_COST = {
    'matrix': 0.5,
    'cordic': 0.15,
}

# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(description="Spacecraft Accelerator Study")
parser.add_argument("--binary", type=str, required=True, help="RISC-V binary")
parser.add_argument("--options", type=str, default="", help="Options for binary")
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                    choices=["AtomicSimpleCPU", "TimingSimpleCPU"])
parser.add_argument("--num-cpus", type=int, default=1, help="Number of CPUs")
parser.add_argument("--mem-size", type=str, default="512MB", help="Memory size")
parser.add_argument("--allocation-mode", type=str, default="dedicated",
                    choices=["dedicated", "hybrid", "fully_shared"],
                    help="Accelerator allocation mode")

args = parser.parse_args()

# =============================================================================
# Calculate Resource Metrics
# =============================================================================

def calculate_metrics(mode, num_cpus):
    """Calculate area and power for the given configuration"""
    if mode == "dedicated":
        # Each core has its own matrix + cordic
        area = num_cpus * (AREA_COST['matrix'] + AREA_COST['cordic'])
        power = num_cpus * (POWER_COST['matrix'] + POWER_COST['cordic'])
    elif mode == "hybrid":
        # Dedicated cordic per core, shared matrix
        area = num_cpus * AREA_COST['cordic'] + AREA_COST['matrix'] * 1.5
        power = num_cpus * POWER_COST['cordic'] + POWER_COST['matrix'] * 1.5
    else:  # fully_shared
        # Single shared matrix + single shared cordic
        area = AREA_COST['matrix'] * 1.5 + AREA_COST['cordic'] * 1.3
        power = POWER_COST['matrix'] * 1.5 + POWER_COST['cordic'] * 1.3
    return area, power

# =============================================================================
# System Setup
# =============================================================================

print("\n" + "=" * 70)
print("SPACECRAFT ACCELERATOR STUDY: DEDICATED vs HYBRID vs FULLY SHARED")
print("=" * 70)
print(f"Allocation Mode: {args.allocation_mode.upper()}")
print(f"Number of CPUs:  {args.num_cpus}")
print(f"CPU Type:        {args.cpu_type}")
print("-" * 70)

# Calculate metrics
area, power = calculate_metrics(args.allocation_mode, args.num_cpus)
print(f"Estimated Area:  {area:.2f} (relative units)")
print(f"Estimated Power: {power:.2f} Watts")
print("=" * 70)

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing" if args.cpu_type != "AtomicSimpleCPU" else "atomic"
system.mem_ranges = [AddrRange(args.mem_size)]

# =============================================================================
# CPU Creation
# =============================================================================

if args.cpu_type == "AtomicSimpleCPU":
    system.cpu = [RiscvAtomicSimpleCPU() for i in range(args.num_cpus)]
else:
    system.cpu = [RiscvTimingSimpleCPU() for i in range(args.num_cpus)]

# =============================================================================
# Memory System
# =============================================================================

system.membus = SystemXBar()

for cpu in system.cpu:
    cpu.icache_port = system.membus.cpu_side_ports
    cpu.dcache_port = system.membus.cpu_side_ports

# Memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports

# =============================================================================
# Interrupt Controllers
# =============================================================================

for cpu in system.cpu:
    cpu.createInterruptController()

# =============================================================================
# Accelerator Configuration Info
# =============================================================================

# Note: The actual hardware accelerator operations are handled by gem5's
# instruction execution with custom RISC-V ISA extensions. This config
# tracks the allocation mode for analysis purposes.

config_info = {
    'mode': args.allocation_mode,
    'num_cpus': args.num_cpus,
    'area': area,
    'power': power,
    'cpu_type': args.cpu_type,
}

# Print accelerator configuration based on mode
print("\nAccelerator Configuration:")
if args.allocation_mode == "dedicated":
    for i in range(args.num_cpus):
        print(f"  Core {i}: Dedicated Matrix + Dedicated CORDIC")
elif args.allocation_mode == "hybrid":
    print("  Shared: Matrix Accelerator (all cores)")
    for i in range(args.num_cpus):
        print(f"  Core {i}: Dedicated CORDIC")
else:
    print("  Shared: Matrix Accelerator (all cores)")
    print("  Shared: CORDIC Accelerator (all cores)")

# =============================================================================
# Workload
# =============================================================================

system.workload = SEWorkload.init_compatible(args.binary)

process = Process()
process.cmd = [args.binary]
if args.options:
    process.cmd += args.options.split()

for cpu in system.cpu:
    cpu.workload = process
    cpu.createThreads()

# =============================================================================
# Save Configuration
# =============================================================================

root = Root(full_system=False, system=system)

# Save config to output directory
m5.instantiate()

config_file = path.join(m5.options.outdir, 'accel_config.json')
with open(config_file, 'w') as f:
    json.dump(config_info, f, indent=2)

# =============================================================================
# Run Simulation
# =============================================================================

print(f"\nBinary: {args.binary}")
print(f"Memory: {args.mem_size}")
print("=" * 70)
print("\nStarting simulation...")

exit_event = m5.simulate()

print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")

# =============================================================================
# Summary
# =============================================================================

print("\n" + "=" * 70)
print("SIMULATION SUMMARY")
print("=" * 70)
print(f"Mode:           {args.allocation_mode.upper()}")
print(f"CPUs:           {args.num_cpus}")
print(f"Simulated Time: {m5.curTick() / 1e12:.6f} seconds")
print(f"Area Cost:      {area:.2f} units")
print(f"Power Cost:     {power:.2f} Watts")
print("=" * 70)


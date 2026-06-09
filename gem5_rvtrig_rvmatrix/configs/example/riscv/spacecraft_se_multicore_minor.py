"""
Spacecraft SE Mode - Multicore with MinorCPU (Pipelined In-Order)
PhD Research: Chandraboul

This configuration uses MinorCPU (4-stage pipelined in-order core) instead of
TimingSimpleCPU, for sensitivity study validating that scaling trends hold
across different CPU models.

Key differences from TimingSimpleCPU:
- 4-stage pipeline: Fetch → Decode → Execute → Commit
- Branch prediction (tournament predictor by default)
- Pipeline hazard handling (data forwarding, stalls)
- Memory-level parallelism (pipeline continues while waiting for cache)
- More realistic cache miss interaction

Usage:
    gem5.opt spacecraft_se_multicore_minor.py -c <binary> -n <num_cpus> \\
        --mem-size=2GB --max-stack-size=64MiB
"""

import argparse
import os
import sys

import m5
from m5.objects import *
from m5.util import addToPath

addToPath("../../")

from common import Options, Simulation, CacheConfig, MemConfig

# =============================================================================
# Command Line Arguments
# =============================================================================
parser = argparse.ArgumentParser(
    description='Spacecraft SE Mode - MinorCPU Multicore with Pthreads'
)

Options.addCommonOptions(parser)
Options.addSEOptions(parser)

# Custom options
parser.add_argument('--enable-caches', action='store_true',
                    help='Enable L1 caches')
parser.add_argument('--max-stack-size', type=str, default='64MiB',
                    help='Maximum per-thread stack size reported to the guest runtime')

args = parser.parse_args()

# Force MinorCPU - override whatever cpu-type was specified
args.cpu_type = 'RiscvMinorCPU'
args.enable_caches = True  # MinorCPU requires caches

# =============================================================================
# Validate Arguments
# =============================================================================
if not args.cmd:
    print("Error: No command specified. Use --cmd=<binary>", file=sys.stderr)
    sys.exit(1)

# =============================================================================
# System Configuration
# =============================================================================
print("=" * 70)
print("SPACECRAFT SE MODE - MinorCPU SENSITIVITY STUDY")
print("PhD Research: Chandraboul")
print("=" * 70)

# Import MinorCPU
from m5.objects.RiscvCPU import RiscvMinorCPU

CPUClass = RiscvMinorCPU
test_mem_mode = 'timing'

print(f"CPUs:        {args.num_cpus} x {CPUClass.__name__}")
print(f"CPU Model:   MinorCPU (4-stage pipelined in-order)")
print(f"Memory Mode: {test_mem_mode}")
print(f"Binary:      {args.cmd}")
if args.options:
    print(f"Options:     {args.options}")

# Create system
system = System(
    cpu=[CPUClass(cpu_id=i) for i in range(args.num_cpus)],
    mem_mode=test_mem_mode,
    mem_ranges=[AddrRange(args.mem_size)],
    cache_line_size=args.cacheline_size,
)

# Enable multi-threading for pthreads support
system.multi_thread = True

# Voltage domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

# Clock domains
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock,
    voltage_domain=system.voltage_domain
)

system.cpu_voltage_domain = VoltageDomain()
system.cpu_clk_domain = SrcClockDomain(
    clock=args.cpu_clock,
    voltage_domain=system.cpu_voltage_domain
)

# Configure CPUs
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

# =============================================================================
# Memory Hierarchy - Same as TimingSimpleCPU config for fair comparison
# =============================================================================
system.membus = SystemXBar()

from common.Caches import *

print("Caches:      L1 I/D enabled (32kB each, 2-way)")

for cpu in system.cpu:
    # Create L1 caches - SAME as TimingSimpleCPU config
    cpu.icache = L1_ICache(size='32kB', assoc=2)
    cpu.dcache = L1_DCache(size='32kB', assoc=2)

    # Connect caches
    cpu.icache.cpu_side = cpu.icache_port
    cpu.dcache.cpu_side = cpu.dcache_port
    cpu.icache.mem_side = system.membus.cpu_side_ports
    cpu.dcache.mem_side = system.membus.cpu_side_ports

    cpu.createInterruptController()

# Memory controller - SAME as TimingSimpleCPU config
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# System port
system.system_port = system.membus.cpu_side_ports

# =============================================================================
# Workload Setup
# =============================================================================
binary = args.cmd
cmd_args = args.options.split() if args.options else []

# Create process
process = Process(pid=100)
process.executable = binary
process.cmd = [binary] + cmd_args
process.cwd = os.getcwd()
process.maxStackSize = args.max_stack_size

# SEWorkload for RISC-V
system.workload = SEWorkload.init_compatible(binary)

# CRITICAL for pthreads: All CPUs share the SAME process
for i, cpu in enumerate(system.cpu):
    cpu.workload = process
    cpu.createThreads()

print(f"Workload:    {binary}")
print("Threading:   Pthreads via clone syscall")
print(f"Stack limit: {args.max_stack_size}")
print("=" * 70)

# =============================================================================
# Instantiate and Run
# =============================================================================
root = Root(full_system=False, system=system)

m5.instantiate()

print("\n*** Starting MinorCPU Simulation ***\n")

exit_event = m5.simulate()

print("\n" + "=" * 70)
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
print("=" * 70)

# Print some stats
print("\nSimulation Statistics:")
print(f"  Total ticks:     {m5.curTick()}")
print(f"  Simulated time:  {m5.curTick() / 1e12:.6f} seconds")

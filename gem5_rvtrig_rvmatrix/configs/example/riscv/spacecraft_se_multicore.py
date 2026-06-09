"""
Spacecraft SE Mode - Multicore with Pthreads Support
PhD Research: Chandraboul

This configuration properly supports pthreads-based multicore workloads in SE mode.
For RISC-V gem5 simulation.

Key Features:
- Uses TimingSimpleCPU for proper multicore simulation
- Supports pthreads via clone syscall
- Uses ISA-level accelerators (MMACDF, fsin.d, fcos.d) - works in SE mode!
- Simulates shared vs dedicated contention via software synchronization
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
    description='Spacecraft SE Mode - Multicore with Pthreads'
)

Options.addCommonOptions(parser)
Options.addSEOptions(parser)

# Custom options
parser.add_argument('--enable-caches', action='store_true',
                    help='Enable L1 caches')
parser.add_argument('--max-stack-size', type=str, default='64MiB',
                    help='Maximum per-thread stack size reported to the guest runtime')

args = parser.parse_args()

# Force TimingSimpleCPU for proper contention tracking
# AtomicSimpleCPU uses instant memory - no timing/contention modeling!
# Must use TimingSimpleCPU for PhD contention study
if 'Atomic' in args.cpu_type:
    print(f"Note: Forcing TimingSimpleCPU for contention tracking (was {args.cpu_type})")
    # Handle ISA-prefixed CPU names (e.g., RiscvAtomicSimpleCPU -> RiscvTimingSimpleCPU)
    args.cpu_type = args.cpu_type.replace('Atomic', 'Timing')
    # TimingSimpleCPU requires caches for memory hierarchy
    args.enable_caches = True

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
print("SPACECRAFT SE MODE - MULTICORE WITH PTHREADS")
print("PhD Research: Chandraboul")
print("=" * 70)

# Get CPU class
(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(args)

print(f"CPUs:        {args.num_cpus} x {CPUClass.__name__}")
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
# Memory Hierarchy
# =============================================================================
system.membus = SystemXBar()

if args.enable_caches or test_mem_mode == 'timing':
    # Add L1 caches for timing mode
    from common.Caches import *
    
    print("Caches:      L1 I/D enabled")
    
    for cpu in system.cpu:
        # Create L1 caches
        cpu.icache = L1_ICache(size='32kB', assoc=2)
        cpu.dcache = L1_DCache(size='32kB', assoc=2)
        
        # Connect caches
        cpu.icache.cpu_side = cpu.icache_port
        cpu.dcache.cpu_side = cpu.dcache_port
        cpu.icache.mem_side = system.membus.cpu_side_ports
        cpu.dcache.mem_side = system.membus.cpu_side_ports
        
        cpu.createInterruptController()
else:
    # Direct connection for atomic mode
    print("Caches:      Disabled (atomic mode)")
    
    for cpu in system.cpu:
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports
        cpu.createInterruptController()

# Memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# System port
system.system_port = system.membus.cpu_side_ports

# =============================================================================
# Workload Setup
# =============================================================================
# Parse command and options
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
# Threads created via clone() will be scheduled on available CPUs
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

print("\n*** Starting Simulation ***\n")

exit_event = m5.simulate()

print("\n" + "=" * 70)
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
print("=" * 70)

# Print some stats
print("\nSimulation Statistics:")
print(f"  Total ticks:     {m5.curTick()}")
print(f"  Simulated time:  {m5.curTick() / 1e12:.6f} seconds")


"""
Spacecraft DLS SoC - Simple SE Mode Configuration
PhD Research: Chandraboul

Simplified configuration for testing - single core, simple caches
"""

import argparse
import sys
from os import path

import m5
from m5.objects import *
from m5.util import addToPath

# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(description="Simple Spacecraft SoC Test")
parser.add_argument("--binary", type=str, required=True, help="RISC-V binary")
parser.add_argument("--options", type=str, default="", help="Options to pass to binary")
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                    choices=["AtomicSimpleCPU", "TimingSimpleCPU", "MinorCPU"])
parser.add_argument("--num-cpus", type=int, default=1, help="Number of CPUs")
parser.add_argument("--mem-size", type=str, default="512MB", help="Memory size")
parser.add_argument("--enable-ml", action="store_true")
parser.add_argument("--enable-compression", action="store_true")
parser.add_argument("--enable-all-accel", action="store_true")

args = parser.parse_args()

if args.enable_all_accel:
    args.enable_ml = True
    args.enable_compression = True

# =============================================================================
# System Setup
# =============================================================================

print("\n" + "="*60)
print("SPACECRAFT SOC - SIMPLE SE MODE TEST")
print("="*60)

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing"
system.mem_ranges = [AddrRange(args.mem_size)]

# =============================================================================
# CPU Creation
# =============================================================================

if args.cpu_type == "AtomicSimpleCPU":
    system.cpu = [RiscvAtomicSimpleCPU() for i in range(args.num_cpus)]
    system.mem_mode = "atomic"
elif args.cpu_type == "TimingSimpleCPU":
    system.cpu = [RiscvTimingSimpleCPU() for i in range(args.num_cpus)]
elif args.cpu_type == "MinorCPU":
    system.cpu = [RiscvMinorCPU() for i in range(args.num_cpus)]

print(f"CPUs: {args.num_cpus} x {args.cpu_type}")

# =============================================================================
# Memory System
# =============================================================================

system.membus = SystemXBar()

# For atomic, connect directly to membus
if args.cpu_type == "AtomicSimpleCPU":
    for cpu in system.cpu:
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports
else:
    # For timing, create simple caches
    for i, cpu in enumerate(system.cpu):
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
# Run
# =============================================================================

root = Root(full_system=False, system=system)
m5.instantiate()

print(f"\nBinary: {args.binary}")
print(f"Memory: {args.mem_size}")
print("="*60)
print("\nBeginning simulation!")

exit_event = m5.simulate()

print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")


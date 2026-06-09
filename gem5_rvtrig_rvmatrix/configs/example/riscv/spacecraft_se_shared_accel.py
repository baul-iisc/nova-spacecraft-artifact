"""
Spacecraft SE Mode with Shared Accelerators (MMIO)
PhD Research: Chandraboul

This configuration creates a multicore RISC-V system with shared accelerators
connected via MMIO for contention testing.

NOTE: In SE mode, MMIO-based accelerators require special handling.
For proper MMIO testing, use Full System mode instead.
This SE mode config tests the pthreads-based workload with SW fallback.
"""

import argparse
import sys
import os

import m5
from m5.objects import *
from m5.util import addToPath

addToPath("../../")

# ============================================================================
# Main Configuration
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Spacecraft SE with Shared Accelerators')
    parser.add_argument('--binary', type=str, required=True, help='Binary to run')
    parser.add_argument('--options', type=str, default='', help='Binary options')
    parser.add_argument('--num-cpus', type=int, default=4, help='Number of CPUs')
    parser.add_argument('--mem-size', type=str, default='512MB', help='Memory size')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("SPACECRAFT SE MODE - MULTICORE PTHREADS TEST")
    print("PhD Research: Chandraboul")
    print("=" * 70)
    print(f"CPUs: {args.num_cpus}")
    print(f"Binary: {args.binary}")
    print("NOTE: MMIO accelerators require Full System mode.")
    print("      This SE mode runs SW fallback path.")
    print("=" * 70)
    
    # Create system
    system = System()
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = '1GHz'
    system.clk_domain.voltage_domain = VoltageDomain()
    
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange(args.mem_size)]
    
    # Create CPUs
    system.cpu = [TimingSimpleCPU(cpu_id=i) for i in range(args.num_cpus)]
    
    # Memory bus
    system.membus = SystemXBar()
    
    # Connect CPUs to membus (directly, no caches for simplicity in SE mode)
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
    
    # Workload
    system.workload = SEWorkload.init_compatible(args.binary)
    
    # Create process
    process = Process()
    process.cmd = [args.binary] + args.options.split()
    
    # Assign process to first CPU (main thread)
    system.cpu[0].workload = process
    system.cpu[0].createThreads()
    
    # Other CPUs will get threads via clone syscall
    for i in range(1, args.num_cpus):
        system.cpu[i].workload = process
        system.cpu[i].createThreads()
    
    # Root object
    root = Root(full_system=False, system=system)
    
    # Instantiate
    m5.instantiate()
    
    print("\nBeginning simulation!")
    print("=" * 70)
    
    # Run simulation
    exit_event = m5.simulate()
    
    print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")

if __name__ == "__m5_main__":
    main()

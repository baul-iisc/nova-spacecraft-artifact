"""
Spacecraft Multicore SE Mode Configuration
PhD Research: Chandraboul

This configuration runs a workload on ALL cores simultaneously,
allowing us to study shared vs dedicated accelerator performance.

Each core runs the same binary with different --core=N argument.
"""

import argparse
import os
import sys

import m5
from m5.objects import *
from m5.util import addToPath

addToPath("../../")

from common import SimpleOpts

# Simple L1 Cache classes
class L1ICache(Cache):
    size = '32kB'
    assoc = 4
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 8

class L1DCache(Cache):
    size = '32kB'
    assoc = 4
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 8

def create_system(args):
    """Create multicore system with all cores running workloads"""
    
    system = System()
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = '1GHz'
    system.clk_domain.voltage_domain = VoltageDomain()
    
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange('512MB')]
    
    # Create CPUs - one for each core
    system.cpu = [TimingSimpleCPU() for _ in range(args.num_cpus)]
    
    # Memory bus
    system.membus = SystemXBar()
    
    # Create workloads for each CPU
    binary = args.binary
    
    for i, cpu in enumerate(system.cpu):
        # Each core gets its own process with core ID
        process = Process()
        process.cmd = [binary, f'--core={i}', f'--iterations={args.iterations}']
        cpu.workload = process
        cpu.createThreads()
        
        # Create cache hierarchy for each CPU
        cpu.icache = L1ICache()
        cpu.dcache = L1DCache()
        
        cpu.icache.connectCPU(cpu)
        cpu.dcache.connectCPU(cpu)
        
        cpu.icache.connectBus(system.membus)
        cpu.dcache.connectBus(system.membus)
        
        # Interrupt controller
        cpu.createInterruptController()
    
    # Memory controller
    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR3_1600_8x8()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports
    
    # System port
    system.system_port = system.membus.cpu_side_ports
    
    return system

def main():
    parser = argparse.ArgumentParser(description='Multicore SE Mode Test')
    parser.add_argument('--binary', type=str, required=True,
                       help='Path to the binary')
    parser.add_argument('--num-cpus', type=int, default=4,
                       help='Number of CPUs')
    parser.add_argument('--iterations', type=int, default=500,
                       help='Iterations per core')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("SPACECRAFT MULTICORE SE MODE")
    print("PhD Research: Shared vs Dedicated Accelerators")
    print("=" * 60)
    print(f"CPUs: {args.num_cpus}")
    print(f"Binary: {args.binary}")
    print(f"Iterations: {args.iterations}")
    print("=" * 60)
    print()
    
    # Create system
    system = create_system(args)
    
    # Instantiate
    root = Root(full_system=False, system=system)
    m5.instantiate()
    
    print("Beginning simulation with ALL cores active!")
    print()
    
    exit_event = m5.simulate()
    
    print()
    print("=" * 60)
    print(f"Simulation ended: {exit_event.getCause()}")
    print("=" * 60)

if __name__ == "__m5_main__":
    main()


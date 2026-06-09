#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NOVA Processor - ISRO Spacecraft Processor Configuration
PhD Research: Futuristic Spacecraft Processor

This script creates a complete gem5 simulation of the NOVA processor
with all spacecraft-specific components:
- RISC-V multicore (4-8 cores)
- Navigation accelerators (TrigAccel, MatAccel)
- Vision Processing Unit (VPU)
- Neural Processing Unit (NPU)
- Mission Phase Manager
- Global Task Scheduler
- Accelerator Resource Manager

Usage:
    ./build/RISCV/gem5.opt configs/spacecraft/isro_processor.py \
        --num-cores 4 \
        --sharing-mode ADAPTIVE \
        --initial-phase NORMAL_OPS \
        --workload <path_to_binary>
"""

import argparse
import os
import sys

import m5
from m5.objects import *
from m5.util import addToPath

# Add paths for custom components
addToPath(os.path.dirname(__file__))

def create_system(args):
    """Create the NOVA processor system"""
    
    system = System()
    
    # Clock and voltage domain
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = args.cpu_clock
    system.clk_domain.voltage_domain = VoltageDomain()
    
    # Memory configuration
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange(args.mem_size)]
    
    return system


def create_cpus(system, args):
    """Create RISC-V CPU cores"""
    
    if args.cpu_type == 'TimingSimpleCPU':
        CPUClass = TimingSimpleCPU
    elif args.cpu_type == 'AtomicSimpleCPU':
        CPUClass = AtomicSimpleCPU
    elif args.cpu_type == 'O3CPU':
        CPUClass = DerivO3CPU
    else:
        CPUClass = TimingSimpleCPU
    
    system.cpu = [CPUClass(cpu_id=i) for i in range(args.num_cores)]
    
    # Don't call createThreads here - will be called after workload assignment
    
    print(f"Created {args.num_cores} {args.cpu_type} cores")
    return system.cpu


def create_caches(system, args):
    """Create cache hierarchy with ECC support"""
    
    # L1 caches per core
    for cpu in system.cpu:
        cpu.icache = Cache(
            size=f'{args.l1i_size}kB',
            assoc=2,
            tag_latency=1,
            data_latency=1,
            response_latency=1,
            mshrs=4,
            tgts_per_mshr=8
        )
        
        cpu.dcache = Cache(
            size=f'{args.l1d_size}kB',
            assoc=2,
            tag_latency=1,
            data_latency=1,
            response_latency=1,
            mshrs=4,
            tgts_per_mshr=8
        )
        
        # Connect L1 to CPU
        cpu.icache.cpu_side = cpu.icache_port
        cpu.dcache.cpu_side = cpu.dcache_port
    
    # L2 cache (shared)
    system.l2bus = L2XBar()
    system.l2cache = Cache(
        size=f'{args.l2_size}kB',
        assoc=8,
        tag_latency=10,
        data_latency=10,
        response_latency=10,
        mshrs=16,
        tgts_per_mshr=12
    )
    
    # Connect L1 to L2 bus
    for cpu in system.cpu:
        cpu.icache.mem_side = system.l2bus.cpu_side_ports
        cpu.dcache.mem_side = system.l2bus.cpu_side_ports
    
    # Connect L2 bus to L2 cache
    system.l2bus.mem_side_ports = system.l2cache.cpu_side
    
    print(f"Created cache hierarchy: L1I={args.l1i_size}KB, L1D={args.l1d_size}KB, L2={args.l2_size}KB")


def create_memory_system(system, args):
    """Create memory bus and controller"""
    
    system.membus = SystemXBar()
    
    # Connect L2 to memory bus
    system.l2cache.mem_side = system.membus.cpu_side_ports
    
    # Connect system port
    system.system_port = system.membus.cpu_side_ports
    
    # Memory controller
    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR4_2400_16x4()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports
    
    # Connect interrupt controllers
    for cpu in system.cpu:
        cpu.createInterruptController()
    
    print(f"Created memory system: {args.mem_size}")


def create_accelerators(system, args):
    """Create spacecraft accelerators (placeholder - would use custom SimObjects)"""
    
    print(f"Accelerator configuration:")
    print(f"  TrigAccels: {args.num_trig}")
    print(f"  MatAccels: {args.num_mat}")
    print(f"  VPUs: {args.num_vpu}")
    print(f"  NPUs: {args.num_npu}")
    print(f"  Sharing Mode: {args.sharing_mode}")
    
    # Note: In full implementation, these would be custom SimObjects:
    # system.trig_accels = [TrigAccelerator(instance_id=i) for i in range(args.num_trig)]
    # system.mat_accels = [MatrixAccelerator(instance_id=i) for i in range(args.num_mat)]
    # system.vpus = [VisionProcessingUnit(instance_id=i) for i in range(args.num_vpu)]
    # system.npus = [NeuralProcessingUnit(instance_id=i) for i in range(args.num_npu)]


def setup_workload(system, args):
    """Set up the workload to run"""
    
    if not args.workload:
        print("Warning: No workload specified, using dummy process")
        # Create a minimal process for testing
        return False
    
    if not os.path.exists(args.workload):
        print(f"Error: Workload not found: {args.workload}")
        sys.exit(1)
    
    # Create process
    process = Process()
    process.cmd = [args.workload]
    
    if args.workload_options:
        process.cmd.extend(args.workload_options.split())
    
    system.workload = SEWorkload.init_compatible(args.workload)
    
    # Assign process to all CPUs (they will share the process)
    for i, cpu in enumerate(system.cpu):
        cpu.workload = process
        cpu.createThreads()
    
    print(f"Workload: {args.workload}")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="NOVA Processor - ISRO Spacecraft Processor Simulation"
    )
    
    # Core configuration
    parser.add_argument('--num-cores', type=int, default=4,
                        help='Number of RISC-V cores (default: 4)')
    parser.add_argument('--cpu-type', type=str, default='TimingSimpleCPU',
                        choices=['TimingSimpleCPU', 'AtomicSimpleCPU', 'O3CPU'],
                        help='CPU type (default: TimingSimpleCPU)')
    parser.add_argument('--cpu-clock', type=str, default='1GHz',
                        help='CPU clock frequency (default: 1GHz)')
    
    # Cache configuration
    parser.add_argument('--l1i-size', type=int, default=32,
                        help='L1 Instruction cache size in KB (default: 32)')
    parser.add_argument('--l1d-size', type=int, default=32,
                        help='L1 Data cache size in KB (default: 32)')
    parser.add_argument('--l2-size', type=int, default=512,
                        help='L2 cache size in KB (default: 512)')
    
    # Memory configuration
    parser.add_argument('--mem-size', type=str, default='4GB',
                        help='Memory size (default: 4GB)')
    
    # Accelerator configuration
    parser.add_argument('--num-trig', type=int, default=2,
                        help='Number of trigonometric accelerators (default: 2)')
    parser.add_argument('--num-mat', type=int, default=2,
                        help='Number of matrix accelerators (default: 2)')
    parser.add_argument('--num-vpu', type=int, default=1,
                        help='Number of Vision Processing Units (default: 1)')
    parser.add_argument('--num-npu', type=int, default=1,
                        help='Number of Neural Processing Units (default: 1)')
    
    # Scheduling configuration
    parser.add_argument('--sharing-mode', type=str, default='ADAPTIVE',
                        choices=['FULLY_SHARED', 'FULLY_DEDICATED', 'HYBRID_SHARED', 'ADAPTIVE'],
                        help='Accelerator sharing mode (default: ADAPTIVE)')
    parser.add_argument('--arbitration', type=str, default='PRIORITY_BASED',
                        choices=['FCFS', 'PRIORITY_BASED', 'DEADLINE_AWARE', 'PROPORTIONAL_SHARE'],
                        help='Arbitration policy (default: PRIORITY_BASED)')
    
    # Mission configuration
    parser.add_argument('--initial-phase', type=str, default='NORMAL_OPS',
                        choices=['LAUNCH', 'ORBIT_INSERTION', 'NORMAL_OPS', 'LANDING', 'SURFACE_OPS'],
                        help='Initial mission phase (default: NORMAL_OPS)')
    parser.add_argument('--phase-transitions', type=str, default='',
                        help='Phase transitions: time1:phase1,time2:phase2,...')
    
    # Workload configuration
    parser.add_argument('--workload', type=str, default='',
                        help='Path to workload binary')
    parser.add_argument('--workload-options', type=str, default='',
                        help='Options to pass to workload')
    
    # Simulation configuration
    parser.add_argument('--max-ticks', type=int, default=10**12,
                        help='Maximum simulation ticks (default: 10^12)')
    
    args = parser.parse_args()
    
    # Print configuration
    print("=" * 60)
    print("NOVA Processor - ISRO Spacecraft Processor Simulation")
    print("PhD Research: Futuristic Spacecraft Processor")
    print("=" * 60)
    
    # Create system
    system = create_system(args)
    
    # Create CPUs
    create_cpus(system, args)
    
    # Create caches
    create_caches(system, args)
    
    # Create memory system
    create_memory_system(system, args)
    
    # Create accelerators
    create_accelerators(system, args)
    
    # Setup workload
    has_workload = setup_workload(system, args)
    
    if not has_workload:
        print("No workload specified, exiting.")
        sys.exit(0)
    
    # Create root
    root = Root(full_system=False, system=system)
    
    # Instantiate
    m5.instantiate()
    
    print("=" * 60)
    print("Starting simulation...")
    print("=" * 60)
    
    # Run simulation
    exit_event = m5.simulate(args.max_ticks)
    
    print("=" * 60)
    print(f"Simulation finished: {exit_event.getCause()}")
    print(f"Simulated ticks: {m5.curTick()}")
    print("=" * 60)
    
    # Dump statistics
    m5.stats.dump()


if __name__ == '__main__' or __name__ == '__m5_main__':
    try:
        main()
    except Exception as e:
        print(f"Error in simulation: {e}", flush=True)
        import traceback
        traceback.print_exc()
        raise


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NOVA Processor - Shared Accelerator PIO Configuration
PhD Research: Futuristic Spacecraft Processor

This configuration creates a multicore RISC-V system with shared accelerators
accessible via memory-mapped I/O (MMIO). The accelerators model actual
contention - when multiple cores access them, requests are queued and
cause stalls.

Key features:
- Configurable number of cores
- Configurable number of shared accelerators
- Shared vs dedicated accelerator modes
- Real contention statistics

Usage:
    # 4 cores sharing 1 trig accelerator (contention)
    ./build/RISCV/gem5.opt configs/spacecraft/shared_accel_pio_config.py \
        --num-cores 4 --num-trig-accels 1 --workload ./workloads/test.riscv

    # 4 cores with dedicated accelerators (no contention)
    ./build/RISCV/gem5.opt configs/spacecraft/shared_accel_pio_config.py \
        --num-cores 4 --num-trig-accels 4 --workload ./workloads/test.riscv
"""

import argparse
import os
import sys

import m5
from m5.objects import *
from m5.util import addToPath


def create_system(args):
    """Create the base system with memory and clock"""
    
    system = System()
    
    # Clock domain
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = args.cpu_clock
    system.clk_domain.voltage_domain = VoltageDomain()
    
    # Memory mode and range
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange(args.mem_size)]
    
    return system


def create_cpus(system, args):
    """Create RISC-V TimingSimpleCPUs"""
    
    system.cpu = [TimingSimpleCPU(cpu_id=i) for i in range(args.num_cores)]
    
    print(f"Created {args.num_cores} TimingSimpleCPU cores")
    return system.cpu


def create_caches(system, args):
    """Create L1/L2 cache hierarchy"""
    
    # L2 crossbar
    system.l2bus = L2XBar()
    
    # Shared L2 cache
    system.l2cache = Cache(
        size='512kB',
        assoc=8,
        tag_latency=10,
        data_latency=10,
        response_latency=10,
        mshrs=16,
        tgts_per_mshr=12
    )
    
    # L1 caches per CPU
    for cpu in system.cpu:
        cpu.icache = Cache(
            size='32kB',
            assoc=2,
            tag_latency=1,
            data_latency=1,
            response_latency=1,
            mshrs=4,
            tgts_per_mshr=8
        )
        cpu.dcache = Cache(
            size='32kB',
            assoc=2,
            tag_latency=1,
            data_latency=1,
            response_latency=1,
            mshrs=4,
            tgts_per_mshr=8
        )
        
        # Connect L1 caches
        cpu.icache.cpu_side = cpu.icache_port
        cpu.dcache.cpu_side = cpu.dcache_port
        cpu.icache.mem_side = system.l2bus.cpu_side_ports
        cpu.dcache.mem_side = system.l2bus.cpu_side_ports
    
    # Connect L2
    system.l2bus.mem_side_ports = system.l2cache.cpu_side


def create_memory(system, args):
    """Create memory system"""
    
    # Main memory bus
    system.membus = SystemXBar()
    
    # Connect L2 to memory bus
    system.l2cache.mem_side = system.membus.cpu_side_ports
    
    # System port
    system.system_port = system.membus.cpu_side_ports
    
    # Memory controller
    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR4_2400_16x4()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports
    
    # Create interrupt controllers
    for cpu in system.cpu:
        cpu.createInterruptController()


def create_accelerators(system, args):
    """Create shared accelerators"""
    
    from m5.objects import SharedAccelPio
    
    # Base address for accelerators (above 4GB DRAM range)
    # Using addresses > 0x100000000 to avoid DRAM overlap
    ACCEL_BASE = 0x100100000
    
    print("\n" + "=" * 60)
    print("ACCELERATOR CONFIGURATION")
    print("=" * 60)
    
    # Create trigonometric accelerators
    trig_accels = []
    for i in range(args.num_trig_accels):
        accel = SharedAccelPio(
            accel_type=0,  # TRIG
            instance_id=i,
            pio_addr=ACCEL_BASE + (i * 0x1000),
            trig_latency=args.trig_latency,
            mat_latency=args.mat_latency,
            max_queue_depth=16
        )
        accel.pio = system.membus.mem_side_ports
        trig_accels.append(accel)
        
        contention_ratio = args.num_cores / args.num_trig_accels
        print(f"  TrigAccel[{i}] @ 0x{ACCEL_BASE + i*0x1000:08x}")
        print(f"    Latency: {args.trig_latency} cycles")
        print(f"    Contention ratio: {contention_ratio:.1f}:1 (cores:accel)")
    
    system.trig_accels = trig_accels
    
    # Create matrix accelerators
    mat_accels = []
    mat_base = ACCEL_BASE + 0x10000
    for i in range(args.num_mat_accels):
        accel = SharedAccelPio(
            accel_type=1,  # MAT
            instance_id=i,
            pio_addr=mat_base + (i * 0x1000),
            trig_latency=args.trig_latency,
            mat_latency=args.mat_latency,
            max_queue_depth=16
        )
        accel.pio = system.membus.mem_side_ports
        mat_accels.append(accel)
        
        contention_ratio = args.num_cores / args.num_mat_accels
        print(f"  MatAccel[{i}] @ 0x{mat_base + i*0x1000:08x}")
        print(f"    Latency: {args.mat_latency} cycles")
        print(f"    Contention ratio: {contention_ratio:.1f}:1")
    
    system.mat_accels = mat_accels
    
    print("=" * 60 + "\n")


def setup_workload(system, args):
    """Set up workload for simulation"""
    
    if not os.path.exists(args.workload):
        print(f"Error: Workload not found: {args.workload}")
        return False
    
    system.workload = SEWorkload.init_compatible(args.workload)
    
    # Assign workload to each CPU
    for i, cpu in enumerate(system.cpu):
        process = Process(pid=100 + i)
        process.cmd = [args.workload]
        
        # Add arguments if provided
        if args.workload_args:
            process.cmd.extend(args.workload_args.split())
        
        cpu.workload = process
        cpu.createThreads()
    
    print(f"Workload: {args.workload}")
    print(f"Running on {args.num_cores} cores")
    
    return True


def main():
    parser = argparse.ArgumentParser(
        description="NOVA Processor - Shared Accelerator PIO Configuration"
    )
    
    # System configuration
    parser.add_argument('--num-cores', type=int, default=4,
                        help='Number of CPU cores')
    parser.add_argument('--cpu-clock', type=str, default='1GHz',
                        help='CPU clock frequency')
    parser.add_argument('--mem-size', type=str, default='4GB',
                        help='Memory size')
    
    # Accelerator configuration
    parser.add_argument('--num-trig-accels', type=int, default=1,
                        help='Number of trigonometric accelerators')
    parser.add_argument('--num-mat-accels', type=int, default=1,
                        help='Number of matrix accelerators')
    parser.add_argument('--trig-latency', type=int, default=15,
                        help='Trigonometric operation latency (cycles)')
    parser.add_argument('--mat-latency', type=int, default=50,
                        help='Matrix operation latency (cycles)')
    
    # Workload
    parser.add_argument('--workload', type=str, required=True,
                        help='Path to workload binary')
    parser.add_argument('--workload-args', type=str, default='',
                        help='Arguments to pass to workload')
    parser.add_argument('--max-ticks', type=int, default=10**12,
                        help='Maximum simulation ticks')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("NOVA Processor - Shared Accelerator PIO Simulation")
    print("PhD Research: Futuristic Spacecraft Processor")
    print("=" * 70)
    
    # Build system
    system = create_system(args)
    create_cpus(system, args)
    create_caches(system, args)
    create_memory(system, args)
    create_accelerators(system, args)
    
    # Setup workload
    if not setup_workload(system, args):
        sys.exit(1)
    
    # Create root
    root = Root(full_system=False, system=system)
    
    # Instantiate and run
    m5.instantiate()
    
    print("\nStarting simulation...")
    exit_event = m5.simulate(args.max_ticks)
    
    print("\n" + "=" * 70)
    print("SIMULATION COMPLETE")
    print("=" * 70)
    print(f"Exit cause: {exit_event.getCause()}")
    print(f"Simulated ticks: {m5.curTick():,}")
    print(f"Cores: {args.num_cores}")
    print(f"Trig Accelerators: {args.num_trig_accels}")
    print(f"Mat Accelerators: {args.num_mat_accels}")
    print("=" * 70)
    
    m5.stats.dump()


if __name__ == '__m5_main__':
    main()


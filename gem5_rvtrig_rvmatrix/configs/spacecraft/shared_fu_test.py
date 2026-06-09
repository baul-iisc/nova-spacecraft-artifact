#!/usr/bin/env python3
"""
NOVA Processor - Shared FU Test Configuration
PhD Research: Direct CPU-level shared FU contention modeling

This configuration tests the TimingSimpleCPU's built-in shared FU contention
tracking. When trig/matrix instructions are executed, the CPU checks if the
shared FU is available and tracks wait cycles.

Usage:
    # Shared mode: 4 cores, 1 trig accel (contention expected)
    ./build/RISCV/gem5.opt configs/spacecraft/shared_fu_test.py \
        --num-cores 4 --num-trig-accels 1 --workload ./workloads/trig_only_test.riscv

    # Dedicated mode: 4 cores, 4 trig accels (no contention)
    ./build/RISCV/gem5.opt configs/spacecraft/shared_fu_test.py \
        --num-cores 4 --num-trig-accels 4 --workload ./workloads/trig_only_test.riscv
"""

import argparse
import os
import sys

import m5
from m5.objects import *


def create_system(args):
    system = System()
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = args.cpu_clock
    system.clk_domain.voltage_domain = VoltageDomain()
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange(args.mem_size)]
    return system


def create_cpus(system, args):
    system.cpu = [TimingSimpleCPU(cpu_id=i) for i in range(args.num_cores)]
    print(f"Created {args.num_cores} TimingSimpleCPU cores with shared FU tracking")
    return system.cpu


def create_caches_and_memory(system, args):
    system.membus = SystemXBar()
    
    for cpu in system.cpu:
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports
        cpu.createInterruptController()
    
    system.system_port = system.membus.cpu_side_ports
    
    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR4_2400_16x4()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports


def setup_workload(system, args):
    if not os.path.exists(args.workload):
        print(f"Error: Workload not found: {args.workload}")
        return False
    
    system.workload = SEWorkload.init_compatible(args.workload)
    
    for i, cpu in enumerate(system.cpu):
        process = Process(pid=100 + i)
        process.cmd = [args.workload]
        cpu.workload = process
        cpu.createThreads()
    
    return True


def main():
    parser = argparse.ArgumentParser(description="NOVA Shared FU Test")
    parser.add_argument('--num-cores', type=int, default=4)
    parser.add_argument('--num-trig-accels', type=int, default=1)
    parser.add_argument('--num-mat-accels', type=int, default=1)
    parser.add_argument('--num-vpu-accels', type=int, default=1)
    parser.add_argument('--num-npu-accels', type=int, default=1)
    parser.add_argument('--trig-latency', type=int, default=15)
    parser.add_argument('--mat-latency', type=int, default=50)
    parser.add_argument('--vpu-latency', type=int, default=100)
    parser.add_argument('--npu-latency', type=int, default=200)
    parser.add_argument('--cpu-clock', type=str, default='1GHz')
    parser.add_argument('--mem-size', type=str, default='4GB')
    parser.add_argument('--workload', type=str, required=True)
    parser.add_argument('--max-ticks', type=int, default=0)  # 0 = no limit, run to completion
    args = parser.parse_args()
    
    # Set environment variables for C++ to read
    os.environ['NOVA_NUM_TRIG_ACCELS'] = str(args.num_trig_accels)
    os.environ['NOVA_NUM_MAT_ACCELS'] = str(args.num_mat_accels)
    os.environ['NOVA_NUM_VPU_ACCELS'] = str(args.num_vpu_accels)
    os.environ['NOVA_NUM_NPU_ACCELS'] = str(args.num_npu_accels)
    
    print("=" * 70)
    print("NOVA Processor - Shared FU Contention Test")
    print("=" * 70)
    print(f"Cores: {args.num_cores}")
    print(f"Trig Accelerators: {args.num_trig_accels} (latency={args.trig_latency})")
    print(f"Mat Accelerators: {args.num_mat_accels} (latency={args.mat_latency})")
    print(f"VPU Accelerators: {args.num_vpu_accels} (latency={args.vpu_latency})")
    print(f"NPU Accelerators: {args.num_npu_accels} (latency={args.npu_latency})")
    print(f"Contention ratio: {args.num_cores/args.num_trig_accels:.1f}:1")
    print("=" * 70)
    
    system = create_system(args)
    create_cpus(system, args)
    create_caches_and_memory(system, args)
    
    if not setup_workload(system, args):
        sys.exit(1)
    
    root = Root(full_system=False, system=system)
    m5.instantiate()
    
    # Configure shared FU parameters via the CPU's static method
    # This is called from C++ during simulation startup
    print(f"\nConfiguring shared FU: {args.num_trig_accels} TrigAccels, "
          f"{args.num_mat_accels} MatAccels, {args.num_vpu_accels} VPUs, "
          f"{args.num_npu_accels} NPUs")
    
    print("\nStarting simulation...")
    # Run until workload completes (no tick limit)
    if args.max_ticks > 0:
        exit_event = m5.simulate(args.max_ticks)
    else:
        exit_event = m5.simulate()  # Run to completion
    
    print("\n" + "=" * 70)
    print("SIMULATION COMPLETE")
    print("=" * 70)
    print(f"Exit cause: {exit_event.getCause()}")
    print(f"Simulated ticks: {m5.curTick():,}")
    print("=" * 70)
    
    m5.stats.dump()


if __name__ == '__m5_main__':
    main()


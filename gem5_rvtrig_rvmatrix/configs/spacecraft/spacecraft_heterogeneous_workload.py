"""
Spacecraft Heterogeneous Workload - Multi-Process SE Mode
PhD Research: Chandraboul

This configuration runs DIFFERENT workloads on each CPU to simulate
realistic spacecraft multicore processor scenarios where each core
handles a different task (navigation, telemetry, control, etc.)

Workloads are assigned in cyclic order:
  Core 0 -> Workload 0
  Core 1 -> Workload 1
  Core 2 -> Workload 2 (or Workload 0 if only 2 workloads)
  ...

This creates realistic contention patterns where diverse workloads
compete for shared accelerators.

Usage:
  ./gem5.opt configs/spacecraft/spacecraft_heterogeneous_workload.py \
      --binaries bin1.riscv,bin2.riscv,bin3.riscv \
      --num-cpus 4 --mode shared --caches --l2cache
"""

import argparse
import sys
import os

import m5
from m5.objects import *
from m5.util import addToPath

addToPath("../../")


# ============================================================================
# Cache Definitions (for reduced memory footprint)
# ============================================================================

class L1ICache(Cache):
    """L1 Instruction Cache - 32KB, 2-way set associative"""
    size = '32kB'
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

    def __init__(self, options=None):
        super(L1ICache, self).__init__()


class L1DCache(Cache):
    """L1 Data Cache - 32KB, 2-way set associative"""
    size = '32kB'
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

    def __init__(self, options=None):
        super(L1DCache, self).__init__()


class L2Cache(Cache):
    """L2 Cache - 256KB, 8-way set associative, shared"""
    size = '256kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

    def __init__(self, options=None):
        super(L2Cache, self).__init__()


def main():
    parser = argparse.ArgumentParser(description='Heterogeneous Workload Contention Study')
    parser.add_argument('--binaries', type=str, required=True,
                       help='Comma-separated list of binaries to run (cyclic assignment)')
    parser.add_argument('--num-cpus', type=int, default=4, help='Number of CPUs')
    parser.add_argument('--mode', type=str, default='shared',
                       choices=['dedicated', 'hybrid', 'shared'],
                       help='Accelerator sharing mode')
    parser.add_argument('--mem-size', type=str, default='512MB', help='Memory size')
    parser.add_argument('--cpu-type', type=str, default='TimingSimpleCPU',
                       choices=['TimingSimpleCPU', 'AtomicSimpleCPU'])
    parser.add_argument('--caches', action='store_true', default=True,
                       help='Enable L1 caches (default: enabled)')
    parser.add_argument('--no-caches', action='store_true',
                       help='Disable L1 caches (WARNING: high memory usage!)')
    parser.add_argument('--l2cache', action='store_true', default=True,
                       help='Enable shared L2 cache (default: enabled)')
    
    args = parser.parse_args()
    
    # Parse binaries list
    binaries = [b.strip() for b in args.binaries.split(',')]
    
    # Validate binaries exist
    for binary in binaries:
        if not os.path.exists(binary):
            print(f"ERROR: Binary not found: {binary}")
            sys.exit(1)
    
    # Handle --no-caches flag
    use_caches = args.caches and not args.no_caches
    use_l2cache = args.l2cache and use_caches
    
    # Calculate number of accelerators based on mode
    if args.mode == 'dedicated':
        num_accels = args.num_cpus
    elif args.mode == 'hybrid':
        num_accels = max(1, args.num_cpus // 2)
    else:  # shared
        num_accels = 1
    
    # Set environment variables for TimingSimpleCPU shared FU support
    os.environ['GEM5_SHARED_FU_ENABLE'] = '1'
    os.environ['GEM5_NUM_SHARED_ACCELS'] = str(num_accels)
    os.environ['GEM5_NUM_CPUS'] = str(args.num_cpus)
    os.environ['GEM5_SHARING_MODE'] = args.mode
    
    # Set individual accelerator counts
    os.environ['NOVA_NUM_TRIG_ACCELS'] = str(num_accels)
    os.environ['NOVA_NUM_MAT_ACCELS'] = str(num_accels)
    os.environ['NOVA_NUM_VPU_ACCELS'] = str(num_accels)
    os.environ['NOVA_NUM_NPU_ACCELS'] = str(num_accels)
    os.environ['NOVA_NUM_CPUS'] = str(args.num_cpus)
    
    # Assign workloads cyclically
    workload_assignment = []
    for i in range(args.num_cpus):
        workload_idx = i % len(binaries)
        workload_assignment.append((i, binaries[workload_idx]))
    
    print("=" * 70)
    print("HETEROGENEOUS WORKLOAD CONTENTION STUDY")
    print("=" * 70)
    print(f"CPUs: {args.num_cpus}")
    print(f"Mode: {args.mode}")
    print(f"Accelerators: {num_accels}")
    print(f"Sharing Ratio: {args.num_cpus / num_accels:.1f} CPUs per accelerator")
    print(f"Available Workloads: {len(binaries)}")
    print(f"CPU Type: {args.cpu_type}")
    print(f"L1 Caches: {'ENABLED (32KB I$ + 32KB D$)' if use_caches else 'DISABLED'}")
    print(f"L2 Cache: {'ENABLED (256KB shared)' if use_l2cache else 'DISABLED'}")
    print("-" * 70)
    print("Workload Assignment (Cyclic):")
    for cpu_id, binary in workload_assignment:
        workload_name = os.path.basename(binary).replace('.riscv', '')
        print(f"  CPU {cpu_id}: {workload_name}")
    print("=" * 70)
    
    # Create system
    system = System()
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = '1GHz'
    system.clk_domain.voltage_domain = VoltageDomain()
    
    system.mem_mode = 'timing' if args.cpu_type == 'TimingSimpleCPU' else 'atomic'
    system.mem_ranges = [AddrRange(args.mem_size)]
    
    # Create CPUs based on type
    if args.cpu_type == 'TimingSimpleCPU':
        CPUClass = TimingSimpleCPU
    else:
        CPUClass = AtomicSimpleCPU
    
    system.cpu = [CPUClass(cpu_id=i) for i in range(args.num_cpus)]
    
    # Memory bus
    system.membus = SystemXBar()
    
    # Create caches if enabled
    if use_caches:
        system.l1_icaches = [L1ICache() for _ in range(args.num_cpus)]
        system.l1_dcaches = [L1DCache() for _ in range(args.num_cpus)]
        
        if use_l2cache:
            system.l2bus = L2XBar()
            system.l2cache = L2Cache()
            system.l2cache.cpu_side = system.l2bus.mem_side_ports
            system.l2cache.mem_side = system.membus.cpu_side_ports
            
            for i, cpu in enumerate(system.cpu):
                system.l1_icaches[i].cpu_side = cpu.icache_port
                system.l1_dcaches[i].cpu_side = cpu.dcache_port
                system.l1_icaches[i].mem_side = system.l2bus.cpu_side_ports
                system.l1_dcaches[i].mem_side = system.l2bus.cpu_side_ports
                cpu.createInterruptController()
        else:
            for i, cpu in enumerate(system.cpu):
                system.l1_icaches[i].cpu_side = cpu.icache_port
                system.l1_dcaches[i].cpu_side = cpu.dcache_port
                system.l1_icaches[i].mem_side = system.membus.cpu_side_ports
                system.l1_dcaches[i].mem_side = system.membus.cpu_side_ports
                cpu.createInterruptController()
    else:
        print("\n*** WARNING: Running without caches - expect high memory usage! ***\n")
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
    
    # Use first binary for SEWorkload compatibility check
    system.workload = SEWorkload.init_compatible(binaries[0])
    
    # Create separate process for each CPU with DIFFERENT workloads
    for i, cpu in enumerate(system.cpu):
        workload_idx = i % len(binaries)
        binary = binaries[workload_idx]
        
        process = Process(pid=100 + i)
        process.cmd = [binary, str(i)]  # Pass CPU ID as argument
        cpu.workload = process
        cpu.createThreads()
    
    # Root object
    root = Root(full_system=False, system=system)
    
    # Instantiate
    m5.instantiate()
    
    print("\nStarting heterogeneous multi-process simulation...")
    print("Each CPU runs a DIFFERENT workload (cyclic assignment):")
    for cpu_id, binary in workload_assignment:
        print(f"  CPU {cpu_id}: {os.path.basename(binary)}")
    print(f"Mode: {args.mode} ({num_accels} accelerator(s) for {args.num_cpus} CPU(s))")
    print("=" * 70)
    
    # Run simulation
    exit_event = m5.simulate()
    
    print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")
    print(f"Simulation Mode: {args.mode}")
    print(f"Accelerator Contention: {args.num_cpus / num_accels:.1f}:1 CPU to accelerator ratio")
    print("Heterogeneous Workload Summary:")
    for cpu_id, binary in workload_assignment:
        print(f"  CPU {cpu_id}: {os.path.basename(binary)}")

if __name__ == "__m5_main__":
    main()

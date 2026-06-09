#!/usr/bin/env python3
# Copyright (c) 2024 Chandraboul - PhD Research
#
# Spacecraft Ruby NoC Experiment
# 
# This configuration combines:
# 1. Ruby memory system with Garnet NoC for detailed interconnect modeling
# 2. Space workload traffic generators (GNC patterns)
# 3. Fault injection for resilience testing
# 4. Shared vs Dedicated accelerator comparison

import argparse
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

# Add paths for gem5 libraries
addToPath('../')
addToPath('../../')

from common import Options, Simulation, CacheConfig, MemConfig, ObjectList
from common.Caches import *

# Import Ruby
from ruby import Ruby

# Check for Ruby/Garnet support
has_ruby = hasattr(m5.objects, 'RubySystem')
has_garnet = hasattr(m5.objects, 'GarnetNetwork')

# Accelerator MMIO addresses
SHARED_MATRIX_BASE = 0x42000000
SHARED_CORDIC_BASE = 0x42010000
DEDICATED_BASE = 0x42100000
COMPRESSION_BASE = 0x40050000
IMAGE_COMP_BASE = 0x40070000

def build_system(options):
    """Build the spacecraft system with Ruby NoC"""
    
    # Determine if using Ruby
    use_ruby = options.ruby and has_ruby
    
    # Create system
    system = System(
        clk_domain=SrcClockDomain(
            clock=options.sys_clock,
            voltage_domain=VoltageDomain()
        ),
        mem_mode='timing',
        mem_ranges=[AddrRange(start=0x80000000, size=options.mem_size)],
    )
    
    # Create CPUs
    CPUClass = ObjectList.cpu_list.get(options.cpu_type)
    system.cpu = [CPUClass(cpu_id=i) for i in range(options.num_cpus)]
    
    for cpu in system.cpu:
        cpu.clk_domain = SrcClockDomain(
            clock=options.cpu_clock,
            voltage_domain=system.clk_domain.voltage_domain
        )
    
    if use_ruby:
        # Use Ruby memory system with Garnet NoC
        Ruby.create_system(options, False, system, None, None)
        
        # Connect CPUs to Ruby
        for i, cpu in enumerate(system.cpu):
            cpu.icache_port = system.ruby._cpu_ports[i].in_ports
            cpu.dcache_port = system.ruby._cpu_ports[i].in_ports
            
            cpu.createInterruptController()
            if hasattr(cpu, 'interrupts'):
                for interrupt in cpu.interrupts:
                    interrupt.pio = system.ruby._cpu_ports[i].in_ports
                    interrupt.int_requestor = system.ruby._cpu_ports[i].in_ports
                    interrupt.int_responder = system.ruby._cpu_ports[i].in_ports
    else:
        # Fallback to classic memory system
        system.membus = SystemXBar()
        
        for cpu in system.cpu:
            cpu.icache = L1_ICache(size='16kB', assoc=4)
            cpu.dcache = L1_DCache(size='16kB', assoc=4)
            
            cpu.icache_port = cpu.icache.cpu_side
            cpu.dcache_port = cpu.dcache.cpu_side
            
            cpu.icache.mem_side = system.membus.cpu_side_ports
            cpu.dcache.mem_side = system.membus.cpu_side_ports
            
            cpu.createInterruptController()
        
        # Memory controller
        system.mem_ctrl = MemCtrl()
        system.mem_ctrl.dram = DDR4_2400_16x4()
        system.mem_ctrl.dram.range = system.mem_ranges[0]
        system.mem_ctrl.port = system.membus.mem_side_ports
        
        system.system_port = system.membus.cpu_side_ports
    
    return system


def add_accelerators(system, options):
    """Add shared and dedicated accelerators"""
    np = options.num_cpus
    accel_mode = options.accel_mode
    
    # Create accelerator bus
    system.accel_bus = NoncoherentXBar(
        frontend_latency=1,
        forward_latency=0,
        response_latency=1,
        width=64
    )
    
    # Bridge from main memory bus
    if hasattr(system, 'membus'):
        system.accel_bridge = Bridge(delay='10ns')
        system.accel_bridge.ranges = [
            AddrRange(0x40000000, 0x50000000)  # All accelerator space
        ]
        system.accel_bridge.cpu_side_port = system.membus.mem_side_ports
        system.accel_bridge.mem_side_port = system.accel_bus.cpu_side_ports
    
    if accel_mode == 'shared':
        # Create shared accelerators
        from m5.objects import SharedMatrixAccel, SharedCORDICAccel
        
        system.shared_matrix = SharedMatrixAccel(
            mmio_base=SHARED_MATRIX_BASE,
            mmio_size='4kB',
            num_cores=np,
            compute_latency=10,
            arbitration_latency=2
        )
        system.shared_matrix.mmio_port = system.accel_bus.cpu_side_ports
        
        system.shared_cordic = SharedCORDICAccel(
            mmio_base=SHARED_CORDIC_BASE,
            mmio_size='4kB',
            num_cores=np,
            compute_latency=5,
            arbitration_latency=1
        )
        system.shared_cordic.mmio_port = system.accel_bus.cpu_side_ports
        
        print(f"[Config] Shared accelerators: Matrix@0x{SHARED_MATRIX_BASE:x}, "
              f"CORDIC@0x{SHARED_CORDIC_BASE:x}")
    
    elif accel_mode == 'dedicated':
        # Create dedicated accelerators per core
        from m5.objects import SharedMatrixAccel, SharedCORDICAccel
        
        system.dedicated_matrix = []
        system.dedicated_cordic = []
        
        for i in range(np):
            matrix_base = DEDICATED_BASE + (i * 0x20000)
            cordic_base = DEDICATED_BASE + (i * 0x20000) + 0x10000
            
            matrix = SharedMatrixAccel(
                mmio_base=matrix_base,
                mmio_size='4kB',
                num_cores=1,
                compute_latency=10,
                arbitration_latency=0
            )
            matrix.mmio_port = system.accel_bus.cpu_side_ports
            system.dedicated_matrix.append(matrix)
            
            cordic = SharedCORDICAccel(
                mmio_base=cordic_base,
                mmio_size='4kB',
                num_cores=1,
                compute_latency=5,
                arbitration_latency=0
            )
            cordic.mmio_port = system.accel_bus.cpu_side_ports
            system.dedicated_cordic.append(cordic)
            
            print(f"[Config] Core {i}: Matrix@0x{matrix_base:x}, "
                  f"CORDIC@0x{cordic_base:x}")


def add_traffic_generator(system, options):
    """Add space workload traffic generator"""
    if not options.enable_traffic_gen:
        return
    
    from m5.objects import SpaceTrafficGen
    
    # Workload type mapping
    workload_map = {
        'attitude': 0,
        'orbit': 1,
        'navigation': 2,
        'image': 3,
        'compress': 4,
        'telemetry': 5,
        'command': 6,
        'mixed': 7,
        'stress': 8
    }
    
    workload_type = workload_map.get(options.traffic_workload, 7)
    
    system.traffic_gen = SpaceTrafficGen(
        workload_type=workload_type,
        injection_rate=options.injection_rate,
        num_cores=options.num_cpus,
        enable_deadlines=options.enable_deadlines,
        matrix_accel_base=SHARED_MATRIX_BASE,
        cordic_accel_base=SHARED_CORDIC_BASE,
        compression_base=COMPRESSION_BASE,
        image_comp_base=IMAGE_COMP_BASE,
    )
    
    if hasattr(system, 'accel_bus'):
        system.traffic_gen.traffic_port = system.accel_bus.cpu_side_ports
    
    print(f"[Config] Traffic generator: workload={options.traffic_workload}, "
          f"rate={options.injection_rate} req/us")


def add_fault_injector(system, options):
    """Add fault injection framework"""
    if not options.enable_fault_injection:
        return
    
    from m5.objects import FaultInjector
    
    system.fault_injector = FaultInjector(
        fault_rate=options.fault_rate,
        enable_random_faults=options.random_faults,
        max_faults=options.max_faults,
        watchdog_timeout=options.watchdog_timeout,
        enable_recovery=options.enable_recovery,
    )
    
    print(f"[Config] Fault injector: rate={options.fault_rate}/Mcycles, "
          f"random={options.random_faults}, max={options.max_faults}")


def main():
    parser = argparse.ArgumentParser(
        description="Spacecraft Ruby NoC Experiment"
    )
    
    # Basic options
    parser.add_argument("--num-cpus", type=int, default=4,
                        help="Number of CPU cores")
    parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                        help="CPU type")
    parser.add_argument("--cpu-clock", type=str, default="1GHz",
                        help="CPU clock frequency")
    parser.add_argument("--sys-clock", type=str, default="1GHz",
                        help="System clock frequency")
    parser.add_argument("--mem-size", type=str, default="512MB",
                        help="Memory size")
    
    # Ruby/Garnet options
    parser.add_argument("--ruby", action="store_true",
                        help="Use Ruby memory system")
    parser.add_argument("--network", type=str, default="garnet",
                        choices=["simple", "garnet"],
                        help="Network type for Ruby")
    parser.add_argument("--topology", type=str, default="Mesh_XY",
                        help="Network topology")
    parser.add_argument("--mesh-rows", type=int, default=2,
                        help="Mesh rows")
    
    # Accelerator options
    parser.add_argument("--accel-mode", type=str, default="shared",
                        choices=["shared", "dedicated", "none"],
                        help="Accelerator mode")
    
    # Traffic generator options
    parser.add_argument("--enable-traffic-gen", action="store_true",
                        help="Enable traffic generator")
    parser.add_argument("--traffic-workload", type=str, default="mixed",
                        choices=["attitude", "orbit", "navigation", "image",
                                "compress", "telemetry", "command", "mixed", "stress"],
                        help="Workload type")
    parser.add_argument("--injection-rate", type=float, default=1.0,
                        help="Injection rate (requests/us)")
    parser.add_argument("--enable-deadlines", action="store_true",
                        help="Enable real-time deadline checking")
    
    # Fault injection options
    parser.add_argument("--enable-fault-injection", action="store_true",
                        help="Enable fault injection")
    parser.add_argument("--fault-rate", type=float, default=0.1,
                        help="Fault rate (faults/million cycles)")
    parser.add_argument("--random-faults", action="store_true",
                        help="Enable random fault injection")
    parser.add_argument("--max-faults", type=int, default=-1,
                        help="Maximum faults to inject")
    parser.add_argument("--watchdog-timeout", type=int, default=1000000,
                        help="Watchdog timeout cycles")
    parser.add_argument("--enable-recovery", action="store_true",
                        help="Enable automatic fault recovery")
    
    # Workload
    parser.add_argument("--cmd", type=str, default="",
                        help="Binary to run")
    parser.add_argument("--options", type=str, default="",
                        help="Options for binary")
    
    # Add Ruby options
    if has_ruby:
        Ruby.define_options(parser)
    
    Options.addNoISAOptions(parser)
    
    args = parser.parse_args()
    
    # Build system
    print("=" * 60)
    print("SPACECRAFT RUBY NOC EXPERIMENT")
    print("=" * 60)
    print(f"CPUs: {args.num_cpus}, Type: {args.cpu_type}")
    print(f"Ruby: {args.ruby}, Network: {args.network}")
    print(f"Accelerator Mode: {args.accel_mode}")
    print("=" * 60)
    
    system = build_system(args)
    
    # Add accelerators
    if args.accel_mode != 'none':
        add_accelerators(system, args)
    
    # Add traffic generator
    add_traffic_generator(system, args)
    
    # Add fault injector
    add_fault_injector(system, args)
    
    # Create workload
    if args.cmd:
        process = Process(
            cmd=[args.cmd] + args.options.split() if args.options else [args.cmd]
        )
        for cpu in system.cpu:
            cpu.workload = process
            cpu.createThreads()
    
    # Create root
    root = Root(full_system=False, system=system)
    
    # Instantiate
    m5.instantiate()
    
    print("\nStarting simulation...")
    exit_event = m5.simulate()
    
    print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")
    
    # Print statistics summary
    print("\n" + "=" * 60)
    print("EXPERIMENT RESULTS SUMMARY")
    print("=" * 60)
    
    if args.ruby and has_ruby:
        print("\nRuby Network Statistics available in stats.txt")
    
    if args.enable_traffic_gen:
        print("\nTraffic Generator Statistics available in stats.txt")
    
    if args.enable_fault_injection:
        print("\nFault Injection Statistics available in stats.txt")


if __name__ == "__main__":
    main()



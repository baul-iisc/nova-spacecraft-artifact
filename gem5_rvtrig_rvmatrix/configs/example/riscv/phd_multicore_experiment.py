#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PhD Research: Multicore Spacecraft Processor Experiment Configuration

This configuration implements the full PhD research framework including:
- Adaptive Accelerator Allocation (Research Q1)
- Congestion-Aware Scheduling (Research Q2)
- Radiation-Hardened Architecture (Research Q3)
- Heterogeneous Accelerator Network (Research Q4)
- Energy-Proportional Design (Research Q5)
- Space Benchmark Suite (Research Q7)
- Mission-Aware Metrics (Research Q8)

Author: Chandraboul
"""

import argparse
import os
import sys

import m5
from m5.objects import *
from m5.util import addToPath

# Add gem5 paths
addToPath('../../')
addToPath('../../../')

from common import Options
from common import Simulation
from common import CacheConfig
from common import ObjectList


def create_adaptive_allocator(options):
    """Create the Adaptive Accelerator Allocator (Research Q1)"""
    from m5.objects import AdaptiveAllocator
    
    allocator = AdaptiveAllocator()
    allocator.num_cores = options.num_cpus
    allocator.num_accelerators_per_type = max(1, options.num_cpus // 2)
    allocator.evaluation_interval = options.alloc_eval_interval
    allocator.contention_threshold = options.contention_threshold
    allocator.hysteresis_margin = 0.1
    allocator.enable_adaptive = options.enable_adaptive
    
    return allocator


def create_congestion_scheduler(options):
    """Create the Congestion-Aware Scheduler (Research Q2)"""
    from m5.objects import CongestionScheduler
    
    scheduler = CongestionScheduler()
    scheduler.num_accelerators = options.num_cpus * 2  # Matrix + CORDIC
    scheduler.congestion_threshold = 0.7
    scheduler.max_queue_depth = 64
    scheduler.starvation_timeout = 100000
    scheduler.enable_preemption = options.enable_preemption
    scheduler.enable_deadline_scheduling = options.enable_deadlines
    
    return scheduler


def create_radiation_hardened(options):
    """Create Radiation-Hardened Accelerator (Research Q3)"""
    from m5.objects import RadiationHardenedAccel
    
    rad_hard = RadiationHardenedAccel()
    rad_hard.num_lanes = options.tmr_lanes
    rad_hard.redundancy_mode = options.redundancy_mode
    rad_hard.seu_probability = options.seu_probability
    rad_hard.mbu_probability = options.mbu_probability
    rad_hard.tmr_voting_latency = 2
    rad_hard.reduced_precision_bits = 32
    rad_hard.enable_checkpointing = True
    
    return rad_hard


def create_heterogeneous_network(options):
    """Create Heterogeneous Accelerator Network (Research Q4)"""
    from m5.objects import HeterogeneousAccelNetwork
    
    het_net = HeterogeneousAccelNetwork()
    het_net.num_hp_matrix = options.num_hp_matrix
    het_net.num_lp_matrix = options.num_lp_matrix
    het_net.num_hp_trig = options.num_hp_trig
    het_net.num_lp_trig = options.num_lp_trig
    het_net.total_power_budget = options.power_budget
    het_net.enable_dynamic_routing = True
    
    return het_net


def create_energy_manager(options):
    """Create Energy-Proportional Manager (Research Q5)"""
    from m5.objects import EnergyProportionalManager
    
    energy_mgr = EnergyProportionalManager()
    energy_mgr.total_power_budget = options.power_budget
    energy_mgr.base_power_consumption = 2.0
    energy_mgr.solar_panel_capacity = options.solar_capacity
    energy_mgr.battery_capacity = options.battery_capacity
    energy_mgr.thermal_limit = 85.0
    energy_mgr.enable_dvfs = options.enable_dvfs
    energy_mgr.enable_power_gating = options.enable_power_gating
    energy_mgr.dvfs_transition_cycles = 100
    
    return energy_mgr


def create_benchmark_suite(options):
    """Create Space Benchmark Suite (Research Q7)"""
    from m5.objects import SpaceBenchmark
    
    benchmark = SpaceBenchmark()
    benchmark.default_iterations = options.benchmark_iterations
    benchmark.default_data_size = options.benchmark_data_size
    benchmark.enable_validation = True
    benchmark.use_hardware_accel = options.use_hw_accel
    
    return benchmark


def create_mission_metrics(options):
    """Create Mission-Aware Metrics (Research Q8)"""
    from m5.objects import MissionMetrics
    
    metrics = MissionMetrics()
    metrics.mission_name = options.mission_name
    metrics.target_ops_per_joule = options.target_ops_per_joule
    metrics.target_mtbf = options.target_mtbf
    metrics.max_deadline_miss_rate = options.max_deadline_miss_rate
    metrics.nominal_temperature = 25.0
    
    return metrics


def create_system(options):
    """Create the full system configuration"""
    
    # Create base system
    system = System()
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = options.sys_clock
    system.clk_domain.voltage_domain = VoltageDomain()
    
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange(options.mem_size)]
    
    # Create CPUs
    system.cpu = [TimingSimpleCPU(cpu_id=i) for i in range(options.num_cpus)]
    
    for cpu in system.cpu:
        cpu.clk_domain = SrcClockDomain()
        cpu.clk_domain.clock = options.cpu_clock
        cpu.clk_domain.voltage_domain = VoltageDomain()
    
    # Create memory bus
    system.membus = SystemXBar()
    
    # Connect CPUs to memory bus
    for cpu in system.cpu:
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports
    
    # Create memory controller
    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR4_2400_8x8()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports
    
    # Create interrupt controllers
    for cpu in system.cpu:
        cpu.createInterruptController()
    
    # System port
    system.system_port = system.membus.cpu_side_ports
    
    # ==========================================================================
    # PhD Research Components
    # ==========================================================================
    
    # Research Q1: Adaptive Allocator
    system.adaptive_allocator = create_adaptive_allocator(options)
    
    # Research Q2: Congestion Scheduler
    system.congestion_scheduler = create_congestion_scheduler(options)
    
    # Research Q3: Radiation-Hardened Accelerator
    system.rad_hard_accel = create_radiation_hardened(options)
    
    # Research Q4: Heterogeneous Accelerator Network
    system.het_accel_network = create_heterogeneous_network(options)
    
    # Research Q5: Energy-Proportional Manager
    system.energy_manager = create_energy_manager(options)
    
    # Research Q7: Space Benchmark Suite
    system.space_benchmark = create_benchmark_suite(options)
    
    # Research Q8: Mission Metrics
    system.mission_metrics = create_mission_metrics(options)
    
    return system


def add_phd_options(parser):
    """Add PhD research-specific command line options"""
    
    # General options
    parser.add_argument("--num-cpus", type=int, default=4,
                       help="Number of CPU cores")
    parser.add_argument("--cpu-clock", type=str, default="400MHz",
                       help="CPU clock frequency")
    parser.add_argument("--sys-clock", type=str, default="400MHz",
                       help="System clock frequency")
    parser.add_argument("--mem-size", type=str, default="512MB",
                       help="Memory size")
    
    # Research Q1: Adaptive Allocation
    parser.add_argument("--enable-adaptive", action="store_true", default=True,
                       help="Enable adaptive accelerator allocation")
    parser.add_argument("--alloc-eval-interval", type=int, default=10000,
                       help="Allocation evaluation interval (cycles)")
    parser.add_argument("--contention-threshold", type=float, default=0.3,
                       help="Contention threshold for mode switching")
    
    # Research Q2: Congestion Scheduling
    parser.add_argument("--enable-preemption", action="store_true", default=False,
                       help="Enable request preemption")
    parser.add_argument("--enable-deadlines", action="store_true", default=True,
                       help="Enable deadline-aware scheduling")
    
    # Research Q3: Radiation Hardening
    parser.add_argument("--tmr-lanes", type=int, default=4,
                       help="Number of TMR compute lanes")
    parser.add_argument("--redundancy-mode", type=int, default=2,
                       help="Redundancy mode (0=none, 1=DMR, 2=TMR)")
    parser.add_argument("--seu-probability", type=float, default=1e-6,
                       help="SEU probability per operation")
    parser.add_argument("--mbu-probability", type=float, default=1e-8,
                       help="MBU probability per operation")
    
    # Research Q4: Heterogeneous Network
    parser.add_argument("--num-hp-matrix", type=int, default=1,
                       help="Number of high-precision matrix units")
    parser.add_argument("--num-lp-matrix", type=int, default=2,
                       help="Number of low-precision matrix units")
    parser.add_argument("--num-hp-trig", type=int, default=1,
                       help="Number of high-precision trig units")
    parser.add_argument("--num-lp-trig", type=int, default=2,
                       help="Number of low-precision trig units")
    
    # Research Q5: Energy Proportional
    parser.add_argument("--power-budget", type=float, default=15.0,
                       help="Total power budget (Watts)")
    parser.add_argument("--solar-capacity", type=float, default=20.0,
                       help="Solar panel capacity (Watts)")
    parser.add_argument("--battery-capacity", type=float, default=100.0,
                       help="Battery capacity (Watt-hours)")
    parser.add_argument("--enable-dvfs", action="store_true", default=True,
                       help="Enable DVFS")
    parser.add_argument("--enable-power-gating", action="store_true", default=True,
                       help="Enable power gating")
    
    # Research Q7: Benchmarks
    parser.add_argument("--benchmark-iterations", type=int, default=100,
                       help="Benchmark iterations")
    parser.add_argument("--benchmark-data-size", type=int, default=256,
                       help="Benchmark data size")
    parser.add_argument("--use-hw-accel", action="store_true", default=True,
                       help="Use hardware accelerators in benchmarks")
    
    # Research Q8: Metrics
    parser.add_argument("--mission-name", type=str, default="spacecraft",
                       help="Mission name for metrics")
    parser.add_argument("--target-ops-per-joule", type=float, default=1e9,
                       help="Target operations per Joule")
    parser.add_argument("--target-mtbf", type=float, default=86400.0,
                       help="Target MTBF in seconds")
    parser.add_argument("--max-deadline-miss-rate", type=float, default=0.001,
                       help="Maximum deadline miss rate")
    
    # Experiment selection
    parser.add_argument("--experiment", type=str, default="scaling",
                       choices=["scaling", "contention", "radiation", 
                               "energy", "benchmark", "full"],
                       help="Experiment type to run")
    
    # Workload
    parser.add_argument("--workload", type=str, default=None,
                       help="Path to workload binary")


def main():
    """Main entry point"""
    
    parser = argparse.ArgumentParser(description="PhD Multicore Experiment")
    add_phd_options(parser)
    options = parser.parse_args()
    
    # Create system
    root = Root(full_system=False)
    root.system = create_system(options)
    
    # Set up workload if provided
    if options.workload:
        process = Process(pid=100)
        process.executable = options.workload
        process.cmd = [options.workload]
        
        for i, cpu in enumerate(root.system.cpu):
            cpu.workload = process
            cpu.createThreads()
    
    # Instantiate
    m5.instantiate()
    
    print("=" * 60)
    print("PhD Research: Multicore Spacecraft Processor Experiment")
    print("=" * 60)
    print(f"Experiment: {options.experiment}")
    print(f"CPUs: {options.num_cpus}")
    print(f"CPU Clock: {options.cpu_clock}")
    print(f"Memory: {options.mem_size}")
    print(f"Power Budget: {options.power_budget} W")
    print(f"TMR Mode: {options.redundancy_mode}")
    print("=" * 60)
    
    # Run simulation
    print("Beginning simulation...")
    exit_event = m5.simulate()
    
    print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
    
    # Generate report
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
"""
Comprehensive PhD Study: Dedicated vs Hybrid vs Fully Shared Accelerators
Author: Chandraboul
PhD Research - Spacecraft Processor Architecture

This configuration supports three accelerator allocation strategies:
1. DEDICATED: Each core has its own accelerators (no contention, max area)
2. HYBRID: Critical accelerators dedicated, others shared
3. FULLY_SHARED: All accelerators shared among cores (min area, max contention)

Accelerators modeled:
- Matrix Tile Accelerator (3x3 systolic array for GNC matrix ops)
- CORDIC Trigonometric Accelerator (for attitude determination)
- TinyTPU (for ML inference - anomaly detection, FDIR)
- Compression Core (CCSDS compression)
- TinyML Accelerator (lightweight NN processing)

Metrics collected:
- Performance: cycles, throughput, latency
- Power: energy consumption, power states
- Resource: area utilization, contention overhead
- Reliability: fault tolerance (optional)
"""

import argparse
import os
import sys
import json
from os import path

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn
from m5.util.fdthelper import *

addToPath("../../")
addToPath("../../../")

from common import (
    CacheConfig,
    CpuConfig,
    MemConfig,
    ObjectList,
    Options,
    Simulation,
)
from common.Caches import *
from common.FSConfig import *
from common.SysPaths import *

# =============================================================================
# Check Available Accelerators
# =============================================================================

ACCELERATORS = {}

accelerator_imports = [
    ('MatrixTileAccel', 'Matrix Tile Accelerator (3x3 systolic)'),
    ('TinyTPU', 'TinyTPU Systolic Array'),
    ('SharedMatrixAccel', 'Shared Matrix Accelerator'),
    ('SharedCORDICAccel', 'Shared CORDIC Accelerator'),
    ('TinyMLAccel', 'TinyML Accelerator'),
    ('CompressionCore', 'CCSDS Compression Core'),
    ('EnergyProportionalManager', 'Energy/Power Manager'),
    ('FaultInjector', 'Fault Injection Framework'),
    ('SpaceTrafficGen', 'GNC Traffic Generator'),
    ('HeterogeneousAccelNetwork', 'Heterogeneous Accelerator Network'),
    ('MissionMetrics', 'Mission Performance Metrics'),
]

for accel_name, description in accelerator_imports:
    try:
        ACCELERATORS[accel_name] = getattr(m5.objects, accel_name)
        print(f"✓ {description}: Available")
    except AttributeError:
        ACCELERATORS[accel_name] = None
        print(f"✗ {description}: Not available")

# =============================================================================
# Memory Map
# =============================================================================

MMIO_BASE = 0x40000000

# Shared mode addresses
SHARED_MATRIX_BASE      = MMIO_BASE + 0x100000
SHARED_CORDIC_BASE      = MMIO_BASE + 0x110000
SHARED_TPU_BASE         = MMIO_BASE + 0x120000
SHARED_ML_BASE          = MMIO_BASE + 0x130000
SHARED_COMPRESS_BASE    = MMIO_BASE + 0x140000

# Per-core dedicated address spacing
DEDICATED_CORE_SPACING = 0x100000

def get_dedicated_addr(core_id, accel_offset):
    """Calculate dedicated accelerator address for a core"""
    return MMIO_BASE + 0x200000 + (core_id * DEDICATED_CORE_SPACING) + accel_offset

# Accelerator area cost (relative units)
AREA_COST = {
    'matrix': 1.0,      # Reference unit
    'cordic': 0.3,      # 30% of matrix
    'tpu': 2.5,         # 2.5x matrix
    'ml': 1.5,          # 1.5x matrix
    'compress': 0.8,    # 80% of matrix
}

# Accelerator power cost (Watts)
POWER_COST = {
    'matrix': 0.5,
    'cordic': 0.15,
    'tpu': 1.2,
    'ml': 0.8,
    'compress': 0.3,
}

# =============================================================================
# Configuration Classes
# =============================================================================

class AcceleratorConfig:
    """Base configuration for an accelerator"""
    def __init__(self, name, addr, size='4kB', latency=10, is_shared=False):
        self.name = name
        self.addr = addr
        self.size = size
        self.latency = latency
        self.is_shared = is_shared

class AllocationStrategy:
    """Defines accelerator allocation strategy"""
    DEDICATED = 'dedicated'
    HYBRID = 'hybrid'
    FULLY_SHARED = 'fully_shared'

# =============================================================================
# Accelerator Factory
# =============================================================================

def create_matrix_accel(addr, num_cores=1, shared=False, size='8kB'):
    """Create a Matrix Tile Accelerator"""
    if ACCELERATORS['MatrixTileAccel']:
        return MatrixTileAccel(
            addr_range=AddrRange(addr, size=size),
            input_buffer_size='8kB' if shared else '4kB',
            weight_buffer_size='8kB' if shared else '4kB',
            accum_buffer_size='4kB' if shared else '2kB',
            output_buffer_size='4kB' if shared else '2kB',
            mac_latency=3,
            dma_latency=10,
        )
    elif ACCELERATORS['SharedMatrixAccel']:
        return SharedMatrixAccel(
            mmio_base=addr,
            mmio_size=size,
            num_cores=num_cores,
            compute_latency=10,
            arbitration_latency=2 if shared else 0,
        )
    return None

def create_cordic_accel(addr, num_cores=1, shared=False, size='4kB'):
    """Create a CORDIC Trigonometric Accelerator"""
    if ACCELERATORS['SharedCORDICAccel']:
        return SharedCORDICAccel(
            mmio_base=addr,
            mmio_size=size,
            num_cores=num_cores,
            compute_latency=5,
            arbitration_latency=1 if shared else 0,
        )
    return None

def create_tpu_accel(addr, shared=False, size='8kB'):
    """Create TinyTPU accelerator"""
    if ACCELERATORS['TinyTPU']:
        return TinyTPU(
            addr_range=AddrRange(addr, size=size),
            unified_buffer_size='128kB' if shared else '64kB',
            weight_fifo_size='64kB' if shared else '32kB',
            accum_buffer_size='8kB' if shared else '4kB',
            mxu_latency=1,
            dma_latency=5,
        )
    return None

def create_ml_accel(addr, shared=False, size='4kB'):
    """Create TinyML accelerator"""
    if ACCELERATORS['TinyMLAccel']:
        return TinyMLAccel(
            pio_addr=addr,
            pio_size=size,
        )
    return None

def create_compression_accel(addr, shared=False, size='4kB'):
    """Create CCSDS Compression Core"""
    if ACCELERATORS['CompressionCore']:
        return CompressionCore(
            pio_addr=addr,
            pio_size=size,
        )
    return None

# =============================================================================
# System Builder
# =============================================================================

def build_spacecraft_system(args):
    """Build the spacecraft system with specified accelerator configuration"""
    
    np = args.num_cpus
    mode = args.allocation_mode
    
    print("\n" + "=" * 80)
    print("SPACECRAFT PHD COMPREHENSIVE STUDY")
    print("=" * 80)
    print(f"Allocation Mode:    {mode.upper()}")
    print(f"Number of Cores:    {np}")
    print(f"CPU Type:           {args.cpu_type}")
    print("=" * 80)
    
    # Determine simulation mode
    if args.full_system:
        return build_fs_system(args)
    else:
        return build_se_system(args)

def build_se_system(args):
    """Build System Emulation mode system"""
    np = args.num_cpus
    mode = args.allocation_mode
    
    # Create system
    system = System()
    system.clk_domain = SrcClockDomain(
        clock=args.sys_clock,
        voltage_domain=VoltageDomain(voltage=args.sys_voltage)
    )
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange(args.mem_size)]
    
    # Create CPUs
    CPUClass = ObjectList.cpu_list.get(args.cpu_type)
    system.cpu = [CPUClass(cpu_id=i) for i in range(np)]
    
    for cpu in system.cpu:
        cpu.clk_domain = SrcClockDomain(
            clock=args.cpu_clock,
            voltage_domain=system.clk_domain.voltage_domain
        )
    
    # Create memory bus
    system.membus = SystemXBar()
    system.system_port = system.membus.cpu_side_ports
    
    # Create accelerator bus
    system.accel_bus = IOXBar(
        frontend_latency=1,
        forward_latency=0,
        response_latency=1,
        width=64
    )
    
    # Bridge accelerator bus
    system.accel_bridge = Bridge(delay='10ns')
    system.accel_bridge.ranges = [AddrRange(MMIO_BASE, size='256MB')]
    system.accel_bridge.cpu_side_port = system.membus.mem_side_ports
    system.accel_bridge.mem_side_port = system.accel_bus.cpu_side_ports
    
    # Configure accelerators based on mode
    accel_info = configure_accelerators(system, np, mode, args)
    
    # Setup caches and memory
    for i, cpu in enumerate(system.cpu):
        # L1 caches
        cpu.icache = L1_ICache(size='32kB', assoc=4)
        cpu.dcache = L1_DCache(size='32kB', assoc=4)
        
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
    
    # Add power manager if available
    if args.enable_power_model and ACCELERATORS['EnergyProportionalManager']:
        add_power_manager(system, args)
    
    # Add fault injector if enabled
    if args.enable_fault_injection and ACCELERATORS['FaultInjector']:
        add_fault_injector(system, args)
    
    # Add traffic generator if enabled
    if args.enable_traffic_gen and ACCELERATORS['SpaceTrafficGen']:
        add_traffic_generator(system, args)
    
    # Add mission metrics if available
    if args.enable_metrics and ACCELERATORS['MissionMetrics']:
        add_mission_metrics(system, args)
    
    # Store configuration info for analysis
    system._accel_info = accel_info
    
    return system, accel_info

def build_fs_system(args):
    """Build Full System mode system (for kernel workloads)"""
    # For now, delegate to SE mode with a warning
    print("Warning: FS mode support is experimental. Using SE mode.")
    return build_se_system(args)

# =============================================================================
# Accelerator Configuration
# =============================================================================

def configure_accelerators(system, np, mode, args):
    """Configure accelerators based on allocation mode"""
    
    accel_info = {
        'mode': mode,
        'num_cores': np,
        'accelerators': [],
        'total_area': 0.0,
        'total_power': 0.0,
        'shared_count': 0,
        'dedicated_count': 0,
    }
    
    if mode == AllocationStrategy.DEDICATED:
        configure_dedicated(system, np, args, accel_info)
    elif mode == AllocationStrategy.HYBRID:
        configure_hybrid(system, np, args, accel_info)
    elif mode == AllocationStrategy.FULLY_SHARED:
        configure_fully_shared(system, np, args, accel_info)
    
    print("\n" + "-" * 80)
    print("ACCELERATOR CONFIGURATION SUMMARY")
    print("-" * 80)
    print(f"Total Accelerator Instances: {len(accel_info['accelerators'])}")
    print(f"Shared Accelerators:         {accel_info['shared_count']}")
    print(f"Dedicated Accelerators:      {accel_info['dedicated_count']}")
    print(f"Total Area (relative):       {accel_info['total_area']:.2f}")
    print(f"Max Power (Watts):           {accel_info['total_power']:.2f}")
    print("-" * 80)
    
    return accel_info

def configure_dedicated(system, np, args, accel_info):
    """Configure DEDICATED mode - each core has its own accelerators"""
    print("\n>>> DEDICATED MODE: Each core has dedicated accelerator instances")
    
    system.dedicated_matrix = []
    system.dedicated_cordic = []
    system.dedicated_tpu = []
    system.dedicated_ml = []
    system.dedicated_compress = []
    
    for core_id in range(np):
        print(f"\n  Core {core_id} Accelerators:")
        
        # Matrix accelerator for each core
        if args.enable_matrix:
            matrix_addr = get_dedicated_addr(core_id, 0x0000)
            matrix = create_matrix_accel(matrix_addr, num_cores=1, shared=False)
            if matrix:
                matrix.cpu_side = system.accel_bus.mem_side_ports
                if hasattr(matrix, 'mem_side'):
                    matrix.mem_side = system.membus.cpu_side_ports
                system.dedicated_matrix.append(matrix)
                accel_info['accelerators'].append({
                    'name': f'matrix_core{core_id}',
                    'type': 'matrix',
                    'core': core_id,
                    'addr': matrix_addr,
                    'shared': False
                })
                accel_info['total_area'] += AREA_COST['matrix']
                accel_info['total_power'] += POWER_COST['matrix']
                accel_info['dedicated_count'] += 1
                print(f"    ✓ Matrix @ 0x{matrix_addr:08x}")
        
        # CORDIC accelerator for each core
        if args.enable_cordic:
            cordic_addr = get_dedicated_addr(core_id, 0x10000)
            cordic = create_cordic_accel(cordic_addr, num_cores=1, shared=False)
            if cordic:
                cordic.mmio_port = system.accel_bus.mem_side_ports
                system.dedicated_cordic.append(cordic)
                accel_info['accelerators'].append({
                    'name': f'cordic_core{core_id}',
                    'type': 'cordic',
                    'core': core_id,
                    'addr': cordic_addr,
                    'shared': False
                })
                accel_info['total_area'] += AREA_COST['cordic']
                accel_info['total_power'] += POWER_COST['cordic']
                accel_info['dedicated_count'] += 1
                print(f"    ✓ CORDIC @ 0x{cordic_addr:08x}")
        
        # TPU accelerator for each core (if enabled)
        if args.enable_tpu:
            tpu_addr = get_dedicated_addr(core_id, 0x20000)
            tpu = create_tpu_accel(tpu_addr, shared=False)
            if tpu:
                tpu.cpu_side = system.accel_bus.mem_side_ports
                if hasattr(tpu, 'mem_side'):
                    tpu.mem_side = system.membus.cpu_side_ports
                system.dedicated_tpu.append(tpu)
                accel_info['accelerators'].append({
                    'name': f'tpu_core{core_id}',
                    'type': 'tpu',
                    'core': core_id,
                    'addr': tpu_addr,
                    'shared': False
                })
                accel_info['total_area'] += AREA_COST['tpu']
                accel_info['total_power'] += POWER_COST['tpu']
                accel_info['dedicated_count'] += 1
                print(f"    ✓ TinyTPU @ 0x{tpu_addr:08x}")

def configure_hybrid(system, np, args, accel_info):
    """Configure HYBRID mode - critical accelerators dedicated, others shared"""
    print("\n>>> HYBRID MODE: Critical accelerators dedicated, bulk accelerators shared")
    
    # Dedicated CORDIC for each core (critical for real-time attitude control)
    system.dedicated_cordic = []
    for core_id in range(np):
        if args.enable_cordic:
            cordic_addr = get_dedicated_addr(core_id, 0x10000)
            cordic = create_cordic_accel(cordic_addr, num_cores=1, shared=False)
            if cordic:
                cordic.mmio_port = system.accel_bus.mem_side_ports
                system.dedicated_cordic.append(cordic)
                accel_info['accelerators'].append({
                    'name': f'cordic_core{core_id}',
                    'type': 'cordic',
                    'core': core_id,
                    'addr': cordic_addr,
                    'shared': False
                })
                accel_info['total_area'] += AREA_COST['cordic']
                accel_info['total_power'] += POWER_COST['cordic']
                accel_info['dedicated_count'] += 1
                print(f"  Core {core_id}: Dedicated CORDIC @ 0x{cordic_addr:08x}")
    
    # Shared Matrix accelerator (bulk computation)
    if args.enable_matrix:
        matrix = create_matrix_accel(SHARED_MATRIX_BASE, num_cores=np, shared=True, size='16kB')
        if matrix:
            matrix.cpu_side = system.accel_bus.mem_side_ports
            if hasattr(matrix, 'mem_side'):
                matrix.mem_side = system.membus.cpu_side_ports
            system.shared_matrix = matrix
            accel_info['accelerators'].append({
                'name': 'shared_matrix',
                'type': 'matrix',
                'core': 'all',
                'addr': SHARED_MATRIX_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['matrix'] * 1.5  # Larger shared version
            accel_info['total_power'] += POWER_COST['matrix'] * 1.5
            accel_info['shared_count'] += 1
            print(f"  Shared Matrix @ 0x{SHARED_MATRIX_BASE:08x} (for all {np} cores)")
    
    # Shared TPU (bulk ML inference)
    if args.enable_tpu:
        tpu = create_tpu_accel(SHARED_TPU_BASE, shared=True, size='16kB')
        if tpu:
            tpu.cpu_side = system.accel_bus.mem_side_ports
            if hasattr(tpu, 'mem_side'):
                tpu.mem_side = system.membus.cpu_side_ports
            system.shared_tpu = tpu
            accel_info['accelerators'].append({
                'name': 'shared_tpu',
                'type': 'tpu',
                'core': 'all',
                'addr': SHARED_TPU_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['tpu'] * 1.5
            accel_info['total_power'] += POWER_COST['tpu'] * 1.5
            accel_info['shared_count'] += 1
            print(f"  Shared TinyTPU @ 0x{SHARED_TPU_BASE:08x} (for all {np} cores)")
    
    # Shared compression
    if args.enable_compress:
        compress = create_compression_accel(SHARED_COMPRESS_BASE, shared=True)
        if compress:
            compress.pio = system.accel_bus.mem_side_ports
            system.shared_compress = compress
            accel_info['accelerators'].append({
                'name': 'shared_compress',
                'type': 'compress',
                'core': 'all',
                'addr': SHARED_COMPRESS_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['compress']
            accel_info['total_power'] += POWER_COST['compress']
            accel_info['shared_count'] += 1
            print(f"  Shared Compression @ 0x{SHARED_COMPRESS_BASE:08x}")

def configure_fully_shared(system, np, args, accel_info):
    """Configure FULLY_SHARED mode - all accelerators shared among cores"""
    print("\n>>> FULLY SHARED MODE: All accelerators shared among cores")
    
    # Shared Matrix accelerator
    if args.enable_matrix:
        matrix = create_matrix_accel(SHARED_MATRIX_BASE, num_cores=np, shared=True, size='16kB')
        if matrix:
            matrix.cpu_side = system.accel_bus.mem_side_ports
            if hasattr(matrix, 'mem_side'):
                matrix.mem_side = system.membus.cpu_side_ports
            system.shared_matrix = matrix
            accel_info['accelerators'].append({
                'name': 'shared_matrix',
                'type': 'matrix',
                'core': 'all',
                'addr': SHARED_MATRIX_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['matrix'] * 1.5
            accel_info['total_power'] += POWER_COST['matrix'] * 1.5
            accel_info['shared_count'] += 1
            print(f"  Shared Matrix @ 0x{SHARED_MATRIX_BASE:08x}")
    
    # Shared CORDIC
    if args.enable_cordic:
        cordic = create_cordic_accel(SHARED_CORDIC_BASE, num_cores=np, shared=True, size='8kB')
        if cordic:
            cordic.mmio_port = system.accel_bus.mem_side_ports
            system.shared_cordic = cordic
            accel_info['accelerators'].append({
                'name': 'shared_cordic',
                'type': 'cordic',
                'core': 'all',
                'addr': SHARED_CORDIC_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['cordic'] * 1.3
            accel_info['total_power'] += POWER_COST['cordic'] * 1.3
            accel_info['shared_count'] += 1
            print(f"  Shared CORDIC @ 0x{SHARED_CORDIC_BASE:08x}")
    
    # Shared TPU
    if args.enable_tpu:
        tpu = create_tpu_accel(SHARED_TPU_BASE, shared=True, size='16kB')
        if tpu:
            tpu.cpu_side = system.accel_bus.mem_side_ports
            if hasattr(tpu, 'mem_side'):
                tpu.mem_side = system.membus.cpu_side_ports
            system.shared_tpu = tpu
            accel_info['accelerators'].append({
                'name': 'shared_tpu',
                'type': 'tpu',
                'core': 'all',
                'addr': SHARED_TPU_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['tpu'] * 1.5
            accel_info['total_power'] += POWER_COST['tpu'] * 1.5
            accel_info['shared_count'] += 1
            print(f"  Shared TinyTPU @ 0x{SHARED_TPU_BASE:08x}")
    
    # Shared ML accelerator
    if args.enable_ml:
        ml = create_ml_accel(SHARED_ML_BASE, shared=True)
        if ml:
            ml.pio = system.accel_bus.mem_side_ports
            system.shared_ml = ml
            accel_info['accelerators'].append({
                'name': 'shared_ml',
                'type': 'ml',
                'core': 'all',
                'addr': SHARED_ML_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['ml']
            accel_info['total_power'] += POWER_COST['ml']
            accel_info['shared_count'] += 1
            print(f"  Shared TinyML @ 0x{SHARED_ML_BASE:08x}")
    
    # Shared compression
    if args.enable_compress:
        compress = create_compression_accel(SHARED_COMPRESS_BASE, shared=True)
        if compress:
            compress.pio = system.accel_bus.mem_side_ports
            system.shared_compress = compress
            accel_info['accelerators'].append({
                'name': 'shared_compress',
                'type': 'compress',
                'core': 'all',
                'addr': SHARED_COMPRESS_BASE,
                'shared': True
            })
            accel_info['total_area'] += AREA_COST['compress']
            accel_info['total_power'] += POWER_COST['compress']
            accel_info['shared_count'] += 1
            print(f"  Shared Compression @ 0x{SHARED_COMPRESS_BASE:08x}")

# =============================================================================
# Power Manager
# =============================================================================

def add_power_manager(system, args):
    """Add energy-proportional power manager"""
    system.power_manager = EnergyProportionalManager(
        total_power_budget=args.power_budget,
        base_power=0.5,
        solar_panel_capacity=50.0,
        battery_capacity=100.0,
        thermal_limit=10.0,
        enable_dvfs=args.enable_dvfs,
        enable_power_gating=args.enable_power_gating,
    )
    print(f"\n[Power] Energy manager enabled: budget={args.power_budget}W")

# =============================================================================
# Fault Injector
# =============================================================================

def add_fault_injector(system, args):
    """Add fault injection framework for resilience testing"""
    system.fault_injector = FaultInjector(
        fault_rate=args.fault_rate,
        enable_random_faults=args.random_faults,
        max_faults=args.max_faults,
        watchdog_timeout=Cycles(args.watchdog_timeout),
        enable_recovery=args.enable_recovery,
    )
    print(f"\n[Fault] Fault injector enabled: rate={args.fault_rate}/Mcycles")

# =============================================================================
# Traffic Generator
# =============================================================================

def add_traffic_generator(system, args):
    """Add GNC traffic generator"""
    workload_map = {
        'attitude': 0, 'orbit': 1, 'navigation': 2,
        'image': 3, 'compress': 4, 'telemetry': 5,
        'command': 6, 'mixed': 7, 'stress': 8
    }
    
    system.traffic_gen = SpaceTrafficGen(
        workload_type=workload_map.get(args.traffic_workload, 7),
        injection_rate=args.injection_rate,
        num_cores=args.num_cpus,
        enable_deadlines=args.enable_deadlines,
        matrix_accel_base=SHARED_MATRIX_BASE,
        cordic_accel_base=SHARED_CORDIC_BASE,
    )
    system.traffic_gen.traffic_port = system.accel_bus.cpu_side_ports
    print(f"\n[Traffic] GNC traffic generator: {args.traffic_workload}")

# =============================================================================
# Mission Metrics
# =============================================================================

def add_mission_metrics(system, args):
    """Add mission-aware performance metrics collector"""
    system.mission_metrics = MissionMetrics()
    print("\n[Metrics] Mission metrics collector enabled")

# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Spacecraft PhD Study: Dedicated vs Hybrid vs Fully Shared"
    )
    
    # Basic options
    parser.add_argument("--num-cpus", type=int, default=4,
                        help="Number of CPU cores")
    parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                        choices=["AtomicSimpleCPU", "TimingSimpleCPU", "O3CPU"],
                        help="CPU model")
    parser.add_argument("--cpu-clock", type=str, default="1GHz",
                        help="CPU clock frequency")
    parser.add_argument("--sys-clock", type=str, default="1GHz",
                        help="System clock frequency")
    parser.add_argument("--sys-voltage", type=str, default="1.0V",
                        help="System voltage")
    parser.add_argument("--mem-size", type=str, default="512MB",
                        help="System memory size")
    
    # Allocation mode
    parser.add_argument("--allocation-mode", type=str, default="dedicated",
                        choices=["dedicated", "hybrid", "fully_shared"],
                        help="Accelerator allocation strategy")
    
    # Accelerator enables
    parser.add_argument("--enable-matrix", action="store_true", default=True,
                        help="Enable matrix accelerator")
    parser.add_argument("--enable-cordic", action="store_true", default=True,
                        help="Enable CORDIC accelerator")
    parser.add_argument("--enable-tpu", action="store_true", default=False,
                        help="Enable TinyTPU accelerator")
    parser.add_argument("--enable-ml", action="store_true", default=False,
                        help="Enable TinyML accelerator")
    parser.add_argument("--enable-compress", action="store_true", default=False,
                        help="Enable compression accelerator")
    
    # Power model options
    parser.add_argument("--enable-power-model", action="store_true",
                        help="Enable power/energy modeling")
    parser.add_argument("--power-budget", type=float, default=10.0,
                        help="Total power budget (Watts)")
    parser.add_argument("--enable-dvfs", action="store_true",
                        help="Enable dynamic voltage/frequency scaling")
    parser.add_argument("--enable-power-gating", action="store_true",
                        help="Enable power gating for idle accelerators")
    
    # Fault injection options
    parser.add_argument("--enable-fault-injection", action="store_true",
                        help="Enable fault injection for resilience testing")
    parser.add_argument("--fault-rate", type=float, default=0.1,
                        help="Fault rate (faults per million cycles)")
    parser.add_argument("--random-faults", action="store_true",
                        help="Enable random fault injection")
    parser.add_argument("--max-faults", type=int, default=-1,
                        help="Maximum faults to inject (-1 for unlimited)")
    parser.add_argument("--watchdog-timeout", type=int, default=1000000,
                        help="Watchdog timeout in cycles")
    parser.add_argument("--enable-recovery", action="store_true",
                        help="Enable automatic fault recovery")
    
    # Traffic generator options
    parser.add_argument("--enable-traffic-gen", action="store_true",
                        help="Enable GNC traffic generator")
    parser.add_argument("--traffic-workload", type=str, default="mixed",
                        choices=["attitude", "orbit", "navigation", "image",
                                "compress", "telemetry", "command", "mixed", "stress"],
                        help="Traffic workload type")
    parser.add_argument("--injection-rate", type=float, default=1.0,
                        help="Traffic injection rate (requests/us)")
    parser.add_argument("--enable-deadlines", action="store_true",
                        help="Enable real-time deadline checking")
    
    # Metrics options
    parser.add_argument("--enable-metrics", action="store_true",
                        help="Enable mission metrics collection")
    
    # Workload options
    parser.add_argument("--cmd", type=str, default="",
                        help="Binary to execute")
    parser.add_argument("--options", type=str, default="",
                        help="Options for binary")
    
    # Full system option
    parser.add_argument("--full-system", action="store_true",
                        help="Use full system simulation (experimental)")
    
    args = parser.parse_args()
    
    # Check if binary is provided
    if not args.cmd:
        print("Error: --cmd is required to specify the binary to execute")
        sys.exit(1)
    
    # Build system
    system, accel_info = build_spacecraft_system(args)
    
    # Set up SE workload (critical for SE mode!)
    system.workload = SEWorkload.init_compatible(args.cmd)
    
    # Create workload process - shared process for all CPUs
    process = Process()
    process.cmd = [args.cmd]
    if args.options:
        process.cmd.extend(args.options.split())
    
    # Set workload for each CPU
    for cpu in system.cpu:
        cpu.workload = process
        cpu.createThreads()
    
    # Create root
    root = Root(full_system=False, system=system)
    
    # Save configuration
    config_file = path.join(m5.options.outdir, 'accel_config.json')
    with open(config_file, 'w') as f:
        json.dump(accel_info, f, indent=2, default=str)
    print(f"\nConfiguration saved to: {config_file}")
    
    # Instantiate and simulate
    m5.instantiate()
    
    print("\n" + "=" * 80)
    print("STARTING SIMULATION")
    print("=" * 80)
    
    exit_event = m5.simulate()
    
    print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")
    
    # Print summary
    print("\n" + "=" * 80)
    print("SIMULATION COMPLETE")
    print("=" * 80)
    print(f"Mode: {args.allocation_mode.upper()}")
    print(f"Cores: {args.num_cpus}")
    print(f"Total Ticks: {m5.curTick()}")
    print("=" * 80)

if __name__ == "__main__":
    main()


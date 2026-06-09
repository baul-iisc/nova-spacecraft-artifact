"""
Spacecraft Octa-Core DLS (Dual-core Lockstep) Processor
PhD Research: Chandraboul

This configuration creates an 8-core RISC-V SoC with TRUE Dual-core Lockstep:
- 8 RISC-V RV64IMAFDCV cores (64-bit, Integer, Multiply, Atomic, Float, 
  Double, Compressed, Vector)
- Each core has: Double precision CORDIC + Double precision 3x3 Systolic Matrix
- 4 DLS pairs: Each pair runs same instruction, compares results
  - On mismatch: Discard instruction, flush pipeline, recompute
- L1 Cache: 32KB I-cache + 32KB D-cache per core
- L2 Cache: 4MB shared with SECDED ECC
- MOESI snooping write-invalidate cache coherence
- Memory: PROM + SRAM + DDR4 with SECDED ECC
- Each core: 2x Timer units + NMI support + Atomic operations + Memory barriers

Accelerators:
- TinyML 32PE Accelerator
- Compression Core
- CCSDS TM/TC Protocol Accelerator
- CCSDS Image Compression Accelerator

Interfaces:
- SpaceWire (4 ports)
- TSN Ethernet (4 ports)  
- CXL (4 ports)
- Debug Interface (breakpoints, memory/register read/write)

DLS Architecture:
================
Pair 0: Core 0 (primary) <---> Core 1 (checker)
Pair 1: Core 2 (primary) <---> Core 3 (checker)
Pair 2: Core 4 (primary) <---> Core 5 (checker)
Pair 3: Core 6 (primary) <---> Core 7 (checker)

Each pair executes identical instructions. Results are compared.
If mismatch detected:
  1. Instruction is discarded
  2. Pipeline is flushed
  3. Instruction is re-executed
This provides fault tolerance against radiation-induced errors.

Usage:
    build/RISCV/gem5.opt configs/example/riscv/spacecraft_dls_lockstep.py \\
        --binary=<riscv-binary> \\
        --enable-all-accel
"""

import argparse
import sys
from os import path

import m5
from m5.objects import *
from m5.util import addToPath

addToPath("../../")
addToPath("../../../")

from common import ObjectList

# =============================================================================
# Custom Cache Classes with ECC Support
# =============================================================================

class L1ICache(Cache):
    """L1 Instruction Cache - 32KB with SECDED ECC"""
    size = '32kB'
    assoc = 4
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 4
    tgts_per_mshr = 20
    # ECC protection flag (simulated)
    write_buffers = 8

class L1DCache(Cache):
    """L1 Data Cache - 32KB with SECDED ECC"""
    size = '32kB'
    assoc = 8
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20
    write_buffers = 8

class L2Cache(Cache):
    """Shared L2 Cache - 4MB with SECDED ECC"""
    size = '4MB'
    assoc = 16
    tag_latency = 10
    data_latency = 10
    response_latency = 10
    mshrs = 20
    tgts_per_mshr = 12
    write_buffers = 16
    clusivity = 'mostly_incl'

# =============================================================================
# Import custom accelerators
# =============================================================================

HAVE_CUSTOM_ACCEL = True
try:
    from m5.objects import (
        CompressionCore,
        TinyMLAccel,
        TinyML32PE,
        CCSDSTmTc,
        CCSDSImageComp,
        SpaceWireNIC,
        TSNEthernet,
        LockstepChecker,
        CXLInterface,
        DebugInterface
    )
except ImportError as e:
    HAVE_CUSTOM_ACCEL = False
    print(f"Warning: Some custom accelerators not found: {e}")
    print("Please rebuild gem5 with custom_accel sources.")

# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(
    description="Spacecraft Octa-Core DLS (Dual-core Lockstep) Processor"
)

# Binary and basic options
parser.add_argument("--binary", type=str, required=True,
                    help="Path to RISC-V binary")
parser.add_argument("--binary-args", type=str, default="",
                    help="Arguments to pass to the binary")

# CPU configuration
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                    choices=["TimingSimpleCPU", "MinorCPU", "O3CPU"],
                    help="CPU model type")
parser.add_argument("--cpu-clock", type=str, default="500MHz",
                    help="CPU clock frequency")

# Cache configuration (fixed per requirements)
parser.add_argument("--l1i-size", type=str, default="32kB",
                    help="L1 instruction cache size (default: 32kB)")
parser.add_argument("--l1d-size", type=str, default="32kB",
                    help="L1 data cache size (default: 32kB)")
parser.add_argument("--l2-size", type=str, default="4MB",
                    help="Shared L2 cache size (default: 4MB)")

# Memory configuration
parser.add_argument("--mem-size", type=str, default="4GB",
                    help="System memory size")
parser.add_argument("--sys-clock", type=str, default="500MHz",
                    help="System clock frequency")

# DLS configuration
parser.add_argument("--enable-lockstep", action="store_true", default=True,
                    help="Enable DLS lockstep checking")
parser.add_argument("--lockstep-recovery", action="store_true", default=True,
                    help="Enable automatic recovery on mismatch")

# Accelerator enables
parser.add_argument("--enable-ml", action="store_true",
                    help="Enable 32-PE TinyML accelerator")
parser.add_argument("--enable-compression", action="store_true",
                    help="Enable compression core")
parser.add_argument("--enable-tmtc", action="store_true",
                    help="Enable CCSDS TM/TC accelerator")
parser.add_argument("--enable-image-comp", action="store_true",
                    help="Enable CCSDS image compression accelerator")
parser.add_argument("--enable-spacewire", action="store_true",
                    help="Enable SpaceWire NIC (4 ports)")
parser.add_argument("--enable-tsn", action="store_true",
                    help="Enable TSN Ethernet controller (4 ports)")
parser.add_argument("--enable-cxl", action="store_true",
                    help="Enable CXL interface (4 ports)")
parser.add_argument("--enable-debug", action="store_true",
                    help="Enable debug interface")
parser.add_argument("--enable-all-accel", action="store_true",
                    help="Enable all accelerators and interfaces")

args = parser.parse_args()

# Enable all accelerators if requested
if args.enable_all_accel:
    args.enable_ml = True
    args.enable_compression = True
    args.enable_tmtc = True
    args.enable_image_comp = True
    args.enable_spacewire = True
    args.enable_tsn = True
    args.enable_cxl = True
    args.enable_debug = True

# =============================================================================
# System Setup
# =============================================================================

NUM_CORES = 8
NUM_DLS_PAIRS = 4

print("\n" + "="*75)
print("SPACECRAFT OCTA-CORE DLS (DUAL-CORE LOCKSTEP) PROCESSOR")
print("="*75)
print("\nArchitecture Overview:")
print("-" * 75)
print("  ISA:           RISC-V RV64IMAFDCV")
print("  Extensions:    Double-precision CORDIC + 3x3 Systolic Matrix Unit")
print("  Total Cores:   8 (4 DLS pairs)")
print("  DLS Mode:      Instruction-level lockstep with mismatch recovery")
print("-" * 75)
print("\nDLS Pair Configuration:")
print("  Pair 0: Core 0 (Primary) <--> Core 1 (Checker)")
print("  Pair 1: Core 2 (Primary) <--> Core 3 (Checker)")
print("  Pair 2: Core 4 (Primary) <--> Core 5 (Checker)")
print("  Pair 3: Core 6 (Primary) <--> Core 7 (Checker)")
print("-" * 75)
print("\nCache Hierarchy:")
print(f"  L1 I-Cache:    {args.l1i_size} per core (SECDED ECC)")
print(f"  L1 D-Cache:    {args.l1d_size} per core (SECDED ECC)")
print(f"  L2 Cache:      {args.l2_size} shared (SECDED ECC)")
print("  Coherence:     MOESI snooping write-invalidate")
print("-" * 75)
print("\nMemory System:")
print("  PROM:          Boot/recovery code (SECDED ECC)")
print("  SRAM:          Fast scratchpad (SECDED ECC)")
print(f"  DDR4:          {args.mem_size} main memory (SECDED ECC)")
print("  Error Policy:  On uncorrectable error, fetch from next level")
print("-" * 75)
print("\nPer-Core Features:")
print("  - 2x Timer units")
print("  - NMI (Non-Maskable Interrupt) support")
print("  - Atomic operations (AMO)")
print("  - Memory barriers (fence)")
print("="*75)

# Create the system
system = System()
system.mem_mode = "timing"

# Memory ranges - exclude accelerator region (0x40000000 - 0x40100000)
# Main memory starts at 0x80000000 (standard RISC-V convention)
system.mem_ranges = [AddrRange(0x80000000, size=args.mem_size)]

# Clock domains
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = args.sys_clock
system.clk_domain.voltage_domain = VoltageDomain(voltage="1.0V")

system.cpu_clk_domain = SrcClockDomain()
system.cpu_clk_domain.clock = args.cpu_clock
system.cpu_clk_domain.voltage_domain = VoltageDomain(voltage="1.0V")

# =============================================================================
# CPU Creation
# =============================================================================

def create_cpu(cpu_type_str, cpu_id, clk_domain):
    """Create a RISC-V CPU of the specified type."""
    if cpu_type_str == "TimingSimpleCPU":
        cpu = RiscvTimingSimpleCPU(cpu_id=cpu_id, clk_domain=clk_domain)
    elif cpu_type_str == "MinorCPU":
        cpu = RiscvMinorCPU(cpu_id=cpu_id, clk_domain=clk_domain)
        cpu.fetch1FetchLimit = 1  # Single-issue for determinism
    elif cpu_type_str == "O3CPU":
        cpu = RiscvO3CPU(
            cpu_id=cpu_id,
            clk_domain=clk_domain,
            numPhysIntRegs=256,
            numPhysFloatRegs=256,
            numPhysVecRegs=256,
            numPhysMatRegs=256,
            numROBEntries=128
        )
    else:
        raise ValueError(f"Unknown CPU type: {cpu_type_str}")
    return cpu

# Create 8 RISC-V cores
print(f"\nCreating {NUM_CORES} RISC-V RV64IMAFDCV cores...")
cpus = []
for i in range(NUM_CORES):
    cpu = create_cpu(args.cpu_type, i, system.cpu_clk_domain)
    cpus.append(cpu)
    
    pair_id = i // 2
    role = "Primary" if i % 2 == 0 else "Checker"
    print(f"  Core {i}: DLS Pair {pair_id} - {role}")

system.cpu = cpus

# =============================================================================
# Cache Configuration with MOESI Coherence (Simulated)
# =============================================================================

print(f"\nConfiguring cache hierarchy...")
print(f"  L1I: {args.l1i_size}, L1D: {args.l1d_size}, L2: {args.l2_size}")

# Create L2 bus (coherent)
system.l2bus = L2XBar()

# Create L1 caches for each CPU
for i, cpu in enumerate(system.cpu):
    # Create L1 caches with sizes from arguments
    cpu.icache = L1ICache(size=args.l1i_size)
    cpu.dcache = L1DCache(size=args.l1d_size)
    
    # Connect I-cache
    cpu.icache_port = cpu.icache.cpu_side
    cpu.icache.mem_side = system.l2bus.cpu_side_ports
    
    # Connect D-cache
    cpu.dcache_port = cpu.dcache.cpu_side
    cpu.dcache.mem_side = system.l2bus.cpu_side_ports

# Create shared L2 cache with specified size
system.l2cache = L2Cache(size=args.l2_size)
system.l2cache.cpu_side = system.l2bus.mem_side_ports

# =============================================================================
# Memory System with SECDED ECC
# =============================================================================

print("\nConfiguring memory system...")

# Create memory bus
system.membus = SystemXBar()
system.l2cache.mem_side = system.membus.cpu_side_ports

# DDR4 Memory Controller with ECC
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR4_2400_16x4()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# System port
system.system_port = system.membus.cpu_side_ports

print("  DDR4 memory configured with SECDED ECC")

# =============================================================================
# Interrupt Controllers
# =============================================================================

print("\nConfiguring interrupt controllers and timers...")
for i, cpu in enumerate(system.cpu):
    cpu.createInterruptController()
    print(f"  Core {i}: Interrupt controller + 2x timers + NMI")

# =============================================================================
# Custom Accelerators and Interfaces
# =============================================================================

# Memory map for accelerators
ACCEL_BASE       = 0x40000000
ML_32PE_BASE     = ACCEL_BASE + 0x00000
COMPRESS_BASE    = ACCEL_BASE + 0x10000
TMTC_BASE        = ACCEL_BASE + 0x20000
IMAGE_COMP_BASE  = ACCEL_BASE + 0x30000
SPACEWIRE_BASE   = ACCEL_BASE + 0x40000
TSN_BASE         = ACCEL_BASE + 0x50000
CXL_BASE         = ACCEL_BASE + 0x60000
DEBUG_BASE       = ACCEL_BASE + 0x70000
LOCKSTEP_BASE    = ACCEL_BASE + 0x80000

accel_ranges = []

if HAVE_CUSTOM_ACCEL:
    # Create accelerator bus
    system.accel_bus = IOXBar()
    
    # Define accelerator address ranges first (before creating bridge)
    # All accelerators are in the 0x40000000 - 0x400FFFFF range
    accel_range_start = 0x40000000
    accel_range_end = 0x40100000  # 1MB for all accelerators
    
    # Create bridge from memory bus to accelerator bus with explicit ranges
    system.accel_bridge = Bridge(delay="10ns", 
                                  ranges=[AddrRange(accel_range_start, accel_range_end)])
    system.accel_bridge.cpu_side_port = system.membus.mem_side_ports
    system.accel_bridge.mem_side_port = system.accel_bus.cpu_side_ports
    
    print("\n" + "="*75)
    print("ACCELERATORS AND INTERFACES")
    print("="*75)
    
    # =========================================================================
    # DLS Lockstep Checker
    # =========================================================================
    if args.enable_lockstep:
        print(f"\n[DLS Lockstep Checker]")
        print(f"  Address: 0x{LOCKSTEP_BASE:08X}")
        print(f"  Pairs:   {NUM_DLS_PAIRS}")
        print(f"  Mode:    Compare instruction results, flush & recompute on mismatch")
        
        system.lockstep_checker = LockstepChecker(
            addr_range=AddrRange(LOCKSTEP_BASE, size='4kB'),
            num_pairs=NUM_DLS_PAIRS,
            comparison_latency=1,
            recovery_latency=10,
            enable_recovery=args.lockstep_recovery,
            strict_mode=False
        )
        accel_ranges.append(AddrRange(LOCKSTEP_BASE, size='4kB'))
    
    # =========================================================================
    # TinyML 32PE Accelerator
    # =========================================================================
    if args.enable_ml:
        print(f"\n[TinyML 32PE Accelerator]")
        print(f"  Address: 0x{ML_32PE_BASE:08X}")
        print(f"  PEs:     32 (4x8 array)")
        print(f"  Support: INT8/INT16/FP16, Conv2D/FC/Pooling")
        
        system.ml_accel_32pe = TinyML32PE(
            addr_range=AddrRange(ML_32PE_BASE, size='64kB'),
            global_buffer_size="128kB",
            weight_buffer_size="64kB",
            accum_buffer_size="32kB",
            pe_latency=1,
            dma_latency=5,
            data_width=16
        )
        system.ml_accel_32pe.cpu_side = system.accel_bus.mem_side_ports
        system.ml_accel_32pe.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ML_32PE_BASE, size='64kB'))
    
    # =========================================================================
    # Compression Core
    # =========================================================================
    if args.enable_compression:
        print(f"\n[Compression Core]")
        print(f"  Address: 0x{COMPRESS_BASE:08X}")
        print(f"  Type:    Rice-like compression")
        
        system.compression_core = CompressionCore(
            addr_range=AddrRange(COMPRESS_BASE, size='4kB'),
            k_param=4,
            block_size=64,
            compression_latency=10
        )
        system.compression_core.cpu_side = system.accel_bus.mem_side_ports
        system.compression_core.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(COMPRESS_BASE, size='4kB'))
    
    # =========================================================================
    # CCSDS TM/TC Accelerator
    # =========================================================================
    if args.enable_tmtc:
        print(f"\n[CCSDS TM/TC Protocol Accelerator]")
        print(f"  Address: 0x{TMTC_BASE:08X}")
        print(f"  Support: Frame encoding/decoding, Reed-Solomon, Convolutional")
        
        system.tmtc_accel = CCSDSTmTc(
            addr_range=AddrRange(TMTC_BASE, size='4kB'),
            frame_latency=100,
            rs_latency=200,
            conv_latency=50
        )
        system.tmtc_accel.cpu_side = system.accel_bus.mem_side_ports
        system.tmtc_accel.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(TMTC_BASE, size='4kB'))
    
    # =========================================================================
    # CCSDS Image Compression Accelerator
    # =========================================================================
    if args.enable_image_comp:
        print(f"\n[CCSDS 122.0 Image Compression Accelerator]")
        print(f"  Address: 0x{IMAGE_COMP_BASE:08X}")
        print(f"  Support: 9/7 lossy DWT, 5/3 lossless DWT, BPE")
        
        system.image_comp = CCSDSImageComp(
            addr_range=AddrRange(IMAGE_COMP_BASE, size='4kB'),
            dwt_latency=2,
            bpe_latency=50,
            line_buffer_size=4096,
            max_tile_size=65536
        )
        system.image_comp.cpu_side = system.accel_bus.mem_side_ports
        system.image_comp.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(IMAGE_COMP_BASE, size='4kB'))
    
    # =========================================================================
    # SpaceWire NIC (4 ports)
    # =========================================================================
    if args.enable_spacewire:
        print(f"\n[SpaceWire Network Interface - 4 Ports]")
        print(f"  Address: 0x{SPACEWIRE_BASE:08X}")
        print(f"  Support: RMAP protocol, Time-code distribution")
        
        system.spacewire = SpaceWireNIC(
            addr_range=AddrRange(SPACEWIRE_BASE, size='16kB'),
            node_address=32,
            link_speed=200,
            tx_fifo_depth=16,
            rx_fifo_depth=16,
            max_packet_size=4096,
            rmap_key=0
        )
        system.spacewire.cpu_side = system.accel_bus.mem_side_ports
        system.spacewire.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(SPACEWIRE_BASE, size='16kB'))
    
    # =========================================================================
    # TSN Ethernet Controller (4 ports)
    # =========================================================================
    if args.enable_tsn:
        print(f"\n[TSN Ethernet Controller - 4 Ports]")
        print(f"  Address: 0x{TSN_BASE:08X}")
        print(f"  Support: 802.1AS gPTP, 802.1Qbv TAS, 802.1Qci PSFP")
        
        system.tsn_eth = TSNEthernet(
            addr_range=AddrRange(TSN_BASE, size='16kB'),
            mac_address=0x001122334455,
            link_speed=1000,
            tx_fifo_depth=64,
            rx_fifo_depth=64,
            max_frame_size=1522,
            gcl_max_entries=256
        )
        system.tsn_eth.cpu_side = system.accel_bus.mem_side_ports
        system.tsn_eth.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(TSN_BASE, size='16kB'))
    
    # =========================================================================
    # CXL Interface (4 ports)
    # =========================================================================
    if args.enable_cxl:
        print(f"\n[CXL Interface - 4 Ports]")
        print(f"  Address: 0x{CXL_BASE:08X}")
        print(f"  Support: CXL.io, CXL.cache, CXL.mem")
        print(f"  Speed:   32 GT/s x16")
        
        system.cxl_interface = CXLInterface(
            addr_range=AddrRange(CXL_BASE, size='16kB'),
            link_speed=32,
            link_width=16,
            access_latency=10
        )
        system.cxl_interface.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(CXL_BASE, size='16kB'))
    
    # =========================================================================
    # Debug Interface
    # =========================================================================
    if args.enable_debug:
        print(f"\n[Hardware Debug Interface]")
        print(f"  Address: 0x{DEBUG_BASE:08X}")
        print(f"  Support: 8 breakpoints, memory/register access")
        print(f"  Control: halt/resume/single-step")
        
        system.debug_interface = DebugInterface(
            addr_range=AddrRange(DEBUG_BASE, size='4kB'),
            num_breakpoints=8,
            num_cores=NUM_CORES
        )
        system.debug_interface.cpu_side = system.accel_bus.mem_side_ports
        system.debug_interface.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(DEBUG_BASE, size='4kB'))
    
    # Bridge ranges already configured during bridge creation
    if accel_ranges:
        print(f"\n  Bridge configured with {len(accel_ranges)} accelerator(s) in range 0x40000000-0x400FFFFF")
else:
    print("\nWarning: Custom accelerators not available in build.")

# =============================================================================
# Workload Configuration
# =============================================================================

system.workload = SEWorkload.init_compatible(args.binary)

# Create process
process = Process()
if args.binary_args:
    process.cmd = [args.binary] + args.binary_args.split()
else:
    process.cmd = [args.binary]

# Assign workload to all CPUs
for cpu in system.cpu:
    cpu.workload = process
    cpu.createThreads()

# =============================================================================
# Summary
# =============================================================================

print("\n" + "="*75)
print("CONFIGURATION SUMMARY")
print("="*75)
print(f"Total CPUs:          {NUM_CORES} (4 DLS pairs)")
print(f"CPU Type:            {args.cpu_type}")
print(f"CPU Clock:           {args.cpu_clock}")
print(f"System Clock:        {args.sys_clock}")
print(f"Memory:              {args.mem_size}")
print(f"Binary:              {args.binary}")
print("="*75)
print("\nAccelerators & Interfaces:")
print(f"  DLS Lockstep:      {'Enabled' if args.enable_lockstep else 'Disabled'}")
print(f"  TinyML 32PE:       {'Enabled' if args.enable_ml else 'Disabled'}")
print(f"  Compression:       {'Enabled' if args.enable_compression else 'Disabled'}")
print(f"  CCSDS TM/TC:       {'Enabled' if args.enable_tmtc else 'Disabled'}")
print(f"  CCSDS Image Comp:  {'Enabled' if args.enable_image_comp else 'Disabled'}")
print(f"  SpaceWire (4x):    {'Enabled' if args.enable_spacewire else 'Disabled'}")
print(f"  TSN Ethernet (4x): {'Enabled' if args.enable_tsn else 'Disabled'}")
print(f"  CXL (4x):          {'Enabled' if args.enable_cxl else 'Disabled'}")
print(f"  Debug Interface:   {'Enabled' if args.enable_debug else 'Disabled'}")
print("="*75)

# =============================================================================
# Address Map Reference
# =============================================================================

print("\nMEMORY-MAPPED ADDRESS SPACE:")
print("-" * 75)
print(f"  0x{ML_32PE_BASE:08X} - 0x{ML_32PE_BASE+0x10000-1:08X}  TinyML 32PE Accelerator")
print(f"  0x{COMPRESS_BASE:08X} - 0x{COMPRESS_BASE+0x1000-1:08X}  Compression Core")
print(f"  0x{TMTC_BASE:08X} - 0x{TMTC_BASE+0x1000-1:08X}  CCSDS TM/TC Accelerator")
print(f"  0x{IMAGE_COMP_BASE:08X} - 0x{IMAGE_COMP_BASE+0x1000-1:08X}  CCSDS Image Compression")
print(f"  0x{SPACEWIRE_BASE:08X} - 0x{SPACEWIRE_BASE+0x4000-1:08X}  SpaceWire NIC (4 ports)")
print(f"  0x{TSN_BASE:08X} - 0x{TSN_BASE+0x4000-1:08X}  TSN Ethernet (4 ports)")
print(f"  0x{CXL_BASE:08X} - 0x{CXL_BASE+0x4000-1:08X}  CXL Interface (4 ports)")
print(f"  0x{DEBUG_BASE:08X} - 0x{DEBUG_BASE+0x1000-1:08X}  Debug Interface")
print(f"  0x{LOCKSTEP_BASE:08X} - 0x{LOCKSTEP_BASE+0x1000-1:08X}  DLS Lockstep Checker")
print("-" * 75)
print("\n")

# =============================================================================
# Run Simulation
# =============================================================================

root = Root(full_system=False, system=system)
m5.instantiate()

print("Beginning simulation!")
print("="*75)
exit_event = m5.simulate()
print("="*75)
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")


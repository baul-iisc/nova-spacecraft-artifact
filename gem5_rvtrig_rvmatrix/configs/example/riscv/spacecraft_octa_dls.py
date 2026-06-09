"""
Spacecraft Heterogeneous Octa-Core Processor with DLS Architecture
PhD Research: Chandraboul

This configuration creates an octa-core RISC-V system with:
- 8 RISC-V CPU cores in DLS (Decoupled Look-ahead/Supply) configuration
  - 4 Application Processing cores (AP)
  - 2 Look-ahead/Decoupled Access cores (DAE)
  - 2 Safety/Redundancy cores (SR)
- Enhanced TinyML Accelerator with 32 PEs
- CCSDS TM/TC Protocol Accelerator
- CCSDS 122.0 Image Compression Accelerator
- SpaceWire Network Interface
- TSN Ethernet Controller
- Private L1 caches per core
- Shared L2 cache with ECC
- DDR4 memory controller

DLS Architecture Overview:
------------------------
The Decoupled Look-ahead/Supply (DLS) architecture separates memory access
from computation to achieve deterministic timing and better cache utilization
in spacecraft applications.

Core Configuration:
- AP Cores (0-3): Application Processing - main computation
- DAE Cores (4-5): Decoupled Access/Execute - prefetch and data staging
- SR Cores (6-7): Safety/Redundancy - lockstep or TMR execution

Usage:
    build/RISCV/gem5.opt configs/example/riscv/spacecraft_octa_dls.py \\
        --binary=<riscv-binary> \\
        --num-ap-cores=4 \\
        --num-dae-cores=2 \\
        --num-sr-cores=2 \\
        --enable-ml \\
        --enable-compression \\
        --enable-spacewire \\
        --enable-tsn
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
from learning_gem5.part1.caches import L1ICache, L1DCache, L2Cache

# =============================================================================
# Import custom accelerators
# =============================================================================

try:
    from m5.objects import (
        CompressionCore, 
        TinyMLAccel,
        TinyML32PE,
        CCSDSTmTc,
        CCSDSImageComp,
        SpaceWireNIC,
        TSNEthernet
    )
    HAVE_CUSTOM_ACCEL = True
except ImportError as e:
    HAVE_CUSTOM_ACCEL = False
    print(f"Warning: Some custom accelerators not found: {e}")
    print("Please rebuild gem5 with custom_accel sources.")

# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(
    description="Spacecraft Octa-Core DLS Architecture"
)

# Binary and basic options
parser.add_argument("--binary", type=str, required=True,
                    help="Path to RISC-V binary")
parser.add_argument("--binary-args", type=str, default="",
                    help="Arguments to pass to the binary")

# Core configuration
parser.add_argument("--num-ap-cores", type=int, default=4,
                    help="Number of Application Processing cores (default: 4)")
parser.add_argument("--num-dae-cores", type=int, default=2,
                    help="Number of Decoupled Access/Execute cores (default: 2)")
parser.add_argument("--num-sr-cores", type=int, default=2,
                    help="Number of Safety/Redundancy cores (default: 2)")

# CPU type selection
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                    choices=["TimingSimpleCPU", "MinorCPU", "O3CPU"],
                    help="CPU model type")
parser.add_argument("--ap-cpu-type", type=str, default=None,
                    help="CPU type for AP cores (overrides --cpu-type)")
parser.add_argument("--dae-cpu-type", type=str, default=None,
                    help="CPU type for DAE cores (overrides --cpu-type)")

# Cache configuration
parser.add_argument("--l1i-size", type=str, default="16kB",
                    help="L1 instruction cache size")
parser.add_argument("--l1d-size", type=str, default="32kB",
                    help="L1 data cache size")
parser.add_argument("--l2-size", type=str, default="512kB",
                    help="Shared L2 cache size")

# Memory configuration
parser.add_argument("--mem-size", type=str, default="2GB",
                    help="System memory size")
parser.add_argument("--mem-type", type=str, default="DDR4_2400_16x4",
                    help="Memory type")

# Clock configuration
parser.add_argument("--sys-clock", type=str, default="500MHz",
                    help="System clock frequency")
parser.add_argument("--cpu-clock", type=str, default="500MHz",
                    help="CPU clock frequency")

# Accelerator enables
parser.add_argument("--enable-ml", action="store_true",
                    help="Enable 32-PE TinyML accelerator")
parser.add_argument("--enable-ml-basic", action="store_true",
                    help="Enable basic TinyML accelerator")
parser.add_argument("--enable-compression", action="store_true",
                    help="Enable compression core")
parser.add_argument("--enable-tmtc", action="store_true",
                    help="Enable CCSDS TM/TC accelerator")
parser.add_argument("--enable-image-comp", action="store_true",
                    help="Enable CCSDS image compression accelerator")
parser.add_argument("--enable-spacewire", action="store_true",
                    help="Enable SpaceWire NIC")
parser.add_argument("--enable-tsn", action="store_true",
                    help="Enable TSN Ethernet controller")
parser.add_argument("--enable-all-accel", action="store_true",
                    help="Enable all accelerators")

# TinyML configuration
parser.add_argument("--ml-global-buffer", type=str, default="128kB",
                    help="TinyML global buffer size")
parser.add_argument("--ml-weight-buffer", type=str, default="64kB",
                    help="TinyML weight buffer size")

# SpaceWire configuration
parser.add_argument("--spw-node-addr", type=int, default=32,
                    help="SpaceWire node logical address")
parser.add_argument("--spw-link-speed", type=int, default=200,
                    help="SpaceWire link speed in Mbps")

# TSN configuration
parser.add_argument("--tsn-link-speed", type=int, default=1000,
                    help="TSN Ethernet link speed in Mbps")

args = parser.parse_args()

# Enable all accelerators if requested
if args.enable_all_accel:
    args.enable_ml = True
    args.enable_compression = True
    args.enable_tmtc = True
    args.enable_image_comp = True
    args.enable_spacewire = True
    args.enable_tsn = True

# Calculate total cores
total_cores = args.num_ap_cores + args.num_dae_cores + args.num_sr_cores

# =============================================================================
# System Setup
# =============================================================================

print("\n" + "="*70)
print("SPACECRAFT OCTA-CORE DLS PROCESSOR - Syscall Emulation Mode")
print("="*70)
print(f"Total Cores: {total_cores}")
print(f"  - AP Cores (Application Processing): {args.num_ap_cores}")
print(f"  - DAE Cores (Decoupled Access/Execute): {args.num_dae_cores}")
print(f"  - SR Cores (Safety/Redundancy): {args.num_sr_cores}")
print("="*70)

# Create the system
system = System()
system.mem_mode = "timing"
system.mem_ranges = [AddrRange(args.mem_size)]

# Clock domains
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = args.sys_clock
system.clk_domain.voltage_domain = VoltageDomain(voltage="1.0V")

system.cpu_clk_domain = SrcClockDomain()
system.cpu_clk_domain.clock = args.cpu_clock
system.cpu_clk_domain.voltage_domain = VoltageDomain(voltage="1.0V")

# =============================================================================
# CPU Creation Helper
# =============================================================================

def create_cpu(cpu_type_str, cpu_id, clk_domain):
    """Create a CPU of the specified type."""
    if cpu_type_str == "TimingSimpleCPU":
        return RiscvTimingSimpleCPU(cpu_id=cpu_id, clk_domain=clk_domain)
    elif cpu_type_str == "MinorCPU":
        cpu = RiscvMinorCPU(cpu_id=cpu_id, clk_domain=clk_domain)
        cpu.fetch1FetchLimit = 1  # Single-issue for determinism
        return cpu
    elif cpu_type_str == "O3CPU":
        return RiscvO3CPU(
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

# =============================================================================
# Create CPUs
# =============================================================================

cpus = []
cpu_id = 0

# Determine CPU types
ap_cpu_type = args.ap_cpu_type if args.ap_cpu_type else args.cpu_type
dae_cpu_type = args.dae_cpu_type if args.dae_cpu_type else args.cpu_type

print(f"\nCPU Configuration:")
print(f"  AP Core Type: {ap_cpu_type}")
print(f"  DAE Core Type: {dae_cpu_type}")
print(f"  SR Core Type: {args.cpu_type}")

# Create AP (Application Processing) cores
print(f"\nCreating {args.num_ap_cores} AP Cores...")
for i in range(args.num_ap_cores):
    cpu = create_cpu(ap_cpu_type, cpu_id, system.cpu_clk_domain)
    cpus.append(cpu)
    print(f"  CPU {cpu_id}: AP Core (Application Processing)")
    cpu_id += 1

# Create DAE (Decoupled Access/Execute) cores
print(f"\nCreating {args.num_dae_cores} DAE Cores...")
for i in range(args.num_dae_cores):
    # DAE cores typically use simpler in-order cores
    cpu = create_cpu(dae_cpu_type, cpu_id, system.cpu_clk_domain)
    cpus.append(cpu)
    print(f"  CPU {cpu_id}: DAE Core (Decoupled Access/Execute)")
    cpu_id += 1

# Create SR (Safety/Redundancy) cores
print(f"\nCreating {args.num_sr_cores} SR Cores...")
for i in range(args.num_sr_cores):
    cpu = create_cpu(args.cpu_type, cpu_id, system.cpu_clk_domain)
    cpus.append(cpu)
    print(f"  CPU {cpu_id}: SR Core (Safety/Redundancy)")
    cpu_id += 1

system.cpu = cpus

# =============================================================================
# Cache Configuration
# =============================================================================

print(f"\nCache Configuration:")
print(f"  L1I: {args.l1i_size}, L1D: {args.l1d_size}, L2: {args.l2_size}")

# Create L2 bus
system.l2bus = L2XBar()

# Create L1 caches for each CPU
for i, cpu in enumerate(system.cpu):
    # Create caches
    cpu.icache = L1ICache(size=args.l1i_size)
    cpu.dcache = L1DCache(size=args.l1d_size)
    
    # Connect caches to CPU
    cpu.icache.connectCPU(cpu)
    cpu.dcache.connectCPU(cpu)
    
    # Connect caches to L2 bus
    cpu.icache.connectBus(system.l2bus)
    cpu.dcache.connectBus(system.l2bus)

# Create shared L2 cache
system.l2cache = L2Cache(size=args.l2_size)
system.l2cache.connectCPUSideBus(system.l2bus)

# =============================================================================
# Memory System
# =============================================================================

system.membus = SystemXBar()
system.l2cache.connectMemSideBus(system.membus)

# Memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR4_2400_16x4()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# System port
system.system_port = system.membus.cpu_side_ports

# =============================================================================
# Create Interrupt Controllers
# =============================================================================

for cpu in system.cpu:
    cpu.createInterruptController()

# =============================================================================
# Custom Accelerators
# =============================================================================

# Address map for accelerators
ACCEL_BASE = 0x40000000
ACCEL_ML_32PE     = ACCEL_BASE + 0x0000
ACCEL_ML_BASIC    = ACCEL_BASE + 0x1000
ACCEL_COMPRESS    = ACCEL_BASE + 0x2000
ACCEL_TMTC        = ACCEL_BASE + 0x3000
ACCEL_IMAGE_COMP  = ACCEL_BASE + 0x4000
ACCEL_SPACEWIRE   = ACCEL_BASE + 0x5000
ACCEL_TSN         = ACCEL_BASE + 0x6000

accel_ranges = []

if HAVE_CUSTOM_ACCEL:
    # Create accelerator bus
    system.accel_bus = IOXBar()
    
    # Create bridge from memory bus to accelerator bus
    system.accel_bridge = Bridge(delay="10ns")
    system.accel_bridge.cpu_side_port = system.membus.mem_side_ports
    system.accel_bridge.mem_side_port = system.accel_bus.cpu_side_ports
    
    print("\n" + "="*70)
    print("ACCELERATOR CONFIGURATION")
    print("="*70)
    
    # TinyML 32PE Accelerator
    if args.enable_ml:
        print(f"\n[TinyML 32PE Accelerator]")
        print(f"  Address: 0x{ACCEL_ML_32PE:08X}")
        print(f"  Global Buffer: {args.ml_global_buffer}")
        print(f"  Weight Buffer: {args.ml_weight_buffer}")
        
        system.ml_accel_32pe = TinyML32PE(
            addr_range=AddrRange(ACCEL_ML_32PE, size='4kB'),
            global_buffer_size=args.ml_global_buffer,
            weight_buffer_size=args.ml_weight_buffer,
            accum_buffer_size="32kB",
            pe_latency=1,
            dma_latency=5,
            data_width=16
        )
        system.ml_accel_32pe.cpu_side = system.accel_bus.mem_side_ports
        system.ml_accel_32pe.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ACCEL_ML_32PE, size='4kB'))
    
    # Basic TinyML Accelerator
    if args.enable_ml_basic:
        print(f"\n[TinyML Basic Accelerator]")
        print(f"  Address: 0x{ACCEL_ML_BASIC:08X}")
        
        system.ml_accel_basic = TinyMLAccel(
            addr_range=AddrRange(ACCEL_ML_BASIC, size='256B'),
            mac_array_rows=8,
            mac_array_cols=8,
            sram_size="64kB",
            mac_latency=1,
            dma_latency=5
        )
        system.ml_accel_basic.cpu_side = system.accel_bus.mem_side_ports
        system.ml_accel_basic.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ACCEL_ML_BASIC, size='256B'))
    
    # Compression Core
    if args.enable_compression:
        print(f"\n[Compression Core]")
        print(f"  Address: 0x{ACCEL_COMPRESS:08X}")
        
        system.compression_core = CompressionCore(
            addr_range=AddrRange(ACCEL_COMPRESS, size='256B'),
            k_param=4,
            block_size=64,
            compression_latency=10
        )
        system.compression_core.cpu_side = system.accel_bus.mem_side_ports
        system.compression_core.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ACCEL_COMPRESS, size='256B'))
    
    # CCSDS TM/TC Accelerator
    if args.enable_tmtc:
        print(f"\n[CCSDS TM/TC Accelerator]")
        print(f"  Address: 0x{ACCEL_TMTC:08X}")
        
        system.tmtc_accel = CCSDSTmTc(
            addr_range=AddrRange(ACCEL_TMTC, size='256B'),
            frame_latency=100,
            rs_latency=200,
            conv_latency=50
        )
        system.tmtc_accel.cpu_side = system.accel_bus.mem_side_ports
        system.tmtc_accel.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ACCEL_TMTC, size='256B'))
    
    # CCSDS Image Compression Accelerator
    if args.enable_image_comp:
        print(f"\n[CCSDS 122.0 Image Compression Accelerator]")
        print(f"  Address: 0x{ACCEL_IMAGE_COMP:08X}")
        
        system.image_comp = CCSDSImageComp(
            addr_range=AddrRange(ACCEL_IMAGE_COMP, size='256B'),
            dwt_latency=2,
            bpe_latency=50,
            line_buffer_size=4096,
            max_tile_size=65536
        )
        system.image_comp.cpu_side = system.accel_bus.mem_side_ports
        system.image_comp.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ACCEL_IMAGE_COMP, size='256B'))
    
    # SpaceWire NIC
    if args.enable_spacewire:
        print(f"\n[SpaceWire Network Interface]")
        print(f"  Address: 0x{ACCEL_SPACEWIRE:08X}")
        print(f"  Node Address: {args.spw_node_addr}")
        print(f"  Link Speed: {args.spw_link_speed} Mbps")
        
        system.spacewire = SpaceWireNIC(
            addr_range=AddrRange(ACCEL_SPACEWIRE, size='256B'),
            node_address=args.spw_node_addr,
            link_speed=args.spw_link_speed,
            tx_fifo_depth=16,
            rx_fifo_depth=16,
            max_packet_size=4096,
            rmap_key=0
        )
        system.spacewire.cpu_side = system.accel_bus.mem_side_ports
        system.spacewire.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ACCEL_SPACEWIRE, size='256B'))
    
    # TSN Ethernet Controller
    if args.enable_tsn:
        print(f"\n[TSN Ethernet Controller]")
        print(f"  Address: 0x{ACCEL_TSN:08X}")
        print(f"  Link Speed: {args.tsn_link_speed} Mbps")
        
        system.tsn_eth = TSNEthernet(
            addr_range=AddrRange(ACCEL_TSN, size='2kB'),
            mac_address=0x001122334455,
            link_speed=args.tsn_link_speed,
            tx_fifo_depth=64,
            rx_fifo_depth=64,
            max_frame_size=1522,
            gcl_max_entries=256
        )
        system.tsn_eth.cpu_side = system.accel_bus.mem_side_ports
        system.tsn_eth.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ACCEL_TSN, size='2kB'))
    
    # Configure bridge ranges
    if accel_ranges:
        system.accel_bridge.ranges = accel_ranges
        print(f"\n  Bridge configured for {len(accel_ranges)} accelerator(s)")
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
# Run Simulation
# =============================================================================

print("\n" + "="*70)
print("SYSTEM CONFIGURATION SUMMARY")
print("="*70)
print(f"Total CPUs:         {total_cores}")
print(f"Memory:             {args.mem_size}")
print(f"System Clock:       {args.sys_clock}")
print(f"CPU Clock:          {args.cpu_clock}")
print(f"Binary:             {args.binary}")
print("="*70)
print("\nAccelerators Enabled:")
print(f"  TinyML 32PE:      {'Yes' if args.enable_ml else 'No'}")
print(f"  TinyML Basic:     {'Yes' if args.enable_ml_basic else 'No'}")
print(f"  Compression:      {'Yes' if args.enable_compression else 'No'}")
print(f"  CCSDS TM/TC:      {'Yes' if args.enable_tmtc else 'No'}")
print(f"  Image Comp:       {'Yes' if args.enable_image_comp else 'No'}")
print(f"  SpaceWire:        {'Yes' if args.enable_spacewire else 'No'}")
print(f"  TSN Ethernet:     {'Yes' if args.enable_tsn else 'No'}")
print("="*70 + "\n")

# Create root and run
root = Root(full_system=False, system=system)
m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")




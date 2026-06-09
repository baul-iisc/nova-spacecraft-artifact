"""
Heterogeneous Spacecraft Multicore Processor Configuration
===========================================================
PhD Research: Chandraboul

This configuration creates a heterogeneous multicore processor for spacecraft
applications featuring:
- Multiple RISC-V CPU cores (configurable)
- Systolic Array for matrix operations (if available)
- Compression Core for telemetry/science data
- Tiny ML Accelerator for on-board AI

Architecture:
┌─────────────────────────────────────────────────────────────────┐
│                    Spacecraft SoC                               │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐               │
│  │ RISC-V  │ │ RISC-V  │ │ RISC-V  │ │ RISC-V  │   Main Cores  │
│  │ Core 0  │ │ Core 1  │ │ Core 2  │ │ Core 3  │               │
│  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘               │
│       └──────────┴──────────┴──────────┘                       │
│                         │                                       │
│  ┌──────────────────────┴──────────────────────┐               │
│  │              L2 Crossbar (L2XBar)            │               │
│  └──────────────────────┬──────────────────────┘               │
│       │                 │                 │                     │
│  ┌────┴────┐  ┌─────────┴─────────┐  ┌───┴───┐                 │
│  │  L2     │  │  Accelerator Bus  │  │ Main  │                 │
│  │ Cache   │  │                   │  │ Mem   │                 │
│  └────┬────┘  └──────┬───────┬───┘  └───────┘                 │
│       │              │       │                                  │
│  ┌────┴────┐  ┌──────┴───┐ ┌─┴──────┐                          │
│  │  Main   │  │Compress  │ │ TinyML │                          │
│  │  Bus    │  │  Core    │ │ Accel  │                          │
│  └─────────┘  └──────────┘ └────────┘                          │
└─────────────────────────────────────────────────────────────────┘
"""

import m5
from m5.objects import (
    Root, System, AddrRange, DDR3_1600_8x8, SrcClockDomain, 
    VoltageDomain, L2XBar, SystemXBar, SEWorkload, Process, MemCtrl
)
from caches import L1ICache, L1DCache, L2Cache
import argparse

# Import branch predictors and prefetchers
from m5.objects import TAGE, BiModeBP, LTAGE, TournamentBP
from m5.objects import StridePrefetcher, TaggedPrefetcher

# Import CPU models
from m5.objects import (
    RiscvTimingSimpleCPU, RiscvMinorCPU, RiscvO3CPU, RiscvAtomicSimpleCPU
)

# Import custom accelerators
try:
    from m5.objects import CompressionCore, TinyMLAccel
    HAVE_CUSTOM_ACCEL = True
except ImportError:
    HAVE_CUSTOM_ACCEL = False
    print("Warning: Custom accelerators not found. Building without them.")

# Parse command-line arguments
parser = argparse.ArgumentParser(
    description="Heterogeneous Spacecraft Multicore Processor Simulation"
)

# Binary to execute
parser.add_argument("binary", type=str, help="Path to the binary to execute")

# CPU configuration
parser.add_argument("--num-cores", type=int, default=4, 
                    help="Number of CPU cores")
parser.add_argument("--cpu_type", type=str, default="TimingSimpleCPU",
                    choices=["TimingSimpleCPU", "MinorCPU", "O3CPU", 
                             "AtomicSimpleCPU"],
                    help="CPU model type")

# Cache configuration
parser.add_argument("--l1i_size", type=str, default="32kB",
                    help="L1 instruction cache size")
parser.add_argument("--l1d_size", type=str, default="32kB",
                    help="L1 data cache size")
parser.add_argument("--l1i_assoc", type=int, default=2,
                    help="L1 instruction cache associativity")
parser.add_argument("--l1d_assoc", type=int, default=4,
                    help="L1 data cache associativity")
parser.add_argument("--l1i_tag_latency", type=int, default=1)
parser.add_argument("--l1d_tag_latency", type=int, default=2)
parser.add_argument("--l1i_data_latency", type=int, default=1)
parser.add_argument("--l1d_data_latency", type=int, default=2)
parser.add_argument("--l1i_response_latency", type=int, default=1)
parser.add_argument("--l1d_response_latency", type=int, default=2)
parser.add_argument("--l1i_mshrs", type=int, default=8)
parser.add_argument("--l1d_mshrs", type=int, default=16)
parser.add_argument("--l1i_tgts_per_mshr", type=int, default=16)
parser.add_argument("--l1d_tgts_per_mshr", type=int, default=32)
parser.add_argument("--l2_size", type=str, default="256kB",
                    help="L2 cache size")

# System configuration
parser.add_argument("--system_mem_range", type=str, default="1GB",
                    help="System memory range")
parser.add_argument("--clock_freq", type=str, default="2GHz",
                    help="System clock frequency")

# Branch predictor and prefetcher
parser.add_argument("--branch_pred", type=str, default="TOURNAMENT",
                    choices=["TAGE", "BiModeBP", "LTAGE", "TOURNAMENT"],
                    help="Branch predictor type")
parser.add_argument("--prefetcher", type=str, default="StridePrefetcher",
                    choices=["None", "StridePrefetcher", "TaggedPrefetcher"],
                    help="Prefetcher type")

# Accelerator configuration
parser.add_argument("--enable-compression", action="store_true",
                    help="Enable compression accelerator")
parser.add_argument("--enable-ml", action="store_true",
                    help="Enable ML accelerator")
parser.add_argument("--compression-k-param", type=int, default=4,
                    help="Rice k-parameter for compression")
parser.add_argument("--compression-block-size", type=int, default=64,
                    help="Block size for compression")
parser.add_argument("--ml-mac-rows", type=int, default=8,
                    help="Number of rows in ML MAC array")
parser.add_argument("--ml-mac-cols", type=int, default=8,
                    help="Number of columns in ML MAC array")
parser.add_argument("--ml-sram-size", type=str, default="64kB",
                    help="ML accelerator SRAM size")

args = parser.parse_args()

# Convert "None" string to None for prefetcher
if args.prefetcher == "None":
    args.prefetcher = None

# Create system
system = System()
system.mem_mode = "timing"

# Set up clock and voltage domains
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = args.clock_freq
system.clk_domain.voltage_domain = VoltageDomain(voltage="1.1V")

# Set memory range
system.mem_ranges = [AddrRange(args.system_mem_range)]

# CPU clock domain
system.cpu_clk_domain = SrcClockDomain()
system.cpu_clk_domain.clock = args.clock_freq
system.cpu_clk_domain.voltage_domain = VoltageDomain(voltage="1.1V")

# Create CPUs
print(f"Creating {args.num_cores} {args.cpu_type} cores...")
cpus = []
for i in range(args.num_cores):
    if args.cpu_type == "TimingSimpleCPU":
        cpu = RiscvTimingSimpleCPU(cpu_id=i)
    elif args.cpu_type == "MinorCPU":
        cpu = RiscvMinorCPU(cpu_id=i)
    elif args.cpu_type == "O3CPU":
        cpu = RiscvO3CPU(cpu_id=i)
    elif args.cpu_type == "AtomicSimpleCPU":
        cpu = RiscvAtomicSimpleCPU(cpu_id=i)
    
    # Set branch predictor
    if args.branch_pred == "TAGE":
        cpu.branchPred = TAGE()
    elif args.branch_pred == "BiModeBP":
        cpu.branchPred = BiModeBP()
    elif args.branch_pred == "LTAGE":
        cpu.branchPred = LTAGE()
    elif args.branch_pred == "TOURNAMENT":
        cpu.branchPred = TournamentBP()
    
    # Create L1 caches
    cpu.icache = L1ICache(args)
    cpu.dcache = L1DCache(args)
    
    # Add prefetcher if specified
    if args.prefetcher == "StridePrefetcher":
        cpu.dcache.prefetcher = StridePrefetcher()
        cpu.icache.prefetcher = StridePrefetcher()
    elif args.prefetcher == "TaggedPrefetcher":
        cpu.dcache.prefetcher = TaggedPrefetcher()
        cpu.icache.prefetcher = TaggedPrefetcher()
    
    # Connect caches to CPU
    cpu.icache.connectCPU(cpu)
    cpu.dcache.connectCPU(cpu)
    
    cpus.append(cpu)

# Assign CPUs to system
system.cpu = cpus

# Create L2 bus
system.l2bus = L2XBar()

# Connect L1 caches to L2 bus
for cpu in system.cpu:
    cpu.icache.connectBus(system.l2bus)
    cpu.dcache.connectBus(system.l2bus)

# Create L2 cache
system.l2cache = L2Cache(args)
system.l2cache.connectCPUSideBus(system.l2bus)

# Create main memory bus
system.membus = SystemXBar()
system.l2cache.connectMemSideBus(system.membus)

# Create accelerators if available and enabled
# NOTE: For SE mode, the accelerators add MMIO devices. The workload must
# explicitly access these addresses. The hardware-accelerated workloads
# using RISC-V matrix instructions do NOT need these accelerators enabled.
if HAVE_CUSTOM_ACCEL:
    if args.enable_compression:
        print("Creating Compression Core...")
        system.compression_core = CompressionCore(
            addr_range=AddrRange(0x10000000, size='256B'),
            k_param=args.compression_k_param,
            block_size=args.compression_block_size,
            compression_latency=10
        )
        # Connect compression core to memory bus
        # cpu_side (ResponsePort) receives MMIO from CPUs - connects to bus mem_side
        # mem_side (RequestPort) sends DMA to memory - connects to bus cpu_side
        system.compression_core.cpu_side = system.membus.mem_side_ports
        # For DMA, accelerator needs access to memory through l2bus (before l2cache)
        system.compression_core.mem_side = system.l2bus.cpu_side_ports
        print(f"  - Address: 0x10000000")
        print(f"  - K-parameter: {args.compression_k_param}")
        print(f"  - Block size: {args.compression_block_size}")
    
    if args.enable_ml:
        print("Creating TinyML Accelerator...")
        system.ml_accel = TinyMLAccel(
            addr_range=AddrRange(0x10001000, size='256B'),
            mac_array_rows=args.ml_mac_rows,
            mac_array_cols=args.ml_mac_cols,
            sram_size=args.ml_sram_size,
            mac_latency=1,
            dma_latency=5
        )
        # Connect ML accelerator to memory bus
        # cpu_side (ResponsePort) receives MMIO from CPUs - connects to bus mem_side
        # mem_side (RequestPort) sends DMA to memory - connects to bus cpu_side
        system.ml_accel.cpu_side = system.membus.mem_side_ports
        # For DMA, accelerator needs access to memory through l2bus (before l2cache)
        system.ml_accel.mem_side = system.l2bus.cpu_side_ports
        print(f"  - Address: 0x10001000")
        print(f"  - MAC Array: {args.ml_mac_rows}x{args.ml_mac_cols}")
        print(f"  - SRAM: {args.ml_sram_size}")
        print(f"  - Total MACs: {args.ml_mac_rows * args.ml_mac_cols}")
else:
    if args.enable_compression or args.enable_ml:
        print("Warning: Custom accelerators requested but not available.")
        print("Please rebuild gem5 with custom_accel sources.")

# Create memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# Create interrupt controllers
for cpu in system.cpu:
    cpu.createInterruptController()

# Connect system port
system.system_port = system.membus.cpu_side_ports

# Set workload
system.workload = SEWorkload.init_compatible(args.binary)

# Create process
process = Process()
process.cmd = [args.binary]

# Assign workload to CPUs
for cpu in system.cpu:
    cpu.workload = process
    cpu.createThreads()

# Enable tracing (optional)
from m5.objects import SimpleTrace
for cpu in system.cpu:
    cpu.trace = SimpleTrace()
    cpu.icache.trace = SimpleTrace()
    cpu.dcache.trace = SimpleTrace()
system.l2cache.trace = SimpleTrace()

# Create root and instantiate
root = Root(full_system=False, system=system)
m5.instantiate()

# Print configuration summary
print("\n" + "="*60)
print("SPACECRAFT HETEROGENEOUS PROCESSOR CONFIGURATION")
print("="*60)
print(f"CPU Cores:        {args.num_cores} x {args.cpu_type}")
print(f"Clock Frequency:  {args.clock_freq}")
print(f"L1I Cache:        {args.l1i_size} (assoc={args.l1i_assoc})")
print(f"L1D Cache:        {args.l1d_size} (assoc={args.l1d_assoc})")
print(f"L2 Cache:         {args.l2_size}")
print(f"System Memory:    {args.system_mem_range}")
print(f"Branch Predictor: {args.branch_pred}")
print(f"Prefetcher:       {args.prefetcher or 'None'}")
if HAVE_CUSTOM_ACCEL:
    print(f"Compression Core: {'Enabled' if args.enable_compression else 'Disabled'}")
    print(f"ML Accelerator:   {'Enabled' if args.enable_ml else 'Disabled'}")
print("="*60 + "\n")

# Run simulation
print("Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")


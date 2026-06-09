"""
Spacecraft DLS SoC - SE Mode with Accelerators
PhD Research: Chandraboul

Configuration for testing accelerators in SE mode using
memory-mapped uncached regions.
"""

import argparse
import sys
from os import path

import m5
from m5.objects import *
from m5.util import addToPath

# =============================================================================
# Try to import custom accelerators
# =============================================================================

HAVE_CUSTOM_ACCEL = True
try:
    from m5.objects import TinyML32PE
    from m5.objects import CompressionCore
    from m5.objects import CCSDSTmTc
    from m5.objects import CCSDSImageComp
    from m5.objects import SpaceWireNIC
    from m5.objects import TSNEthernet
    from m5.objects import LockstepChecker
    from m5.objects import CXLInterface
    from m5.objects import DebugInterface
    print("Custom accelerators loaded successfully")
except ImportError as e:
    HAVE_CUSTOM_ACCEL = False
    print(f"Warning: Custom accelerators not available: {e}")

# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(description="Spacecraft SoC with Accelerators")
parser.add_argument("--binary", type=str, required=True, help="RISC-V binary")
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                    choices=["AtomicSimpleCPU", "TimingSimpleCPU", "MinorCPU"])
parser.add_argument("--num-cpus", type=int, default=1, help="Number of CPUs")
parser.add_argument("--mem-size", type=str, default="512MB", help="Memory size")

# Accelerator flags
parser.add_argument("--enable-ml", action="store_true", help="Enable TinyML 32PE")
parser.add_argument("--enable-compression", action="store_true", help="Enable Compression Core")
parser.add_argument("--enable-tmtc", action="store_true", help="Enable CCSDS TM/TC")
parser.add_argument("--enable-image-comp", action="store_true", help="Enable Image Compression")
parser.add_argument("--enable-spacewire", action="store_true", help="Enable SpaceWire")
parser.add_argument("--enable-tsn", action="store_true", help="Enable TSN Ethernet")
parser.add_argument("--enable-cxl", action="store_true", help="Enable CXL Interface")
parser.add_argument("--enable-debug", action="store_true", help="Enable Debug Interface")
parser.add_argument("--enable-lockstep", action="store_true", help="Enable Lockstep Checker")
parser.add_argument("--enable-all-accel", action="store_true", help="Enable all accelerators")

args = parser.parse_args()

if args.enable_all_accel:
    args.enable_ml = True
    args.enable_compression = True
    args.enable_tmtc = True
    args.enable_image_comp = True
    args.enable_spacewire = True
    args.enable_tsn = True
    args.enable_cxl = True
    args.enable_debug = True
    args.enable_lockstep = True

# =============================================================================
# Memory Map
# =============================================================================

# Main memory starts at 0x80000000 (RISC-V convention)
MAIN_MEM_BASE = 0x80000000

# Accelerator base addresses (MMIO region)
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

# =============================================================================
# System Setup
# =============================================================================

print("\n" + "="*60)
print("SPACECRAFT SOC - SE MODE WITH ACCELERATORS")
print("="*60)

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing"

# Memory range - use standard RISC-V base address
system.mem_ranges = [AddrRange(MAIN_MEM_BASE, size=args.mem_size)]

# =============================================================================
# CPU Creation
# =============================================================================

if args.cpu_type == "AtomicSimpleCPU":
    system.cpu = [RiscvAtomicSimpleCPU() for i in range(args.num_cpus)]
    system.mem_mode = "atomic"
elif args.cpu_type == "TimingSimpleCPU":
    system.cpu = [RiscvTimingSimpleCPU() for i in range(args.num_cpus)]
elif args.cpu_type == "MinorCPU":
    system.cpu = [RiscvMinorCPU() for i in range(args.num_cpus)]

print(f"CPUs: {args.num_cpus} x {args.cpu_type}")

# =============================================================================
# Memory System
# =============================================================================

system.membus = SystemXBar()

# Connect CPUs to membus
for cpu in system.cpu:
    cpu.icache_port = system.membus.cpu_side_ports
    cpu.dcache_port = system.membus.cpu_side_ports

# Memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports

# =============================================================================
# Accelerator Configuration
# =============================================================================

accel_count = 0

if HAVE_CUSTOM_ACCEL:
    print("\nConfiguring Accelerators:")
    print("-" * 60)
    
    # Create IO bus for accelerators
    system.iobus = IOXBar()
    
    # Create bridge from membus to iobus
    system.bridge = Bridge(delay='10ns')
    system.bridge.mem_side_port = system.iobus.cpu_side_ports
    system.bridge.cpu_side_port = system.membus.mem_side_ports
    
    # List to collect all accelerator address ranges
    accel_addr_ranges = []
    
    # TinyML 32PE Accelerator
    if args.enable_ml:
        print(f"  TinyML 32PE     @ 0x{ML_32PE_BASE:08X}")
        system.ml_accel = TinyML32PE(
            addr_range=AddrRange(ML_32PE_BASE, size='64kB'),
            global_buffer_size="128kB",
            weight_buffer_size="64kB",
            accum_buffer_size="32kB"
        )
        system.ml_accel.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(ML_32PE_BASE, size='64kB'))
        accel_count += 1
    
    # Compression Core
    if args.enable_compression:
        print(f"  Compression     @ 0x{COMPRESS_BASE:08X}")
        system.compress = CompressionCore(
            addr_range=AddrRange(COMPRESS_BASE, size='4kB')
        )
        system.compress.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(COMPRESS_BASE, size='4kB'))
        accel_count += 1
    
    # CCSDS TM/TC
    if args.enable_tmtc:
        print(f"  CCSDS TM/TC     @ 0x{TMTC_BASE:08X}")
        system.tmtc = CCSDSTmTc(
            addr_range=AddrRange(TMTC_BASE, size='4kB')
        )
        system.tmtc.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(TMTC_BASE, size='4kB'))
        accel_count += 1
    
    # CCSDS Image Compression
    if args.enable_image_comp:
        print(f"  Image Compress  @ 0x{IMAGE_COMP_BASE:08X}")
        system.image_comp = CCSDSImageComp(
            addr_range=AddrRange(IMAGE_COMP_BASE, size='4kB')
        )
        system.image_comp.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(IMAGE_COMP_BASE, size='4kB'))
        accel_count += 1
    
    # SpaceWire (single controller, 4 links in hardware)
    if args.enable_spacewire:
        print(f"  SpaceWire NIC   @ 0x{SPACEWIRE_BASE:08X}")
        system.spacewire = SpaceWireNIC(
            addr_range=AddrRange(SPACEWIRE_BASE, size='4kB'),
            link_speed=200  # 200 Mbps
        )
        system.spacewire.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(SPACEWIRE_BASE, size='4kB'))
        accel_count += 1
    
    # TSN Ethernet (single controller)
    if args.enable_tsn:
        print(f"  TSN Ethernet    @ 0x{TSN_BASE:08X}")
        system.tsn = TSNEthernet(
            addr_range=AddrRange(TSN_BASE, size='4kB'),
            link_speed=1000  # 1 Gbps
        )
        system.tsn.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(TSN_BASE, size='4kB'))
        accel_count += 1
    
    # CXL Interface
    if args.enable_cxl:
        print(f"  CXL Interface   @ 0x{CXL_BASE:08X}")
        system.cxl = CXLInterface(
            addr_range=AddrRange(CXL_BASE, size='4kB'),
            link_speed=32,  # 32 GT/s (CXL 2.0)
            link_width=16   # x16 link
        )
        system.cxl.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(CXL_BASE, size='4kB'))
        accel_count += 1
    
    # Debug Interface
    if args.enable_debug:
        print(f"  Debug Interface @ 0x{DEBUG_BASE:08X}")
        system.debug_if = DebugInterface(
            addr_range=AddrRange(DEBUG_BASE, size='4kB'),
            num_breakpoints=8,
            num_cores=args.num_cpus
        )
        system.debug_if.cpu_side = system.iobus.mem_side_ports
        accel_addr_ranges.append(AddrRange(DEBUG_BASE, size='4kB'))
        accel_count += 1
    
    # DLS Lockstep Checker
    if args.enable_lockstep:
        print(f"  Lockstep Checker@ 0x{LOCKSTEP_BASE:08X}")
        system.lockstep = LockstepChecker(
            addr_range=AddrRange(LOCKSTEP_BASE, size='4kB'),
            num_pairs=4,
            enable_recovery=True
        )
        # Note: LockstepChecker uses cpu_port (VectorResponsePort) not cpu_side
        # We skip the port connection for now since it needs special handling
        accel_addr_ranges.append(AddrRange(LOCKSTEP_BASE, size='4kB'))
        accel_count += 1
    
    # Configure bridge ranges if we have accelerators
    if accel_addr_ranges:
        system.bridge.ranges = accel_addr_ranges
        print(f"\n  Total accelerators: {accel_count}")
    else:
        print("\n  No accelerators enabled")
else:
    print("\nAccelerators not available in this build")

# =============================================================================
# Interrupt Controllers
# =============================================================================

for cpu in system.cpu:
    cpu.createInterruptController()

# =============================================================================
# Workload
# =============================================================================

system.workload = SEWorkload.init_compatible(args.binary)

process = Process()
process.cmd = [args.binary]

# Note: In SE mode, accelerator MMIO addresses are not directly accessible
# to user-space programs. For full accelerator testing, use FS mode.
# The accelerators are configured and will respond to accesses routed
# through the proper memory hierarchy.

for cpu in system.cpu:
    cpu.workload = process
    cpu.createThreads()

# =============================================================================
# Run
# =============================================================================

root = Root(full_system=False, system=system)
m5.instantiate()

print("\n" + "="*60)
print(f"Binary: {args.binary}")
print(f"Memory: {args.mem_size} @ 0x{MAIN_MEM_BASE:08X}")
print(f"Accelerators: {accel_count}")
print("="*60)
print("\nBeginning simulation!")

exit_event = m5.simulate()

print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")


"""
Spacecraft DLS SoC - Full System Configuration with All Accelerators
PhD Research: Chandraboul

Full system simulation with:
- 8-core RISC-V RV64GC CPUs (DLS configuration)
- All custom accelerators (TinyML, CCSDS, SpaceWire, TSN, etc.)
- Linux kernel support
- MMIO accelerator access

Usage:
    ./build/RISCV/gem5.opt configs/example/riscv/spacecraft_fs_accel.py \
        --kernel=<path-to-kernel> \
        --disk-image=<path-to-disk-image> \
        --enable-all-accel
"""

import argparse
import sys
from os import path

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn
from m5.util.fdthelper import *

from gem5.utils.requires import requires

addToPath("../../")

from common import (
    CacheConfig,
    CpuConfig,
    MemConfig,
    ObjectList,
    Options,
    Simulation,
)
from common.Benchmarks import *
from common.Caches import *
from common.FSConfig import *
from common.SysPaths import *

# Import custom accelerators
HAVE_CUSTOM_ACCEL = True
HAVE_MATRIX_ACCEL = True
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

try:
    from m5.objects import MatrixTileAccel
    print("MatrixTileAccel (DianNao-style) loaded successfully")
except ImportError as e:
    HAVE_MATRIX_ACCEL = False
    print(f"Warning: MatrixTileAccel not available: {e}")

HAVE_TINY_TPU = True
try:
    from m5.objects import TinyTPU
    print("TinyTPU (TPU-style systolic array) loaded successfully")
except ImportError as e:
    HAVE_TINY_TPU = False
    print(f"Warning: TinyTPU not available: {e}")

# Shared accelerators for contention study
HAVE_SHARED_ACCEL = True
try:
    from m5.objects.SharedMatrixAccel import SharedMatrixAccel
    from m5.objects.SharedCORDICAccel import SharedCORDICAccel
    print("Shared accelerators (Matrix/CORDIC) loaded successfully")
except ImportError as e:
    HAVE_SHARED_ACCEL = False
    print(f"Warning: Shared accelerators not available: {e}")

requires(isa_required=ISA.RISCV)

# =============================================================================
# Memory Map for Accelerators
# =============================================================================

# Use address range starting at 0x20000000 to avoid HiFive platform conflicts
ACCEL_BASE       = 0x20000000
ML_32PE_BASE     = ACCEL_BASE + 0x00000
COMPRESS_BASE    = ACCEL_BASE + 0x10000
TMTC_BASE        = ACCEL_BASE + 0x20000
IMAGE_COMP_BASE  = ACCEL_BASE + 0x30000
SPACEWIRE_BASE   = ACCEL_BASE + 0x40000
TSN_BASE         = ACCEL_BASE + 0x50000
CXL_BASE         = ACCEL_BASE + 0x60000
DEBUG_BASE       = ACCEL_BASE + 0x70000
LOCKSTEP_BASE    = ACCEL_BASE + 0x80000
MATRIX_TILE_BASE = ACCEL_BASE + 0x90000  # DianNao-style Matrix Tile Accelerator
TINY_TPU_BASE    = ACCEL_BASE + 0xA0000  # TinyTPU - TPU-style systolic array
SHARED_MATRIX_BASE = ACCEL_BASE + 0xB0000  # Shared 3x3 Matrix Accelerator with arbitration
SHARED_CORDIC_BASE = ACCEL_BASE + 0xC0000  # Shared CORDIC Accelerator with arbitration

# =============================================================================
# DTB Generation with Accelerators
# =============================================================================

def generateMemNode(state, mem_range):
    """Generate memory node for device tree."""
    node = FdtNode(f"memory@{int(mem_range.start):x}")
    node.append(FdtPropertyStrings("device_type", ["memory"]))
    node.append(
        FdtPropertyWords(
            "reg",
            state.addrCells(mem_range.start)
            + state.sizeCells(mem_range.size()),
        )
    )
    return node


def generateAcceleratorNode(state, name, base_addr, size, compatible):
    """Generate device tree node for custom accelerator."""
    node = FdtNode(f"{name}@{base_addr:x}")
    node.append(FdtPropertyStrings("compatible", [compatible]))
    node.append(
        FdtPropertyWords(
            "reg",
            state.addrCells(base_addr) + state.sizeCells(size),
        )
    )
    node.append(FdtPropertyStrings("status", ["okay"]))
    return node


def generateDtb(system, args, accel_ranges):
    """Generate device tree blob with accelerator support."""
    state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
    root = FdtNode("/")
    root.append(state.addrCellsProperty())
    root.append(state.sizeCellsProperty())
    root.appendCompatible(["riscv-virtio", "spacecraft-dls-soc"])

    # Memory nodes
    for mem_range in system.mem_ranges:
        root.append(generateMemNode(state, mem_range))

    # Standard platform sections
    sections = [*system.cpu, system.platform]
    for section in sections:
        for node in section.generateDeviceTree(state):
            if node.get_name() == root.get_name():
                root.merge(node)
            else:
                root.append(node)

    # Add accelerator nodes
    if args.enable_ml:
        node = generateAcceleratorNode(state, "tinyml-32pe", ML_32PE_BASE, 
                                       0x10000, "spacecraft,tinyml-32pe")
        root.append(node)
        print("  Added tinyml-32pe to device tree")

    if args.enable_compression:
        node = generateAcceleratorNode(state, "compression-core", COMPRESS_BASE,
                                       0x1000, "spacecraft,compression-core")
        root.append(node)
        print("  Added compression-core to device tree")

    if args.enable_tmtc:
        node = generateAcceleratorNode(state, "ccsds-tmtc", TMTC_BASE,
                                       0x1000, "spacecraft,ccsds-tmtc")
        root.append(node)
        print("  Added ccsds-tmtc to device tree")

    if args.enable_image_comp:
        node = generateAcceleratorNode(state, "ccsds-image-comp", IMAGE_COMP_BASE,
                                       0x1000, "spacecraft,ccsds-image-comp")
        root.append(node)
        print("  Added ccsds-image-comp to device tree")

    if args.enable_spacewire:
        node = generateAcceleratorNode(state, "spacewire-nic", SPACEWIRE_BASE,
                                       0x1000, "spacecraft,spacewire-nic")
        root.append(node)
        print("  Added spacewire-nic to device tree")

    if args.enable_tsn:
        node = generateAcceleratorNode(state, "tsn-ethernet", TSN_BASE,
                                       0x1000, "spacecraft,tsn-ethernet")
        root.append(node)
        print("  Added tsn-ethernet to device tree")

    if args.enable_cxl:
        node = generateAcceleratorNode(state, "cxl-interface", CXL_BASE,
                                       0x1000, "spacecraft,cxl-interface")
        root.append(node)
        print("  Added cxl-interface to device tree")

    if args.enable_debug:
        node = generateAcceleratorNode(state, "debug-interface", DEBUG_BASE,
                                       0x1000, "spacecraft,debug-interface")
        root.append(node)
        print("  Added debug-interface to device tree")

    if args.enable_lockstep:
        node = generateAcceleratorNode(state, "lockstep-checker", LOCKSTEP_BASE,
                                       0x1000, "spacecraft,lockstep-checker")
        root.append(node)
        print("  Added lockstep-checker to device tree")

    fdt = Fdt()
    fdt.add_rootnode(root)
    fdt.writeDtsFile(path.join(m5.options.outdir, "device.dts"))
    fdt.writeDtbFile(path.join(m5.options.outdir, "device.dtb"))


# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(
    description="Spacecraft DLS SoC - Full System with Accelerators"
)

Options.addCommonOptions(parser, ISA.RISCV)
Options.addFSOptions(parser)

parser.add_argument("--virtio-rng", action="store_true",
                    help="Enable VirtIO entropy source")

# Accelerator flags
parser.add_argument("--enable-ml", action="store_true")
parser.add_argument("--enable-compression", action="store_true")
parser.add_argument("--enable-tmtc", action="store_true")
parser.add_argument("--enable-image-comp", action="store_true")
parser.add_argument("--enable-spacewire", action="store_true")
parser.add_argument("--enable-tsn", action="store_true")
parser.add_argument("--enable-cxl", action="store_true")
parser.add_argument("--enable-debug", action="store_true")
parser.add_argument("--enable-lockstep", action="store_true")
parser.add_argument("--enable-all-accel", action="store_true",
                    help="Enable all accelerators")

args = parser.parse_args()

# Handle enable-all-accel
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
# System Setup
# =============================================================================

print("\n" + "="*70)
print("SPACECRAFT DLS SOC - FULL SYSTEM MODE WITH ACCELERATORS")
print("="*70)

# For multicore with atomics, use O3CPU or DerivO3CPU for proper LR/SC handling
# TimingSimpleCPU and AtomicSimpleCPU have issues with atomic operations
if args.num_cpus > 1:
    # Try O3CPU first, fall back to Timing if not available
    try:
        from m5.objects import RiscvO3CPU
        args.cpu_type = "O3CPU"
        print(f"\nNote: Using O3CPU for proper multicore atomic handling")
    except ImportError:
        args.cpu_type = "TimingSimpleCPU"
        print(f"\nNote: O3CPU not available, using TimingSimpleCPU (may have SC warnings)")

(CPUClass, mem_mode, FutureClass) = Simulation.setCPUClass(args)
assert issubclass(CPUClass, RiscvCPU)
MemClass = Simulation.setMemClass(args)

np = args.num_cpus
print(f"\nNumber of CPUs: {np}")

# Create system
system = System()
mdesc = SysConfig(
    disks=args.disk_image,
    rootdev=args.root_device,
    mem=args.mem_size,
    os_type=args.os_type,
)

system.mem_mode = mem_mode
system.mem_ranges = [AddrRange(start=0x80000000, size=mdesc.mem())]

# Workload
system.workload = RiscvLinux()
system.workload.object_file = args.kernel

# Create buses
system.iobus = IOXBar()
system.membus = MemBus()
system.system_port = system.membus.cpu_side_ports

# HiFive Platform
system.platform = HiFive()
system.platform.rtc = RiscvRTC(frequency=Frequency("100MHz"))
system.platform.clint.int_pin = system.platform.rtc.int_pin
system.platform.pci_host.pio = system.iobus.mem_side_ports

# =============================================================================
# Custom Accelerators
# =============================================================================

accel_ranges = []
accel_count = 0

if HAVE_CUSTOM_ACCEL:
    print("\nConfiguring Accelerators:")
    print("-" * 70)
    
    system.accel_bus = IOXBar()
    
    if args.enable_ml:
        print(f"  TinyML 32PE     @ 0x{ML_32PE_BASE:08X}")
        system.ml_accel = TinyML32PE(
            addr_range=AddrRange(ML_32PE_BASE, size='64kB'),
            global_buffer_size="128kB",
            weight_buffer_size="64kB",
            accum_buffer_size="32kB"
        )
        system.ml_accel.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(ML_32PE_BASE, size='64kB'))
        accel_count += 1

    if args.enable_compression:
        print(f"  Compression     @ 0x{COMPRESS_BASE:08X}")
        system.compress = CompressionCore(
            addr_range=AddrRange(COMPRESS_BASE, size='4kB')
        )
        system.compress.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(COMPRESS_BASE, size='4kB'))
        accel_count += 1

    if args.enable_tmtc:
        print(f"  CCSDS TM/TC     @ 0x{TMTC_BASE:08X}")
        system.tmtc = CCSDSTmTc(
            addr_range=AddrRange(TMTC_BASE, size='4kB')
        )
        system.tmtc.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(TMTC_BASE, size='4kB'))
        accel_count += 1

    if args.enable_image_comp:
        print(f"  Image Compress  @ 0x{IMAGE_COMP_BASE:08X}")
        system.image_comp = CCSDSImageComp(
            addr_range=AddrRange(IMAGE_COMP_BASE, size='4kB')
        )
        system.image_comp.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(IMAGE_COMP_BASE, size='4kB'))
        accel_count += 1

    if args.enable_spacewire:
        print(f"  SpaceWire NIC   @ 0x{SPACEWIRE_BASE:08X}")
        system.spacewire = SpaceWireNIC(
            addr_range=AddrRange(SPACEWIRE_BASE, size='4kB'),
            link_speed=200
        )
        system.spacewire.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(SPACEWIRE_BASE, size='4kB'))
        accel_count += 1

    if args.enable_tsn:
        print(f"  TSN Ethernet    @ 0x{TSN_BASE:08X}")
        system.tsn = TSNEthernet(
            addr_range=AddrRange(TSN_BASE, size='4kB'),
            link_speed=1000
        )
        system.tsn.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(TSN_BASE, size='4kB'))
        accel_count += 1

    if args.enable_cxl:
        print(f"  CXL Interface   @ 0x{CXL_BASE:08X}")
        system.cxl = CXLInterface(
            addr_range=AddrRange(CXL_BASE, size='4kB'),
            link_speed=32,
            link_width=16
        )
        system.cxl.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(CXL_BASE, size='4kB'))
        accel_count += 1

    if args.enable_debug:
        print(f"  Debug Interface @ 0x{DEBUG_BASE:08X}")
        system.debug_if = DebugInterface(
            addr_range=AddrRange(DEBUG_BASE, size='4kB'),
            num_breakpoints=8,
            num_cores=np
        )
        system.debug_if.cpu_side = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(DEBUG_BASE, size='4kB'))
        accel_count += 1

    if args.enable_lockstep:
        print(f"  Lockstep Checker@ 0x{LOCKSTEP_BASE:08X}")
        system.lockstep = LockstepChecker(
            addr_range=AddrRange(LOCKSTEP_BASE, size='4kB'),
            num_pairs=4,
            enable_recovery=True
        )
        accel_ranges.append(AddrRange(LOCKSTEP_BASE, size='4kB'))
        accel_count += 1

    # DianNao-style Matrix Tile Accelerator (always enabled with enable-all-accel)
    if HAVE_MATRIX_ACCEL and args.enable_all_accel:
        print(f"  Matrix Tile Acc @ 0x{MATRIX_TILE_BASE:08X} (DianNao-style)")
        system.matrix_accel = MatrixTileAccel(
            addr_range=AddrRange(MATRIX_TILE_BASE, size='4kB'),
            input_buffer_size='4kB',
            weight_buffer_size='4kB',
            accum_buffer_size='2kB',
            output_buffer_size='2kB',
            mac_latency=3,
            dma_latency=10
        )
        system.matrix_accel.cpu_side = system.accel_bus.mem_side_ports
        system.matrix_accel.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(MATRIX_TILE_BASE, size='4kB'))
        accel_count += 1
        print("    - Input Buffer:  4KB")
        print("    - Weight Buffer: 4KB")
        print("    - Accum Buffer:  2KB")
        print("    - MAC Latency:   3 cycles")
        print("    - DMA Latency:   10 cycles")

    # TinyTPU - TPU-style systolic array accelerator
    if HAVE_TINY_TPU and args.enable_all_accel:
        print(f"  TinyTPU          @ 0x{TINY_TPU_BASE:08X} (TPU-style)")
        system.tiny_tpu = TinyTPU(
            addr_range=AddrRange(TINY_TPU_BASE, size='4kB'),
            unified_buffer_size='64kB',
            weight_fifo_size='32kB',
            accum_buffer_size='4kB',
            mxu_latency=1,
            dma_latency=5
        )
        system.tiny_tpu.cpu_side = system.accel_bus.mem_side_ports
        system.tiny_tpu.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(TINY_TPU_BASE, size='4kB'))
        accel_count += 1
        print("    - MXU Size:           8x8 systolic array")
        print("    - Unified Buffer:     64KB")
        print("    - Weight FIFO:        32KB")
        print("    - Accum Buffer:       4KB")
        print("    - MXU Latency:        1 cycle")
        print("    - DMA Latency:        5 cycles")

    # Shared accelerators for contention study
    if HAVE_SHARED_ACCEL and args.enable_all_accel:
        print(f"  Shared Matrix    @ 0x{SHARED_MATRIX_BASE:08X} (with arbitration)")
        system.shared_matrix = SharedMatrixAccel(
            num_cores=args.num_cpus,
            compute_latency=10,
            arbitration_latency=2,
            mmio_base=SHARED_MATRIX_BASE,
            mmio_size=0x1000
        )
        system.shared_matrix.mmio_port = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(SHARED_MATRIX_BASE, size='4kB'))
        accel_count += 1
        print(f"    - Compute Latency:    10 cycles")
        print(f"    - Arbitration Latency: 2 cycles")
        
        print(f"  Shared CORDIC    @ 0x{SHARED_CORDIC_BASE:08X} (with arbitration)")
        system.shared_cordic = SharedCORDICAccel(
            num_cores=args.num_cpus,
            compute_latency=5,
            arbitration_latency=1,
            mmio_base=SHARED_CORDIC_BASE,
            mmio_size=0x1000
        )
        system.shared_cordic.mmio_port = system.accel_bus.mem_side_ports
        accel_ranges.append(AddrRange(SHARED_CORDIC_BASE, size='4kB'))
        accel_count += 1
        print(f"    - Compute Latency:    5 cycles")
        print(f"    - Arbitration Latency: 1 cycle")

    # Create bridge from IO bus to accelerator bus
    if accel_ranges:
        system.accel_bridge = Bridge(delay="10ns")
        system.accel_bridge.cpu_side_port = system.iobus.mem_side_ports
        system.accel_bridge.mem_side_port = system.accel_bus.cpu_side_ports
        system.accel_bridge.ranges = accel_ranges
        print(f"\n  Total accelerators: {accel_count}")
else:
    print("\nWarning: Custom accelerators not available in build")

# =============================================================================
# VirtIO Devices
# =============================================================================

if args.disk_image:
    image = CowDiskImage(child=RawDiskImage(read_only=True), read_only=False)
    image.child.image_file = mdesc.disks()[0]
    system.platform.disk = RiscvMmioVirtIO(
        vio=VirtIOBlock(image=image),
        interrupt_id=0x8,
        pio_size=4096,
        pio_addr=0x10008000,
    )

if args.virtio_rng:
    system.platform.rng = RiscvMmioVirtIO(
        vio=VirtIORng(),
        interrupt_id=0x8,
        pio_size=4096,
        pio_addr=0x10007000
    )

# =============================================================================
# Bridge Configuration
# =============================================================================

system.bridge = Bridge(delay="50ns")
system.bridge.mem_side_port = system.iobus.cpu_side_ports
system.bridge.cpu_side_port = system.membus.mem_side_ports
system.bridge.ranges = system.platform._off_chip_ranges()

system.platform.attachOnChipIO(system.membus)
system.platform.attachOffChipIO(system.iobus)
system.platform.attachPlic()
system.platform.setNumCores(np)

# =============================================================================
# Clock and Voltage
# =============================================================================

system.cache_line_size = args.cacheline_size
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock,
    voltage_domain=system.voltage_domain
)
system.cpu_voltage_domain = VoltageDomain()
system.cpu_clk_domain = SrcClockDomain(
    clock=args.cpu_clock,
    voltage_domain=system.cpu_voltage_domain
)

# =============================================================================
# CPU Configuration
# =============================================================================

print(f"\nConfiguring {np} {CPUClass.__name__} CPUs...")

system.cpu = [
    CPUClass(clk_domain=system.cpu_clk_domain, cpu_id=i)
    for i in range(np)
]

# Cache configuration
if args.caches or args.l2cache:
    system.iocache = IOCache(addr_ranges=system.mem_ranges)
    system.iocache.cpu_side = system.iobus.mem_side_ports
    system.iocache.mem_side = system.membus.cpu_side_ports
elif not args.external_memory_system:
    system.iobridge = Bridge(delay="50ns", ranges=system.mem_ranges)
    system.iobridge.cpu_side_port = system.iobus.mem_side_ports
    system.iobridge.mem_side_port = system.membus.cpu_side_ports

for i in range(np):
    if args.bp_type:
        bpClass = ObjectList.bp_list.get(args.bp_type)
        system.cpu[i].branchPred = bpClass()
    system.cpu[i].createThreads()

# =============================================================================
# PMA Checker
# =============================================================================

uncacheable_range = [
    *system.platform._on_chip_ranges(),
    *system.platform._off_chip_ranges(),
    *accel_ranges,
]

for cpu in system.cpu:
    cpu.mmu.pma_checker = PMAChecker(uncacheable=uncacheable_range)

# =============================================================================
# Device Tree Generation
# =============================================================================

print("\nGenerating device tree...")
if args.dtb_filename:
    system.workload.dtb_filename = args.dtb_filename
else:
    generateDtb(system, args, accel_ranges)
    system.workload.dtb_filename = path.join(m5.options.outdir, "device.dtb")

system.workload.dtb_addr = 0x87E00000

if args.command_line:
    system.workload.command_line = args.command_line
else:
    kernel_cmd = ["console=ttyS0", "root=/dev/vda", "ro"]
    system.workload.command_line = " ".join(kernel_cmd)

# =============================================================================
# Cache and Memory
# =============================================================================

CacheConfig.config_cache(args, system)
MemConfig.config_mem(args, system)

# =============================================================================
# Run Simulation
# =============================================================================

print("\n" + "="*70)
print("SYSTEM CONFIGURATION SUMMARY")
print("="*70)
print(f"CPUs:               {np} x {CPUClass.__name__}")
print(f"Memory:             {args.mem_size}")
print(f"Kernel:             {args.kernel}")
print(f"Disk Image:         {args.disk_image or 'None'}")
print(f"Accelerators:       {accel_count}")
print("="*70 + "\n")

root = Root(full_system=True, system=system)

Simulation.setWorkCountOptions(system, args)
Simulation.run(args, root, system, FutureClass)


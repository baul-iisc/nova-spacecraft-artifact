"""
Spacecraft Heterogeneous Processor - Full System Configuration
PhD Research: Chandraboul

This configuration creates a full system RISC-V simulation with:
- Multiple RISC-V CPU cores
- Compression Core accelerator for telemetry data
- TinyML Accelerator for on-board AI
- Linux kernel support

Usage:
    build/RISCV/gem5.opt configs/example/riscv/spacecraft_fs.py \
        --kernel=<path-to-bbl-with-linux> \
        --disk-image=<path-to-disk-image> \
        --num-cpus=4 \
        --enable-compression \
        --enable-ml
"""

import argparse
import sys
from os import path

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import (
    addToPath,
    fatal,
    warn,
)
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
try:
    from m5.objects import CompressionCore, TinyMLAccel
    HAVE_CUSTOM_ACCEL = True
except ImportError:
    HAVE_CUSTOM_ACCEL = False
    print("Warning: Custom accelerators not found in build.")

# Require RISC-V ISA
requires(isa_required=ISA.RISCV)


# =====================================================================
# DTB Generation with Accelerators
# =====================================================================

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


def generateDtb(system, args):
    """Generate device tree blob with accelerator support."""
    state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
    root = FdtNode("/")
    root.append(state.addrCellsProperty())
    root.append(state.sizeCellsProperty())
    root.appendCompatible(["riscv-virtio", "spacecraft-soc"])

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

    # Add accelerator nodes if enabled (using 0x20000000 base to avoid HiFive conflicts)
    if hasattr(args, 'enable_compression') and args.enable_compression:
        compress_node = generateAcceleratorNode(
            state, 
            "compression-core", 
            0x20000000, 
            0x100,
            "spacecraft,compression-core"
        )
        root.append(compress_node)
        print("  Added compression-core to device tree")

    if hasattr(args, 'enable_ml') and args.enable_ml:
        ml_node = generateAcceleratorNode(
            state,
            "ml-accelerator",
            0x20001000,
            0x100,
            "spacecraft,tinyml-accel"
        )
        root.append(ml_node)
        print("  Added ml-accelerator to device tree")

    fdt = Fdt()
    fdt.add_rootnode(root)
    fdt.writeDtsFile(path.join(m5.options.outdir, "device.dts"))
    fdt.writeDtbFile(path.join(m5.options.outdir, "device.dtb"))


# =====================================================================
# Argument Parser
# =====================================================================

parser = argparse.ArgumentParser(
    description="Spacecraft Heterogeneous Processor Full System Simulation"
)

# Standard gem5 options
Options.addCommonOptions(parser, ISA.RISCV)
Options.addFSOptions(parser)

# Standard FS options
parser.add_argument(
    "--virtio-rng", 
    action="store_true", 
    help="Enable VirtIO entropy source device"
)

# Spacecraft accelerator options
parser.add_argument(
    "--enable-compression",
    action="store_true",
    help="Enable spacecraft compression accelerator"
)
parser.add_argument(
    "--enable-ml",
    action="store_true",
    help="Enable TinyML accelerator"
)
parser.add_argument(
    "--compression-k-param",
    type=int,
    default=4,
    help="Rice k-parameter for compression (default: 4)"
)
parser.add_argument(
    "--compression-block-size",
    type=int,
    default=64,
    help="Block size for compression in bytes (default: 64)"
)
parser.add_argument(
    "--ml-mac-rows",
    type=int,
    default=8,
    help="Number of rows in ML MAC array (default: 8)"
)
parser.add_argument(
    "--ml-mac-cols",
    type=int,
    default=8,
    help="Number of columns in ML MAC array (default: 8)"
)
parser.add_argument(
    "--ml-sram-size",
    type=str,
    default="64kB",
    help="ML accelerator SRAM size (default: 64kB)"
)

args = parser.parse_args()


# =====================================================================
# System Setup
# =====================================================================

print("\n" + "="*60)
print("SPACECRAFT HETEROGENEOUS PROCESSOR - Full System Mode")
print("="*60)

# CPU and Memory configuration
(CPUClass, mem_mode, FutureClass) = Simulation.setCPUClass(args)
assert issubclass(CPUClass, RiscvCPU)
MemClass = Simulation.setMemClass(args)

np = args.num_cpus
print(f"Number of CPUs: {np}")

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


# =====================================================================
# Add Custom Accelerators
# =====================================================================

# Track accelerator address ranges for PMA checker
accel_ranges = []

if HAVE_CUSTOM_ACCEL:
    # Create accelerator bus for custom accelerators
    system.accel_bus = IOXBar()
    
    # Use address range 0x20000000 to avoid conflicts with HiFive platform
    # HiFive uses 0x10000000-0x10010000 for UART, disk, etc.
    COMPRESS_BASE = 0x20000000
    ML_BASE = 0x20001000
    
    if args.enable_compression:
        print("\nSetting up Compression Core...")
        system.compression_core = CompressionCore(
            addr_range=AddrRange(COMPRESS_BASE, size='256B'),
            k_param=args.compression_k_param,
            block_size=args.compression_block_size,
            compression_latency=10
        )
        # Connect to accelerator bus (CPU side) and memory bus (mem side)
        system.compression_core.cpu_side = system.accel_bus.mem_side_ports
        system.compression_core.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(COMPRESS_BASE, size='256B'))
        print(f"  Address: 0x{COMPRESS_BASE:08X}")
        print(f"  K-parameter: {args.compression_k_param}")
        print(f"  Block size: {args.compression_block_size}")

    if args.enable_ml:
        print("\nSetting up TinyML Accelerator...")
        system.ml_accel = TinyMLAccel(
            addr_range=AddrRange(ML_BASE, size='256B'),
            mac_array_rows=args.ml_mac_rows,
            mac_array_cols=args.ml_mac_cols,
            sram_size=args.ml_sram_size,
            mac_latency=1,
            dma_latency=5
        )
        # Connect to accelerator bus (CPU side) and memory bus (mem side)
        system.ml_accel.cpu_side = system.accel_bus.mem_side_ports
        system.ml_accel.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(ML_BASE, size='256B'))
        print(f"  Address: 0x{ML_BASE:08X}")
        print(f"  MAC Array: {args.ml_mac_rows}x{args.ml_mac_cols}")
        print(f"  SRAM: {args.ml_sram_size}")
    
    # Create bridge from IO bus to accelerator bus
    if accel_ranges:
        system.accel_bridge = Bridge(delay="10ns")
        system.accel_bridge.cpu_side_port = system.iobus.mem_side_ports
        system.accel_bridge.mem_side_port = system.accel_bus.cpu_side_ports
        system.accel_bridge.ranges = accel_ranges
        print(f"\n  Accelerator bridge configured for {len(accel_ranges)} device(s)")
else:
    if args.enable_compression or args.enable_ml:
        print("\nWarning: Custom accelerators requested but not available in build.")
        print("Please rebuild gem5 with custom_accel sources.")


# =====================================================================
# VirtIO Devices
# =====================================================================

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


# =====================================================================
# Bridge Configuration
# =====================================================================

system.bridge = Bridge(delay="50ns")
system.bridge.mem_side_port = system.iobus.cpu_side_ports
system.bridge.cpu_side_port = system.membus.mem_side_ports
# Don't include accel_ranges here - they have their own bridge
system.bridge.ranges = system.platform._off_chip_ranges()

system.platform.attachOnChipIO(system.membus)
system.platform.attachOffChipIO(system.iobus)
system.platform.attachPlic()
system.platform.setNumCores(np)


# =====================================================================
# Clock and Voltage Domains
# =====================================================================

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


# =====================================================================
# CPU Configuration
# =====================================================================

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

# Configure CPUs
for i in range(np):
    if args.bp_type:
        bpClass = ObjectList.bp_list.get(args.bp_type)
        system.cpu[i].branchPred = bpClass()
    system.cpu[i].createThreads()


# =====================================================================
# PMA Checker (Physical Memory Attributes)
# =====================================================================

uncacheable_range = [
    *system.platform._on_chip_ranges(),
    *system.platform._off_chip_ranges(),
    *accel_ranges,  # Add accelerator ranges as uncacheable
]

for cpu in system.cpu:
    cpu.mmu.pma_checker = PMAChecker(uncacheable=uncacheable_range)


# =====================================================================
# Device Tree Generation
# =====================================================================

print("\nGenerating device tree...")
if args.dtb_filename:
    system.workload.dtb_filename = args.dtb_filename
else:
    generateDtb(system, args)
    system.workload.dtb_filename = path.join(m5.options.outdir, "device.dtb")

system.workload.dtb_addr = 0x87E00000

# Linux boot command
if args.command_line:
    system.workload.command_line = args.command_line
else:
    kernel_cmd = ["console=ttyS0", "root=/dev/vda", "ro"]
    system.workload.command_line = " ".join(kernel_cmd)


# =====================================================================
# Cache and Memory Configuration
# =====================================================================

CacheConfig.config_cache(args, system)
MemConfig.config_mem(args, system)


# =====================================================================
# Run Simulation
# =====================================================================

print("\n" + "="*60)
print("SYSTEM CONFIGURATION SUMMARY")
print("="*60)
print(f"CPUs:               {np} x {CPUClass.__name__}")
print(f"Memory:             {args.mem_size}")
print(f"Kernel:             {args.kernel}")
print(f"Disk Image:         {args.disk_image or 'None'}")
print(f"Compression Core:   {'Enabled' if args.enable_compression else 'Disabled'}")
print(f"ML Accelerator:     {'Enabled' if args.enable_ml else 'Disabled'}")
print("="*60 + "\n")

root = Root(full_system=True, system=system)

Simulation.setWorkCountOptions(system, args)
Simulation.run(args, root, system, FutureClass)


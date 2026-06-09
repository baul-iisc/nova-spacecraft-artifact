"""
Spacecraft SoC - Shared vs Dedicated Accelerator Configuration
PhD Research: Chandraboul

This configuration enables experimentation with two accelerator topologies:

1. SHARED MODE: All cores share a single instance of each accelerator
   - Accelerators connected via shared bus with arbitration
   - Contention delays when multiple cores access same accelerator

2. DEDICATED MODE: Each core has its own dedicated accelerator instances
   - No contention, maximum parallelism
   - Higher area cost but better throughput

Usage:
    # Shared mode
    ./build/RISCV/gem5.opt configs/example/riscv/spacecraft_shared_vs_dedicated.py \
        --kernel=<path-to-kernel> --disk-image=<path-to-disk-image> \
        --num-cpus=4 --accel-mode=shared

    # Dedicated mode  
    ./build/RISCV/gem5.opt configs/example/riscv/spacecraft_shared_vs_dedicated.py \
        --kernel=<path-to-kernel> --disk-image=<path-to-disk-image> \
        --num-cpus=4 --accel-mode=dedicated
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
HAVE_MATRIX_ACCEL = True
HAVE_TINY_TPU = True

try:
    from m5.objects import MatrixTileAccel
    print("MatrixTileAccel loaded successfully")
except ImportError as e:
    HAVE_MATRIX_ACCEL = False
    print(f"Warning: MatrixTileAccel not available: {e}")

try:
    from m5.objects import TinyTPU
    print("TinyTPU loaded successfully")
except ImportError as e:
    HAVE_TINY_TPU = False
    print(f"Warning: TinyTPU not available: {e}")

requires(isa_required=ISA.RISCV)

# =============================================================================
# Memory Map for Accelerators
# =============================================================================

ACCEL_BASE = 0x20000000

# Shared mode - single instance per accelerator type
SHARED_MATRIX_BASE = ACCEL_BASE + 0x100000
SHARED_TPU_BASE    = ACCEL_BASE + 0x110000

# Dedicated mode - per-core instances
def get_dedicated_matrix_base(core_id):
    return ACCEL_BASE + 0x200000 + (core_id * 0x20000)

def get_dedicated_tpu_base(core_id):
    return ACCEL_BASE + 0x200000 + (core_id * 0x20000) + 0x10000


# =============================================================================
# DTB Generation
# =============================================================================

def generateMemNode(state, mem_range):
    node = FdtNode(f"memory@{int(mem_range.start):x}")
    node.append(FdtPropertyStrings("device_type", ["memory"]))
    node.append(
        FdtPropertyWords(
            "reg",
            state.addrCells(mem_range.start) + state.sizeCells(mem_range.size()),
        )
    )
    return node


def generateDtb(system, args, accel_ranges, mode):
    state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
    root = FdtNode("/")
    root.append(state.addrCellsProperty())
    root.append(state.sizeCellsProperty())
    root.appendCompatible(["riscv-virtio", "spacecraft-dls-soc"])

    for mem_range in system.mem_ranges:
        root.append(generateMemNode(state, mem_range))

    sections = [*system.cpu, system.platform]
    for section in sections:
        for node in section.generateDeviceTree(state):
            if node.get_name() == root.get_name():
                root.merge(node)
            else:
                root.append(node)

    fdt = Fdt()
    fdt.add_rootnode(root)
    fdt.writeDtsFile(path.join(m5.options.outdir, "device.dts"))
    fdt.writeDtbFile(path.join(m5.options.outdir, "device.dtb"))


# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(
    description="Spacecraft SoC - Shared vs Dedicated Accelerator Experiments"
)

Options.addCommonOptions(parser, ISA.RISCV)
Options.addFSOptions(parser)

parser.add_argument("--virtio-rng", action="store_true")
parser.add_argument("--accel-mode", type=str, default="shared",
                    choices=["shared", "dedicated"])
parser.add_argument("--enable-matrix", action="store_true", default=True)
parser.add_argument("--enable-tpu", action="store_true", default=True)
parser.add_argument("--arb-latency", type=int, default=5)

args = parser.parse_args()

# =============================================================================
# System Setup
# =============================================================================

print("\n" + "="*80)
print("SPACECRAFT SOC - SHARED VS DEDICATED ACCELERATOR EXPERIMENTS")
print("="*80)
print(f"Mode: {args.accel_mode.upper()}")
print(f"CPUs: {args.num_cpus}")
print("="*80)

(CPUClass, mem_mode, FutureClass) = Simulation.setCPUClass(args)
assert issubclass(CPUClass, RiscvCPU)
MemClass = Simulation.setMemClass(args)

np = args.num_cpus

system = System()
mdesc = SysConfig(
    disks=args.disk_image,
    rootdev=args.root_device,
    mem=args.mem_size,
    os_type=args.os_type,
)

system.mem_mode = mem_mode
system.mem_ranges = [AddrRange(start=0x80000000, size=mdesc.mem())]

system.workload = RiscvLinux()
system.workload.object_file = args.kernel

system.iobus = IOXBar()
system.membus = MemBus()
system.system_port = system.membus.cpu_side_ports

system.platform = HiFive()
system.platform.rtc = RiscvRTC(frequency=Frequency("100MHz"))
system.platform.clint.int_pin = system.platform.rtc.int_pin
system.platform.pci_host.pio = system.iobus.mem_side_ports

# =============================================================================
# Accelerator Configuration
# =============================================================================

accel_ranges = []
accel_count = 0

print("\n" + "-"*80)
print("ACCELERATOR CONFIGURATION")
print("-"*80)

if args.accel_mode == "shared":
    print("\n>>> SHARED MODE: All cores share accelerator instances")
    
    system.shared_accel_bus = IOXBar()
    
    if HAVE_MATRIX_ACCEL and args.enable_matrix:
        print(f"  [SHARED] Matrix Tile Accel @ 0x{SHARED_MATRIX_BASE:08X}")
        system.shared_matrix = MatrixTileAccel(
            addr_range=AddrRange(SHARED_MATRIX_BASE, size='4kB'),
            input_buffer_size='8kB',
            weight_buffer_size='8kB',
            accum_buffer_size='4kB',
            output_buffer_size='4kB',
            mac_latency=3,
            dma_latency=10
        )
        system.shared_matrix.cpu_side = system.shared_accel_bus.mem_side_ports
        system.shared_matrix.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(SHARED_MATRIX_BASE, size='4kB'))
        accel_count += 1
    
    if HAVE_TINY_TPU and args.enable_tpu:
        print(f"  [SHARED] TinyTPU         @ 0x{SHARED_TPU_BASE:08X}")
        system.shared_tpu = TinyTPU(
            addr_range=AddrRange(SHARED_TPU_BASE, size='4kB'),
            unified_buffer_size='128kB',
            weight_fifo_size='64kB',
            accum_buffer_size='8kB',
            mxu_latency=1,
            dma_latency=5
        )
        system.shared_tpu.cpu_side = system.shared_accel_bus.mem_side_ports
        system.shared_tpu.mem_side = system.membus.cpu_side_ports
        accel_ranges.append(AddrRange(SHARED_TPU_BASE, size='4kB'))
        accel_count += 1
    
    if accel_ranges:
        system.shared_accel_bridge = Bridge(
            delay=f"{args.arb_latency}ns",
            ranges=accel_ranges
        )
        system.shared_accel_bridge.cpu_side_port = system.iobus.mem_side_ports
        system.shared_accel_bridge.mem_side_port = system.shared_accel_bus.cpu_side_ports

else:
    print("\n>>> DEDICATED MODE: Each core has dedicated accelerator instances")
    
    system.dedicated_matrix = []
    system.dedicated_tpu = []
    system.per_core_buses = []
    system.per_core_bridges = []
    
    for core_id in range(np):
        print(f"  Core {core_id} Accelerators:")
        
        per_core_bus = IOXBar()
        system.per_core_buses.append(per_core_bus)
        
        core_ranges = []
        
        if HAVE_MATRIX_ACCEL and args.enable_matrix:
            matrix_base = get_dedicated_matrix_base(core_id)
            print(f"    [DEDICATED] Matrix Tile Accel @ 0x{matrix_base:08X}")
            
            matrix_accel = MatrixTileAccel(
                addr_range=AddrRange(matrix_base, size='4kB'),
                input_buffer_size='4kB',
                weight_buffer_size='4kB',
                accum_buffer_size='2kB',
                output_buffer_size='2kB',
                mac_latency=3,
                dma_latency=10
            )
            matrix_accel.cpu_side = per_core_bus.mem_side_ports
            matrix_accel.mem_side = system.membus.cpu_side_ports
            system.dedicated_matrix.append(matrix_accel)
            core_ranges.append(AddrRange(matrix_base, size='4kB'))
            accel_ranges.append(AddrRange(matrix_base, size='4kB'))
        
        if HAVE_TINY_TPU and args.enable_tpu:
            tpu_base = get_dedicated_tpu_base(core_id)
            print(f"    [DEDICATED] TinyTPU          @ 0x{tpu_base:08X}")
            
            tpu_accel = TinyTPU(
                addr_range=AddrRange(tpu_base, size='4kB'),
                unified_buffer_size='64kB',
                weight_fifo_size='32kB',
                accum_buffer_size='4kB',
                mxu_latency=1,
                dma_latency=5
            )
            tpu_accel.cpu_side = per_core_bus.mem_side_ports
            tpu_accel.mem_side = system.membus.cpu_side_ports
            system.dedicated_tpu.append(tpu_accel)
            core_ranges.append(AddrRange(tpu_base, size='4kB'))
            accel_ranges.append(AddrRange(tpu_base, size='4kB'))
        
        if core_ranges:
            bridge = Bridge(delay="2ns", ranges=core_ranges)
            bridge.cpu_side_port = system.iobus.mem_side_ports
            bridge.mem_side_port = per_core_bus.cpu_side_ports
            system.per_core_bridges.append(bridge)
            accel_count += len(core_ranges)

print(f"\nTotal Accelerator Instances: {accel_count}")
print("-"*80)

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
    generateDtb(system, args, accel_ranges, args.accel_mode)
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

print("\n" + "="*80)
print("EXPERIMENT CONFIGURATION SUMMARY")
print("="*80)
print(f"Mode:               {args.accel_mode.upper()}")
print(f"CPUs:               {np} x {CPUClass.__name__}")
print(f"Memory:             {args.mem_size}")
print(f"Kernel:             {args.kernel}")
print(f"Disk Image:         {args.disk_image or 'None'}")
print(f"Accelerators:       {accel_count} instances")
print("="*80 + "\n")

root = Root(full_system=True, system=system)

Simulation.setWorkCountOptions(system, args)
Simulation.run(args, root, system, FutureClass)

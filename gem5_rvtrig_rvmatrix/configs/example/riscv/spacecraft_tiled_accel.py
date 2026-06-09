"""
Spacecraft DLS SoC - DianNao-Style Tiled Matrix Accelerator Configuration
PhD Research: Chandraboul

Configuration with the new MatrixTileAccel that provides:
- Hardware-managed tiling
- DMA-based data movement
- Input/Weight/Accumulator/Output buffers (DianNao-style)
"""

import argparse
import sys
from os import path

import m5
from m5.objects import *
from m5.util import addToPath

# =============================================================================
# Argument Parser
# =============================================================================

parser = argparse.ArgumentParser(description="Spacecraft SoC with Tiled Matrix Accelerator")
parser.add_argument("--binary", type=str, required=True, help="RISC-V binary")
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                    choices=["AtomicSimpleCPU", "TimingSimpleCPU", "MinorCPU"])
parser.add_argument("--num-cpus", type=int, default=1, help="Number of CPUs")
parser.add_argument("--mem-size", type=str, default="512MB", help="Memory size")

# Accelerator options
parser.add_argument("--enable-matrix-accel", action="store_true", default=True,
                    help="Enable DianNao-style matrix tile accelerator")
parser.add_argument("--input-buffer-size", type=str, default="4kB",
                    help="Input buffer size for A matrix tiles")
parser.add_argument("--weight-buffer-size", type=str, default="4kB",
                    help="Weight buffer size for B matrix tiles")
parser.add_argument("--accum-buffer-size", type=str, default="2kB",
                    help="Accumulator buffer size for partial sums")
parser.add_argument("--mac-latency", type=int, default=3,
                    help="Cycles per 3x3 tile MAC operation")
parser.add_argument("--dma-latency", type=int, default=10,
                    help="Cycles per DMA tile transfer")

args = parser.parse_args()

# =============================================================================
# System Setup
# =============================================================================

print("\n" + "="*60)
print("SPACECRAFT SOC - DIANNAO-STYLE TILED MATRIX ACCELERATOR")
print("="*60)

system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

system.mem_mode = "timing"
system.mem_ranges = [AddrRange(args.mem_size)]

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

# For atomic, connect directly to membus
if args.cpu_type == "AtomicSimpleCPU":
    for cpu in system.cpu:
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports
else:
    # For timing, create simple caches
    for i, cpu in enumerate(system.cpu):
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports

# Memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

system.system_port = system.membus.cpu_side_ports

# =============================================================================
# DianNao-Style Matrix Tile Accelerator
# =============================================================================

if args.enable_matrix_accel:
    from m5.objects.MatrixTileAccel import MatrixTileAccel
    
    # Create the accelerator
    system.matrix_accel = MatrixTileAccel()
    
    # Configure buffer sizes (DianNao-style on-chip SRAM)
    system.matrix_accel.input_buffer_size = args.input_buffer_size
    system.matrix_accel.weight_buffer_size = args.weight_buffer_size
    system.matrix_accel.accum_buffer_size = args.accum_buffer_size
    system.matrix_accel.output_buffer_size = args.accum_buffer_size
    
    # Configure latencies
    system.matrix_accel.mac_latency = args.mac_latency
    system.matrix_accel.dma_latency = args.dma_latency
    
    # Set address range for MMIO (0x60000000)
    system.matrix_accel.addr_range = AddrRange(0x60000000, size='4kB')
    
    # Connect ports
    system.matrix_accel.cpu_side = system.membus.mem_side_ports
    system.matrix_accel.mem_side = system.membus.cpu_side_ports
    
    print(f"\nMatrix Tile Accelerator:")
    print(f"  Input Buffer:  {args.input_buffer_size}")
    print(f"  Weight Buffer: {args.weight_buffer_size}")
    print(f"  Accum Buffer:  {args.accum_buffer_size}")
    print(f"  MAC Latency:   {args.mac_latency} cycles")
    print(f"  DMA Latency:   {args.dma_latency} cycles")
    print(f"  Address:       0x60000000")

# =============================================================================
# Interrupt Controllers
# =============================================================================

for cpu in system.cpu:
    cpu.createInterruptController()

# =============================================================================
# Workload
# =============================================================================

print(f"\nBinary: {args.binary}")
print(f"Memory: {args.mem_size}")
print("="*60)

binary = args.binary
system.workload = SEWorkload.init_compatible(binary)

for i, cpu in enumerate(system.cpu):
    process = Process(pid=100 + i)
    process.cmd = [binary]
    cpu.workload = process
    cpu.createThreads()

# =============================================================================
# Run Simulation
# =============================================================================

root = Root(full_system=False, system=system)
m5.instantiate()

print("\nBeginning simulation!")
exit_event = m5.simulate()
print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")


"""
Spacecraft SoC - Shared vs Dedicated Accelerator (SE Mode)
PhD Research: Chandraboul
"""

import argparse
import sys

import m5
from m5.objects import *
from m5.util import addToPath

# Memory Map
SHARED_MATRIX_BASE = 0x60000000
SHARED_TPU_BASE    = 0x60010000
DEDICATED_BASE     = 0x70000000

def get_dedicated_matrix_base(core_id):
    return DEDICATED_BASE + (core_id * 0x20000)

def get_dedicated_tpu_base(core_id):
    return DEDICATED_BASE + (core_id * 0x20000) + 0x10000

# Argument Parser
parser = argparse.ArgumentParser(description="Spacecraft SoC - Shared vs Dedicated (SE)")
parser.add_argument("--cmd", type=str, required=True, help="Binary to run")
parser.add_argument("--num-cpus", type=int, default=1, help="Number of CPUs")
parser.add_argument("--mem-size", type=str, default="512MB", help="Memory size")
parser.add_argument("--accel-mode", type=str, default="shared", choices=["shared", "dedicated"])
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU")

args = parser.parse_args()

# Import accelerators
HAVE_MATRIX_ACCEL = False
HAVE_TPU = False

try:
    from m5.objects.MatrixTileAccel import MatrixTileAccel
    HAVE_MATRIX_ACCEL = True
except ImportError:
    pass

try:
    from m5.objects.TinyTPU import TinyTPU
    HAVE_TPU = True
except ImportError:
    pass

# System Setup
print("\n" + "="*70)
print(f"SPACECRAFT SOC - {args.accel_mode.upper()} MODE (SE)")
print("="*70)

np = args.num_cpus
system = System()

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

if args.cpu_type == "AtomicSimpleCPU":
    system.mem_mode = "atomic"
    system.cpu = [RiscvAtomicSimpleCPU() for i in range(np)]
else:
    system.mem_mode = "timing"
    system.cpu = [RiscvTimingSimpleCPU() for i in range(np)]

system.mem_ranges = [AddrRange(args.mem_size)]

# Memory bus
system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

# Connect CPUs
for cpu in system.cpu:
    cpu.icache_port = system.membus.cpu_side_ports
    cpu.dcache_port = system.membus.cpu_side_ports

# Memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# Interrupt controllers
for cpu in system.cpu:
    cpu.createInterruptController()

# Accelerator Configuration
accel_count = 0

print(f"Mode: {args.accel_mode.upper()}")
print(f"CPUs: {np}")
print("-"*70)

if args.accel_mode == "shared":
    print("SHARED: All cores share single accelerator instances\n")
    
    if HAVE_MATRIX_ACCEL:
        print(f"  [SHARED] MatrixTileAccel @ 0x{SHARED_MATRIX_BASE:08X}")
        system.shared_matrix = MatrixTileAccel()
        system.shared_matrix.input_buffer_size = '8kB'
        system.shared_matrix.weight_buffer_size = '8kB'
        system.shared_matrix.accum_buffer_size = '4kB'
        system.shared_matrix.output_buffer_size = '4kB'
        system.shared_matrix.mac_latency = 3
        system.shared_matrix.dma_latency = 10
        system.shared_matrix.addr_range = AddrRange(SHARED_MATRIX_BASE, size='4kB')
        system.shared_matrix.cpu_side = system.membus.mem_side_ports
        system.shared_matrix.mem_side = system.membus.cpu_side_ports
        accel_count += 1
    
    if HAVE_TPU:
        print(f"  [SHARED] TinyTPU         @ 0x{SHARED_TPU_BASE:08X}")
        system.shared_tpu = TinyTPU()
        system.shared_tpu.unified_buffer_size = '128kB'
        system.shared_tpu.weight_fifo_size = '64kB'
        system.shared_tpu.accum_buffer_size = '8kB'
        system.shared_tpu.mxu_latency = 1
        system.shared_tpu.dma_latency = 5
        system.shared_tpu.addr_range = AddrRange(SHARED_TPU_BASE, size='4kB')
        system.shared_tpu.cpu_side = system.membus.mem_side_ports
        system.shared_tpu.mem_side = system.membus.cpu_side_ports
        accel_count += 1

else:
    print("DEDICATED: Each core has dedicated accelerator instances\n")
    
    matrix_accels = []
    tpu_accels = []
    
    for core_id in range(np):
        print(f"  Core {core_id}:")
        
        if HAVE_MATRIX_ACCEL:
            matrix_base = get_dedicated_matrix_base(core_id)
            print(f"    [DEDICATED] MatrixTileAccel @ 0x{matrix_base:08X}")
            
            matrix_accel = MatrixTileAccel()
            matrix_accel.input_buffer_size = '4kB'
            matrix_accel.weight_buffer_size = '4kB'
            matrix_accel.accum_buffer_size = '2kB'
            matrix_accel.output_buffer_size = '2kB'
            matrix_accel.mac_latency = 3
            matrix_accel.dma_latency = 10
            matrix_accel.addr_range = AddrRange(matrix_base, size='4kB')
            matrix_accel.cpu_side = system.membus.mem_side_ports
            matrix_accel.mem_side = system.membus.cpu_side_ports
            matrix_accels.append(matrix_accel)
            accel_count += 1
        
        if HAVE_TPU:
            tpu_base = get_dedicated_tpu_base(core_id)
            print(f"    [DEDICATED] TinyTPU         @ 0x{tpu_base:08X}")
            
            tpu_accel = TinyTPU()
            tpu_accel.unified_buffer_size = '64kB'
            tpu_accel.weight_fifo_size = '32kB'
            tpu_accel.accum_buffer_size = '4kB'
            tpu_accel.mxu_latency = 1
            tpu_accel.dma_latency = 5
            tpu_accel.addr_range = AddrRange(tpu_base, size='4kB')
            tpu_accel.cpu_side = system.membus.mem_side_ports
            tpu_accel.mem_side = system.membus.cpu_side_ports
            tpu_accels.append(tpu_accel)
            accel_count += 1
    
    if matrix_accels:
        system.matrix_accels = matrix_accels
    if tpu_accels:
        system.tpu_accels = tpu_accels

print(f"\nTotal accelerator instances: {accel_count}")
print("-"*70)

# Workload
binary = args.cmd
system.workload = SEWorkload.init_compatible(binary)

for i, cpu in enumerate(system.cpu):
    process = Process(pid=100 + i)
    process.cmd = [binary, f"--mode={args.accel_mode}"]
    cpu.workload = process
    cpu.createThreads()

# Run Simulation
print(f"\nBinary: {binary}")
print("="*70)

root = Root(full_system=False, system=system)
m5.instantiate()

print("\nBeginning simulation!")
exit_event = m5.simulate()
print(f"\nExiting @ tick {m5.curTick()} because {exit_event.getCause()}")

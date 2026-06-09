"""
Spacecraft SoC - 3x3 Systolic Matrix & CORDIC Accelerator Experiment
PhD Research: Chandraboul

Configurations:
  - Shared: All cores share single 3x3 matrix and CORDIC accelerators
  - Dedicated: Each core has its own accelerator instances

Metrics tracked:
  - Execution cycles
  - Accelerator utilization
  - Contention/arbitration overhead
  - Matrix and CORDIC operation counts
"""

import argparse
import sys
import os

import m5
from m5.objects import *
from m5.util import addToPath

# Memory Map for Accelerators
SHARED_MATRIX_BASE  = 0x50000000
SHARED_CORDIC_BASE  = 0x50010000
DEDICATED_BASE      = 0x60000000  # Per-core accelerators start here

def get_dedicated_matrix_base(core_id):
    """Each core's dedicated matrix accel is 0x10000 apart"""
    return DEDICATED_BASE + (core_id * 0x20000)

def get_dedicated_cordic_base(core_id):
    """Each core's dedicated CORDIC is 0x10000 after matrix"""
    return DEDICATED_BASE + (core_id * 0x20000) + 0x10000

# Argument Parser
parser = argparse.ArgumentParser(
    description="Spacecraft 3x3 Systolic Matrix & CORDIC Experiment")
parser.add_argument("--cmd", type=str, required=True, 
                    help="Binary to run")
parser.add_argument("--options", type=str, default="",
                    help="Arguments to pass to binary")
parser.add_argument("--num-cpus", type=int, default=1, 
                    help="Number of CPUs (1, 2, 4, 8)")
parser.add_argument("--mem-size", type=str, default="512MB", 
                    help="Memory size")
parser.add_argument("--accel-mode", type=str, default="shared", 
                    choices=["shared", "dedicated"],
                    help="Accelerator mode: shared or dedicated")
parser.add_argument("--cpu-type", type=str, default="TimingSimpleCPU",
                    choices=["AtomicSimpleCPU", "TimingSimpleCPU", "MinorCPU", "O3CPU"],
                    help="CPU type to use")

args = parser.parse_args()

# Try to import accelerators
HAVE_SHARED_MATRIX = False
HAVE_SHARED_CORDIC = False
HAVE_MATRIX_TILE = False

try:
    from m5.objects.SharedMatrixAccel import SharedMatrixAccel
    HAVE_SHARED_MATRIX = True
except ImportError:
    print("WARNING: SharedMatrixAccel not available")

try:
    from m5.objects.SharedCORDICAccel import SharedCORDICAccel
    HAVE_SHARED_CORDIC = True
except ImportError:
    print("WARNING: SharedCORDICAccel not available")

try:
    from m5.objects.MatrixTileAccel import MatrixTileAccel
    HAVE_MATRIX_TILE = True
except ImportError:
    print("WARNING: MatrixTileAccel not available")

# =============================================================================
# System Configuration
# =============================================================================

print("\n" + "="*70)
print(f"SPACECRAFT 3x3 SYSTOLIC MATRIX & CORDIC EXPERIMENT")
print(f"PhD Research: Chandraboul")
print("="*70)
print(f"Mode: {args.accel_mode.upper()}")
print(f"CPUs: {args.num_cpus}")
print(f"CPU Type: {args.cpu_type}")
print("-"*70)

np = args.num_cpus
system = System()

# Clock domain
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

# CPU creation based on type
if args.cpu_type == "AtomicSimpleCPU":
    system.mem_mode = "atomic"
    system.cpu = [RiscvAtomicSimpleCPU() for i in range(np)]
elif args.cpu_type == "MinorCPU":
    system.mem_mode = "timing"
    system.cpu = [RiscvMinorCPU() for i in range(np)]
elif args.cpu_type == "O3CPU":
    system.mem_mode = "timing"
    system.cpu = [RiscvO3CPU() for i in range(np)]
else:  # TimingSimpleCPU (default)
    system.mem_mode = "timing"
    system.cpu = [RiscvTimingSimpleCPU() for i in range(np)]

# Memory
system.mem_ranges = [AddrRange(args.mem_size)]

# System bus
system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

# Connect CPUs to bus
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

# =============================================================================
# Accelerator Configuration
# =============================================================================

accel_count = 0
matrix_accels = []
cordic_accels = []

if args.accel_mode == "shared":
    print("\nSHARED MODE: All cores share single accelerator instances")
    print("-"*70)
    
    if HAVE_SHARED_MATRIX:
        print(f"  [SHARED] 3x3 Systolic Matrix @ 0x{SHARED_MATRIX_BASE:08X}")
        system.shared_matrix = SharedMatrixAccel()
        system.shared_matrix.num_cores = np
        system.shared_matrix.compute_latency = 10  # 10 cycles per 3x3 matmul
        system.shared_matrix.arbitration_latency = 2
        system.shared_matrix.mmio_base = SHARED_MATRIX_BASE
        system.shared_matrix.mmio_size = 0x1000
        # Connect to bus - using AddrRange
        system.shared_matrix_bridge = Bridge(delay='5ns')
        system.shared_matrix_bridge.ranges = [AddrRange(SHARED_MATRIX_BASE, size='4kB')]
        system.shared_matrix_bridge.cpu_side_port = system.membus.mem_side_ports
        system.shared_matrix_bridge.mem_side_port = system.shared_matrix.mmio_port
        accel_count += 1
        print(f"      Parameters: compute_latency=10, arbitration_latency=2")
    
    if HAVE_SHARED_CORDIC:
        print(f"  [SHARED] CORDIC Trig         @ 0x{SHARED_CORDIC_BASE:08X}")
        system.shared_cordic = SharedCORDICAccel()
        system.shared_cordic.num_cores = np
        system.shared_cordic.compute_latency = 5  # 5 cycles per CORDIC op
        system.shared_cordic.arbitration_latency = 1
        system.shared_cordic.mmio_base = SHARED_CORDIC_BASE
        system.shared_cordic.mmio_size = 0x1000
        # Connect to bus
        system.shared_cordic_bridge = Bridge(delay='5ns')
        system.shared_cordic_bridge.ranges = [AddrRange(SHARED_CORDIC_BASE, size='4kB')]
        system.shared_cordic_bridge.cpu_side_port = system.membus.mem_side_ports
        system.shared_cordic_bridge.mem_side_port = system.shared_cordic.mmio_port
        accel_count += 1
        print(f"      Parameters: compute_latency=5, arbitration_latency=1")

else:  # dedicated mode
    print("\nDEDICATED MODE: Each core has its own accelerator instances")
    print("-"*70)
    
    for core_id in range(np):
        matrix_base = get_dedicated_matrix_base(core_id)
        cordic_base = get_dedicated_cordic_base(core_id)
        
        print(f"  Core {core_id}:")
        
        if HAVE_SHARED_MATRIX:
            print(f"    [DEDICATED] 3x3 Matrix @ 0x{matrix_base:08X}")
            matrix_accel = SharedMatrixAccel()
            matrix_accel.num_cores = 1  # Dedicated = 1 core
            matrix_accel.compute_latency = 10
            matrix_accel.arbitration_latency = 0  # No arbitration for dedicated
            matrix_accel.mmio_base = matrix_base
            matrix_accel.mmio_size = 0x1000
            matrix_accels.append(matrix_accel)
            
            # Bridge for this accelerator
            bridge = Bridge(delay='5ns')
            bridge.ranges = [AddrRange(matrix_base, size='4kB')]
            bridge.cpu_side_port = system.membus.mem_side_ports
            bridge.mem_side_port = matrix_accel.mmio_port
            setattr(system, f'matrix_bridge_{core_id}', bridge)
            accel_count += 1
        
        if HAVE_SHARED_CORDIC:
            print(f"    [DEDICATED] CORDIC     @ 0x{cordic_base:08X}")
            cordic_accel = SharedCORDICAccel()
            cordic_accel.num_cores = 1  # Dedicated = 1 core
            cordic_accel.compute_latency = 5
            cordic_accel.arbitration_latency = 0  # No arbitration for dedicated
            cordic_accel.mmio_base = cordic_base
            cordic_accel.mmio_size = 0x1000
            cordic_accels.append(cordic_accel)
            
            # Bridge for this accelerator
            bridge = Bridge(delay='5ns')
            bridge.ranges = [AddrRange(cordic_base, size='4kB')]
            bridge.cpu_side_port = system.membus.mem_side_ports
            bridge.mem_side_port = cordic_accel.mmio_port
            setattr(system, f'cordic_bridge_{core_id}', bridge)
            accel_count += 1
    
    if matrix_accels:
        system.matrix_accels = matrix_accels
    if cordic_accels:
        system.cordic_accels = cordic_accels

print(f"\nTotal accelerator instances: {accel_count}")
print("-"*70)

# =============================================================================
# Workload Setup
# =============================================================================

binary = args.cmd
system.workload = SEWorkload.init_compatible(binary)

# Create process for each CPU
for i, cpu in enumerate(system.cpu):
    process = Process(pid=100 + i)
    # Pass core ID and mode to binary
    cmd_args = [binary]
    if args.options:
        cmd_args.extend(args.options.split())
    cmd_args.extend([f"--core={i}", f"--mode={args.accel_mode}"])
    process.cmd = cmd_args
    cpu.workload = process
    cpu.createThreads()

print(f"\nBinary: {binary}")
print(f"Arguments: {args.options}")
print("="*70)

# =============================================================================
# Run Simulation
# =============================================================================

root = Root(full_system=False, system=system)
m5.instantiate()

print("\nBeginning simulation!")
print("-"*70)

exit_event = m5.simulate()

print("\n" + "="*70)
print(f"Simulation complete @ tick {m5.curTick()}")
print(f"Exit cause: {exit_event.getCause()}")
print("="*70)

# Print summary stats
print("\n" + "-"*70)
print("KEY METRICS:")
print("-"*70)
print(f"  Total Ticks: {m5.curTick():,}")
print(f"  CPUs: {np}")
print(f"  Mode: {args.accel_mode}")
print(f"  Accelerators: {accel_count}")
print("-"*70)


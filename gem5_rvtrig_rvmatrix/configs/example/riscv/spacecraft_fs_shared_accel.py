"""
Spacecraft Full System Configuration with Shared Accelerators
PhD Research: Chandraboul

This configuration creates a multicore RISC-V system with shared accelerators:
- SharedMatrixAccel: 3x3 matrix multiply with arbitration
- SharedCORDICAccel: Trigonometric operations with arbitration

Each accelerator is shared among all cores, causing contention when
multiple cores try to access them simultaneously.
"""

import argparse
import sys
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath("../../")

# Import shared accelerators
try:
    from m5.objects.SharedMatrixAccel import SharedMatrixAccel
    from m5.objects.SharedCORDICAccel import SharedCORDICAccel
    ACCEL_AVAILABLE = True
except ImportError:
    print("WARNING: Shared accelerators not available")
    ACCEL_AVAILABLE = False

# MMIO base addresses
SHARED_MATRIX_BASE = 0x50000000
SHARED_CORDIC_BASE = 0x50010000

# ============================================================================
# Cache Configuration
# ============================================================================

class L1ICache(Cache):
    size = '32kB'
    assoc = 4
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 8

class L1DCache(Cache):
    size = '32kB'
    assoc = 4
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 8

# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='Spacecraft FS with Shared Accelerators')
    parser.add_argument('--num-cpus', type=int, default=4, help='Number of CPUs')
    parser.add_argument('--mem-size', type=str, default='512MB', help='Memory size')
    parser.add_argument('--kernel', type=str, required=True, help='Linux kernel')
    parser.add_argument('--disk-image', type=str, default=None, help='Disk image')
    parser.add_argument('--dtb', type=str, default=None, help='Device tree blob')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("SPACECRAFT FULL SYSTEM WITH SHARED ACCELERATORS")
    print("PhD Research: Chandraboul")
    print("=" * 70)
    print(f"CPUs: {args.num_cpus}")
    print(f"Memory: {args.mem_size}")
    if ACCEL_AVAILABLE:
        print("Shared Accelerators:")
        print(f"  - SharedMatrixAccel @ 0x{SHARED_MATRIX_BASE:08X}")
        print(f"  - SharedCORDICAccel @ 0x{SHARED_CORDIC_BASE:08X}")
    print("=" * 70)
    
    # Create system
    system = System()
    system.clk_domain = SrcClockDomain()
    system.clk_domain.clock = '1GHz'
    system.clk_domain.voltage_domain = VoltageDomain()
    
    system.mem_mode = 'timing'
    system.mem_ranges = [AddrRange(args.mem_size)]
    
    # Create CPUs
    system.cpu = [TimingSimpleCPU(cpu_id=i) for i in range(args.num_cpus)]
    
    # Memory bus
    system.membus = SystemXBar()
    
    # IO bus for accelerators
    system.iobus = IOXBar()
    
    # Bridge from membus to iobus
    system.iobridge = Bridge(delay='50ns')
    system.iobridge.ranges = [
        AddrRange(SHARED_MATRIX_BASE, SHARED_MATRIX_BASE + 0x10000)
    ]
    system.iobridge.cpu_side_port = system.membus.mem_side_ports
    system.iobridge.mem_side_port = system.iobus.cpu_side_ports
    
    # Create shared accelerators if available
    if ACCEL_AVAILABLE:
        system.shared_matrix = SharedMatrixAccel(
            num_cores=args.num_cpus,
            compute_latency=10,
            arbitration_latency=2,
            mmio_base=SHARED_MATRIX_BASE,
            mmio_size=0x1000
        )
        
        system.shared_cordic = SharedCORDICAccel(
            num_cores=args.num_cpus,
            compute_latency=5,
            arbitration_latency=1,
            mmio_base=SHARED_CORDIC_BASE,
            mmio_size=0x1000
        )
        
        # Connect accelerators to IO bus
        system.shared_matrix.mmio_port = system.iobus.mem_side_ports
        system.shared_cordic.mmio_port = system.iobus.mem_side_ports
    
    # Create caches and connect CPUs
    for i, cpu in enumerate(system.cpu):
        cpu.icache = L1ICache()
        cpu.dcache = L1DCache()
        
        cpu.icache_port = cpu.icache.cpu_side
        cpu.dcache_port = cpu.dcache.cpu_side
        
        cpu.icache.mem_side = system.membus.cpu_side_ports
        cpu.dcache.mem_side = system.membus.cpu_side_ports
        
        cpu.createInterruptController()
    
    # Memory controller
    system.mem_ctrl = MemCtrl()
    system.mem_ctrl.dram = DDR3_1600_8x8()
    system.mem_ctrl.dram.range = system.mem_ranges[0]
    system.mem_ctrl.port = system.membus.mem_side_ports
    
    # System port
    system.system_port = system.membus.cpu_side_ports
    
    # Set up workload (Linux kernel boot)
    system.workload = RiscvLinux()
    system.workload.object_file = args.kernel
    
    if args.dtb:
        system.workload.dtb_filename = args.dtb
    
    # Disk image (VirtIO block device)
    if args.disk_image:
        system.vio_blk = RiscvMmioVirtIO(
            vio=VirtIOBlock(image=RawDiskImage(image_file=args.disk_image,
                                                read_only=False)),
            interrupt_id=0x8,
            pio_size=4096,
            pio_addr=0x10008000
        )
        system.vio_blk.pio = system.membus.mem_side_ports
    
    # Terminal/UART
    system.uart = Uart8250(pio_addr=0x10000000)
    system.uart.pio = system.membus.mem_side_ports
    system.uart.device = Terminal()
    
    # RTC
    system.rtc = RiscvRTC(frequency=Frequency("100MHz"))
    system.rtc.port = system.membus.mem_side_ports
    
    # Platform-level interrupt controller
    n_contexts = 2 * args.num_cpus
    system.plic = Plic(pio_addr=0x0c000000, n_src=11, n_contexts=n_contexts)
    system.plic.pio = system.membus.mem_side_ports
    
    for i, cpu in enumerate(system.cpu):
        cpu.interrupts[0].external_interrupt = \
            system.plic.contexts[2*i].int_req_port
        cpu.interrupts[0].platform = system.plic
    
    # Root object
    root = Root(full_system=True, system=system)
    
    # Instantiate
    m5.instantiate()
    
    print("Beginning simulation...")
    print("=" * 70)
    
    # Run simulation
    exit_event = m5.simulate()
    
    print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
    
    # Print stats summary
    if ACCEL_AVAILABLE:
        print("\n" + "=" * 70)
        print("Check stats.txt for accelerator contention statistics")
        print("=" * 70)

if __name__ == "__m5_main__":
    main()

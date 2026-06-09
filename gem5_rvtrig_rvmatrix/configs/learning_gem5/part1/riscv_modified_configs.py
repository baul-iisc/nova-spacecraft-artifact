import m5
from m5.objects import Root, System, AddrRange, DDR3_1600_8x8, SrcClockDomain, VoltageDomain, L2XBar, SystemXBar, SEWorkload, Process, MemCtrl
from caches import L1ICache, L1DCache, L2Cache  # L3Cache removed
import argparse

# Import branch predictors and prefetchers
from m5.objects import TAGE, BiModeBP, LTAGE, TournamentBP, StridePrefetcher, TaggedPrefetcher

# Import CPU models
from m5.objects import RiscvTimingSimpleCPU, RiscvMinorCPU, RiscvO3CPU, RiscvAtomicSimpleCPU

# Parse command-line arguments
parser = argparse.ArgumentParser(description="gem5 ARM SE Simulation Script")
parser.add_argument("binary", type=str, help="Path to the binary to execute")
parser.add_argument("--l1i_size", type=str, default="8kB", help="L1 instruction cache size")
parser.add_argument("--l1d_size", type=str, default="16kB", help="L1 data cache size")
parser.add_argument("--l1i_assoc", type=int, default=2, help="L1 instruction cache associativity")
parser.add_argument("--l1d_assoc", type=int, default=2, help="L1 data cache associativity")
parser.add_argument("--l1i_tag_latency", type=int, default=2, help="L1 instruction cache tag latency")
parser.add_argument("--l1d_tag_latency", type=int, default=4, help="L1 data cache tag latency")
parser.add_argument("--l1i_data_latency", type=int, default=2, help="L1 instruction cache data latency")
parser.add_argument("--l1d_data_latency", type=int, default=4, help="L1 data cache data latency")
parser.add_argument("--l1i_response_latency", type=int, default=2, help="L1 instruction cache response latency")
parser.add_argument("--l1d_response_latency", type=int, default=4, help="L1 data cache response latency")
parser.add_argument("--l1i_mshrs", type=int, default=2, help="L1 instruction cache MSHRs")
parser.add_argument("--l1d_mshrs", type=int, default=2, help="L1 data cache MSHRs")
parser.add_argument("--l1i_tgts_per_mshr", type=int, default=8, help="L1 instruction cache targets per MSHR")
parser.add_argument("--l1d_tgts_per_mshr", type=int, default=8, help="L1 data cache targets per MSHR")
parser.add_argument("--l2_size", type=str, default="64kB", help="L2 cache size")
parser.add_argument("--system_mem_range", type=str, default="1GB", help="System memory range")
parser.add_argument("--clock_freq", type=str, default="500MHz", help="System clock frequency")
parser.add_argument("--branch_pred", type=str, default="BiModeBP", choices=["TAGE", "BiModeBP", "LTAGE", "TOURNAMENT"], help="Branch predictor type")
parser.add_argument("--prefetcher", type=str, default="None", choices=["None", "StridePrefetcher", "TaggedPrefetcher"], help="Prefetcher type (None for spacecraft processors)")
parser.add_argument("--cpu_type", type=str, default="TimingSimpleCPU", choices=["TimingSimpleCPU", "MinorCPU", "O3CPU", "AtomicSimpleCPU"], help="CPU model type")
parser.add_argument("--num-cores", type=int, default=1, help="Number of CPU cores")
parser.add_argument(
    "--simple_trace",
    action="store_true",
    help="Enable SimpleTrace on CPUs and caches (slow; omit for IEEE TC batch runs)",
)

args = parser.parse_args()

# Convert "None" string to None for the prefetcher
if args.prefetcher == "None":
    args.prefetcher = None

system = System()

# Set the system to use "timing" memory mode
system.mem_mode = "timing"

# Set up the system clock and voltage domain
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = args.clock_freq
system.clk_domain.voltage_domain = VoltageDomain(voltage="1.1V")  # Low voltage for spacecraft

# Set the memory range of the system
system.mem_ranges = [AddrRange(args.system_mem_range)]

# Get number of cores
num_cores = args.num_cores

# Create CPU clock domain (shared by all CPUs)
system.cpu_clk_domain = SrcClockDomain()
system.cpu_clk_domain.clock = args.clock_freq
system.cpu_clk_domain.voltage_domain = VoltageDomain(voltage="1.1V")

# Create multiple CPUs based on num_cores
# First, create all CPUs in a regular Python list
cpus = []
for i in range(num_cores):
    # Create the specified CPU model with the specified branch predictor
    if args.cpu_type == "TimingSimpleCPU":
        cpu = RiscvTimingSimpleCPU(cpu_id=i)
    elif args.cpu_type == "MinorCPU":
        cpu = RiscvMinorCPU(cpu_id=i)
        cpu.fetch_width = 1    # Single-issue for determinism
        cpu.decode_width = 1
        cpu.execute_width = 1
        cpu.num_alus = 1       # Minimal functional units
        cpu.num_fpus = 0       # No FPU unless required
    elif args.cpu_type == "O3CPU":
        cpu = RiscvO3CPU(
            cpu_id=i,
            numPhysIntRegs=256,       # Default: 256 (usually sufficient)
            numPhysFloatRegs=256,     # Default: 256 (usually sufficient)
            numPhysVecRegs=256,       # Default: 256 (usually sufficient)
            numPhysMatRegs=256,       # Increase from default (e.g., 256)
            numROBEntries=128,        # Default: 192 (adjust if needed)
            numIQEntries=64,          # Default: 64 (adjust if needed)
        )
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
    
    # Create L1 instruction and data cache for this CPU
    cpu.icache = L1ICache(args)
    cpu.dcache = L1DCache(args)
    
    # Add the specified prefetcher to the caches
    if args.prefetcher == "StridePrefetcher":
        cpu.dcache.prefetcher = StridePrefetcher()
        cpu.icache.prefetcher = StridePrefetcher()
    elif args.prefetcher == "TaggedPrefetcher":
        cpu.dcache.prefetcher = TaggedPrefetcher()
        cpu.icache.prefetcher = TaggedPrefetcher()
    
    # Connect caches to CPU
    cpu.icache.connectCPU(cpu)
    cpu.dcache.connectCPU(cpu)
    
    # Add CPU to the temporary list
    cpus.append(cpu)

# Assign all CPUs to system at once (gem5 SimObject requirement)
system.cpu = cpus

# Create memory bus for L2
system.l2bus = L2XBar()

# Connect all L1 caches to the L2 bus
for cpu in system.cpu:
    cpu.icache.connectBus(system.l2bus)
    cpu.dcache.connectBus(system.l2bus)

# Create and connect the L2 cache
system.l2cache = L2Cache(args)
system.l2cache.connectCPUSideBus(system.l2bus)

# Create the main memory bus
system.membus = SystemXBar()
system.l2cache.connectMemSideBus(system.membus)

# Create the memory controller and connect it to the memory bus
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.dram.tCK = "2ns"  # Set DRAM base clock to 500 MHz (system memory BW 8 GB/s)
system.mem_ctrl.port = system.membus.mem_side_ports

# Create interrupt controllers for all CPUs
for cpu in system.cpu:
    cpu.createInterruptController()

# Connect the system port to the memory bus
system.system_port = system.membus.cpu_side_ports

# Set the binary as the workload
system.workload = SEWorkload.init_compatible(args.binary)

# Create a process for the binary application
# For multicore: each CPU can run the same binary or different binaries
# This example assigns the same binary to all CPUs
process = Process()
process.cmd = [args.binary]
#process.cmd = [args.binary, "synthetic_image.raw", "256", "256", "8", "output.ccsds", "8"]
#process.cmd = [args.binary, "synthetic"]
#process.cmd = [args.binary, "dummy1.pgm", "dummy2.pgm"]
#process.cmd = [args.binary, "input_image.raw", "calibration.dat 50"]
# Split the binary and its arguments
#import shlex
#cmd_parts = shlex.split(args.binary)
#process.cmd = cmd_parts

# Assign workload to each CPU
for cpu in system.cpu:
    cpu.workload = process
    cpu.createThreads()

if getattr(args, "simple_trace", False):
    from m5.objects import SimpleTrace

    for cpu in system.cpu:
        cpu.trace = SimpleTrace()
        cpu.icache.trace = SimpleTrace()
        cpu.dcache.trace = SimpleTrace()
    system.l2cache.trace = SimpleTrace()

# Instantiate the system and run the simulation
root = Root(full_system=False, system=system)
m5.instantiate()

print("Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")

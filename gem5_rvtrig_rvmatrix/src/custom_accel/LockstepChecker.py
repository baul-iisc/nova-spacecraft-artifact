"""
LockstepChecker.py - SimObject for Dual-core Lockstep (DLS) Checker

PhD Research: Chandraboul

This module monitors instruction execution in DLS pairs:
- Compares results between primary and checker cores
- Detects mismatches indicating potential faults
- Triggers pipeline flush and re-execution on mismatch
"""

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class LockstepChecker(ClockedObject):
    """
    Dual-core Lockstep (DLS) Checker for fault tolerance
    
    Monitors instruction commits from paired cores and compares results.
    On mismatch, signals recovery through pipeline flush and re-execution.
    
    DLS Pair Configuration:
    - Pair 0: Core 0 (primary) + Core 1 (checker)
    - Pair 1: Core 2 (primary) + Core 3 (checker)  
    - Pair 2: Core 4 (primary) + Core 5 (checker)
    - Pair 3: Core 6 (primary) + Core 7 (checker)
    """
    type = 'LockstepChecker'
    cxx_header = 'custom_accel/lockstep_checker.hh'
    cxx_class = 'gem5::LockstepChecker'

    # Memory-mapped register interface
    addr_range = Param.AddrRange(AddrRange(0x50000000, size='4kB'),
                                 "Address range for control registers")

    # DLS configuration
    num_pairs = Param.Unsigned(4, "Number of DLS pairs (max 4)")
    
    # Timing parameters
    comparison_latency = Param.Cycles(1, 
        "Cycles to compare instruction results")
    recovery_latency = Param.Cycles(10,
        "Cycles required for recovery after mismatch")
    
    # Mode configuration
    enable_recovery = Param.Bool(True,
        "Enable automatic recovery on mismatch")
    strict_mode = Param.Bool(False,
        "Enable strict comparison (includes instruction encoding)")

    # CPU ports for receiving commit information
    cpu_port = VectorResponsePort("CPU-side ports for commit monitoring")


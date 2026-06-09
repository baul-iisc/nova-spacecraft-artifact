# Fault Injector SimObject for Spacecraft Accelerators
# PhD Research: Chandraboul

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class FaultInjector(ClockedObject):
    """
    Fault Injection Framework for simulating SEUs in spacecraft accelerators.
    
    Features:
    - Random and deterministic fault injection
    - Bit-flip, hang, and timeout fault types
    - Watchdog-based fault detection
    - Error recovery simulation
    """
    type = 'FaultInjector'
    cxx_header = 'custom_accel/fault_injector.hh'
    cxx_class = 'gem5::FaultInjector'
    
    # Fault rate in faults per million cycles
    fault_rate = Param.Float(0.1, "Faults per million cycles")
    
    # Enable random fault injection
    enable_random_faults = Param.Bool(False, "Enable random fault injection")
    
    # Maximum number of faults to inject (-1 for unlimited)
    max_faults = Param.Int(-1, "Maximum faults to inject")
    
    # Watchdog timeout in cycles
    watchdog_timeout = Param.Cycles(1000000, "Watchdog timeout cycles")
    
    # Enable automatic recovery
    enable_recovery = Param.Bool(True, "Enable automatic fault recovery")
    
    # Random seed for reproducibility
    seed = Param.UInt64(0, "Random seed (0 for time-based)")



# NOVA Processor - Shared Functional Unit Python Definitions
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.objects.ClockedObject import ClockedObject


class SharedFunctionalUnit(ClockedObject):
    """
    A shared functional unit that models an accelerator used by multiple cores.
    
    When multiple cores try to use the same accelerator:
    - If free: operation starts immediately
    - If busy: request is queued, core is stalled
    - When complete: next queued request is processed
    
    This provides cycle-accurate contention modeling at the microarchitecture
    level, capturing the actual impact of shared accelerators.
    
    Accelerator types:
    - 0: TrigAccel (trigonometric/CORDIC)
    - 1: MatAccel (3x3 matrix operations)
    - 2: VPU (vision processing)
    - 3: NPU (neural processing)
    """
    
    type = 'SharedFunctionalUnit'
    cxx_header = 'cpu/shared_fu/shared_functional_unit.hh'
    cxx_class = 'gem5::SharedFunctionalUnit'
    
    # Accelerator type
    accel_type = Param.Int(0, "Accelerator type (0=Trig, 1=Mat, 2=VPU, 3=NPU)")
    
    # Instance ID (for identifying among multiple FUs of same type)
    instance_id = Param.Int(0, "Instance ID")
    
    # Default operation latency in cycles
    default_latency = Param.Cycles(15, "Default operation latency")
    
    # Maximum queue depth
    max_queue_depth = Param.Int(16, "Maximum pending requests in queue")


class SharedAcceleratorPool(SimObject):
    """
    Pool of shared functional units across all cores.
    
    Manages accelerator allocation with different sharing strategies:
    
    - FULLY_SHARED (0): All cores share all accelerators
    - FULLY_DEDICATED (1): Each core has private accelerators
    - HYBRID (2): Critical cores get dedicated, others share
    - ADAPTIVE (3): Dynamically adjust based on load
    """
    
    type = 'SharedAcceleratorPool'
    cxx_header = 'cpu/shared_fu/shared_functional_unit.hh'
    cxx_class = 'gem5::SharedAcceleratorPool'
    
    # Sharing mode
    sharing_mode = Param.Int(0, "Sharing mode (0=shared, 1=dedicated)")
    
    # Number of CPUs in the system
    num_cpus = Param.Int(4, "Number of CPUs")

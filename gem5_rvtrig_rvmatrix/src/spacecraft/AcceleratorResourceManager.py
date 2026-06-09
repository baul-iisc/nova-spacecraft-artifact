# NOVA Processor - Accelerator Resource Manager SimObject
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class AcceleratorResourceManager(ClockedObject):
    """
    Accelerator Resource Manager for NOVA Processor.
    
    Manages accelerator allocation with multiple sharing strategies:
    - FULLY_SHARED: All cores share all accelerators (area efficient)
    - FULLY_DEDICATED: Each core has its own accelerators (no contention)
    - HYBRID_SHARED: Critical tasks get dedicated, others share
    - ADAPTIVE: Dynamically switch based on contention levels
    
    Arbitration Policies:
    - FCFS: First-Come-First-Served
    - PRIORITY_BASED: Based on task criticality
    - DEADLINE_AWARE: Earliest deadline first
    - PROPORTIONAL_SHARE: Fair sharing based on quotas
    """
    
    type = 'AcceleratorResourceManager'
    cxx_header = 'spacecraft/accel_resource_manager.hh'
    cxx_class = 'gem5::spacecraft::AcceleratorResourceManager'
    
    # Core configuration
    num_cores = Param.Int(4, "Number of CPU cores")
    
    # Accelerator counts
    num_trig_accels = Param.Int(2, "Number of trigonometric accelerators")
    num_mat_accels = Param.Int(2, "Number of matrix accelerators")
    num_vpus = Param.Int(1, "Number of Vision Processing Units")
    num_npus = Param.Int(1, "Number of Neural Processing Units")
    
    # Sharing mode
    sharing_mode = Param.String("ADAPTIVE",
        "Sharing mode: FULLY_SHARED, FULLY_DEDICATED, HYBRID_SHARED, ADAPTIVE")
    
    # Contention thresholds
    high_contention_threshold = Param.Float(0.8, 
        "Queue depth ratio to trigger dedicated allocation")
    low_contention_threshold = Param.Float(0.3,
        "Queue depth ratio below which to consolidate to shared")
    
    # Power management
    idle_threshold = Param.Tick(10000000,
        "Idle time (ticks) before power gating an accelerator")
    
    # Contention monitoring
    contention_check_period = Param.Tick(1000000,
        "Period for contention check (ticks)")





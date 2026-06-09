# Adaptive Allocator SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

class AdaptiveAllocator(ClockedObject):
    """
    Adaptive Accelerator Allocation System
    
    Implements dynamic switching between shared/dedicated accelerator modes
    based on workload characteristics, contention, and mission phase.
    
    Research Question 1: When does sharing overhead outweigh benefits?
    """
    type = 'AdaptiveAllocator'
    cxx_header = "custom_accel/adaptive_allocator.hh"
    cxx_class = 'gem5::AdaptiveAllocator'

    # Number of cores in the system
    num_cores = Param.Int(4, "Number of processor cores")
    
    # Number of accelerators per type
    num_accelerators_per_type = Param.Int(2, "Accelerators per type")
    
    # Evaluation interval (cycles)
    evaluation_interval = Param.Cycles(10000, 
        "Cycles between allocation evaluations")
    
    # Contention threshold for mode switching
    contention_threshold = Param.Float(0.3, 
        "Contention ratio threshold for switching modes")
    
    # Hysteresis margin to prevent oscillation
    hysteresis_margin = Param.Float(0.1, 
        "Hysteresis margin for mode switching")
    
    # Enable adaptive mode switching
    enable_adaptive = Param.Bool(True, 
        "Enable runtime adaptive mode switching")


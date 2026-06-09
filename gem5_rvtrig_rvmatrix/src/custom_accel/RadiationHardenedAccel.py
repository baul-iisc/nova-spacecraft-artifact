# Radiation-Hardened Accelerator SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

class RadiationHardenedAccel(ClockedObject):
    """
    Radiation-Hardened Accelerator with TMR
    
    Implements radiation-tolerant computation with:
    - Triple Modular Redundancy (TMR)
    - Single-Event Upset (SEU) detection and correction
    - Graceful degradation modes
    - Reduced-precision fallback
    
    Research Question 3: Performance-reliability tradeoff in radiation-hardened designs
    """
    type = 'RadiationHardenedAccel'
    cxx_header = "custom_accel/radiation_hardened.hh"
    cxx_class = 'gem5::RadiationHardenedAccel'

    # Number of compute lanes
    num_lanes = Param.Int(4, "Number of redundant compute lanes")
    
    # Default redundancy mode (0=none, 1=DMR, 2=TMR, 3=TMR+spare)
    redundancy_mode = Param.Int(2, "Default redundancy mode")
    
    # SEU probability per operation (for simulation)
    seu_probability = Param.Float(1e-6, 
        "Single-event upset probability per operation")
    
    # MBU probability per operation
    mbu_probability = Param.Float(1e-8, 
        "Multi-bit upset probability per operation")
    
    # TMR voting latency
    tmr_voting_latency = Param.Cycles(2, "Cycles for TMR voting")
    
    # Reduced precision bits (for degraded mode)
    reduced_precision_bits = Param.Int(32, 
        "Bits of precision in degraded mode")
    
    # Enable checkpointing for recovery
    enable_checkpointing = Param.Bool(True, 
        "Enable state checkpointing for recovery")


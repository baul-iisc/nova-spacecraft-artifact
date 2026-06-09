# Heterogeneous Accelerator Network SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

class HeterogeneousAccelNetwork(ClockedObject):
    """
    Heterogeneous Accelerator Network
    
    Manages a mix of accelerators with different precision/power profiles:
    - High-precision matrix units for navigation
    - Low-precision matrix units for bulk processing
    - Configurable trig accelerators
    - Workload-to-accelerator routing
    
    Research Question 4: How to route workloads to appropriate accelerator tiers
    """
    type = 'HeterogeneousAccelNetwork'
    cxx_header = "custom_accel/heterogeneous_accel.hh"
    cxx_class = 'gem5::HeterogeneousAccelNetwork'

    # High-precision matrix accelerators
    num_hp_matrix = Param.Int(1, "Number of high-precision matrix units")
    
    # Low-precision matrix accelerators
    num_lp_matrix = Param.Int(2, "Number of low-precision matrix units")
    
    # High-precision trig (CORDIC) accelerators
    num_hp_trig = Param.Int(1, "Number of high-precision trig units")
    
    # Low-precision trig (LUT-based) accelerators
    num_lp_trig = Param.Int(2, "Number of low-precision trig units")
    
    # Total power budget (Watts)
    total_power_budget = Param.Float(10.0, "Total power budget in Watts")
    
    # Enable dynamic routing
    enable_dynamic_routing = Param.Bool(True, 
        "Enable dynamic workload routing")


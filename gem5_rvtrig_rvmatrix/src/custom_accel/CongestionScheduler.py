# Congestion-Aware Scheduler SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

class CongestionScheduler(ClockedObject):
    """
    Congestion-Aware Scheduler for Space Workloads
    
    Implements priority-based accelerator arbitration with:
    - Criticality levels (Navigation > Telemetry > Science)
    - Deadline-based scheduling
    - Predictive congestion modeling
    - Starvation prevention
    
    Research Question 2: Congestion-aware scheduling for space workloads
    """
    type = 'CongestionScheduler'
    cxx_header = "custom_accel/congestion_scheduler.hh"
    cxx_class = 'gem5::CongestionScheduler'

    # Number of accelerators managed
    num_accelerators = Param.Int(4, "Number of accelerators to manage")
    
    # Congestion threshold
    congestion_threshold = Param.Float(0.7, 
        "Utilization threshold for congestion")
    
    # Maximum queue depth
    max_queue_depth = Param.Cycles(64, "Maximum pending request queue depth")
    
    # Starvation timeout (cycles)
    starvation_timeout = Param.Cycles(100000, 
        "Cycles before declaring starvation")
    
    # Enable request preemption
    enable_preemption = Param.Bool(False, 
        "Enable preemption of lower priority requests")
    
    # Enable deadline-based scheduling
    enable_deadline_scheduling = Param.Bool(True, 
        "Enable deadline-aware scheduling")


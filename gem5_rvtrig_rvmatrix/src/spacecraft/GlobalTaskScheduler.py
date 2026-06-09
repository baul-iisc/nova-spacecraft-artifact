# NOVA Processor - Global Task Scheduler SimObject
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class GlobalTaskScheduler(ClockedObject):
    """
    Global Task Scheduler for ISRO spacecraft workloads.
    
    Features:
    - Criticality-aware scheduling (MISSION_CRITICAL > SAFETY_CRITICAL > ...)
    - Multiple policies: Fixed Priority, EDF, Energy-Aware EDF, Minimal
    - Preemption with context switch overhead modeling
    - Deadline monitoring and miss tracking
    - Per-core ready queues with load balancing
    
    Task Criticality Levels:
    - MISSION_CRITICAL (0): GNC, life support - cannot miss deadline
    - SAFETY_CRITICAL (1): Hazard avoidance, fault detection
    - OPERATIONAL (2): Navigation, communication
    - SCIENCE (3): Payload processing
    - HOUSEKEEPING (4): Telemetry, diagnostics
    """
    
    type = 'GlobalTaskScheduler'
    cxx_header = 'spacecraft/global_task_scheduler.hh'
    cxx_class = 'gem5::spacecraft::GlobalTaskScheduler'
    
    # Core configuration
    num_cores = Param.Int(4, "Number of CPU cores to manage")
    
    # Scheduling configuration
    scheduling_period = Param.Tick(1000000, "Scheduling tick period (1ms default)")
    context_switch_overhead = Param.Cycles(100, "Context switch overhead in cycles")
    scheduling_overhead = Param.Cycles(10, "Scheduling decision overhead in cycles")
    
    # Power configuration
    power_budget = Param.Float(100.0, "Total power budget in Watts")
    
    # Default scheduling policy
    # 0: FIXED_PRIORITY_PREEMPTIVE
    # 1: MIXED_CRITICALITY_EDF
    # 2: ENERGY_AWARE_EDF
    # 3: MINIMAL_OPERATIONS
    # 4: ROUND_ROBIN
    default_policy = Param.Int(1, "Default scheduling policy (1=MC_EDF)")
    
    # Deadline monitoring
    deadline_check_period = Param.Tick(10000000, "Deadline check period (10ms default)")
    panic_on_critical_miss = Param.Bool(False, 
        "Panic simulation on mission-critical deadline miss")





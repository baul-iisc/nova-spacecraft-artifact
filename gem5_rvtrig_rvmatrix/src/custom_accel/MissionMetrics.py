# Mission-Aware Metrics SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

class MissionMetrics(ClockedObject):
    """
    Mission-Aware Performance Metrics
    
    Space mission-specific performance metrics beyond speedup:
    - Operations per Joule (energy efficiency)
    - Soft-error resilience (MTBF under radiation)
    - Time-to-recovery after upsets
    - Deadline compliance for real-time tasks
    - Thermal performance characterization
    
    Research Question 8: Metrics that matter for space missions
    """
    type = 'MissionMetrics'
    cxx_header = "custom_accel/mission_metrics.hh"
    cxx_class = 'gem5::MissionMetrics'

    # Mission identifier
    mission_name = Param.String("spacecraft", "Mission name identifier")
    
    # Target ops per Joule
    target_ops_per_joule = Param.Float(1e9, 
        "Target operations per Joule")
    
    # Target MTBF (seconds)
    target_mtbf = Param.Float(86400.0, 
        "Target mean time between failures (seconds)")
    
    # Maximum acceptable deadline miss rate
    max_deadline_miss_rate = Param.Float(0.001, 
        "Maximum acceptable deadline miss rate")
    
    # Nominal operating temperature (Celsius)
    nominal_temperature = Param.Float(25.0, 
        "Nominal operating temperature in Celsius")


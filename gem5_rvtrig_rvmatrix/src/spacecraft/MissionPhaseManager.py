# NOVA Processor - Mission Phase Manager SimObject
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class MissionPhaseManager(ClockedObject):
    """
    Mission Phase Manager for ISRO spacecraft operations.
    
    Manages mission phases and coordinates system behavior:
    - LAUNCH: Launch and ascent (critical, full power)
    - ORBIT_INSERTION: Orbit insertion maneuvers
    - NORMAL_OPS: Normal orbital operations
    - LANDING: Powered descent and landing
    - SURFACE_OPS: Surface/rover operations (power constrained)
    - SAFE_MODE: Emergency mode (minimal operations)
    - ECLIPSE: Battery-only operation
    - COMM_WINDOW: Communication downlink priority
    - SCIENCE_OPS: Science observation mode
    - STANDBY: Low-power standby
    """
    
    type = 'MissionPhaseManager'
    cxx_header = 'spacecraft/mission_phase_manager.hh'
    cxx_class = 'gem5::spacecraft::MissionPhaseManager'
    
    # Initial mission phase
    initial_phase = Param.String("NORMAL_OPS", 
        "Initial mission phase (LAUNCH, ORBIT_INSERTION, NORMAL_OPS, LANDING, SURFACE_OPS, SAFE_MODE)")
    
    # Phase power budgets (default values, can be overridden)
    launch_power_budget = Param.Float(100.0, "Power budget during launch (Watts)")
    orbit_insertion_power_budget = Param.Float(100.0, "Power budget during orbit insertion (Watts)")
    normal_ops_power_budget = Param.Float(50.0, "Power budget during normal ops (Watts)")
    landing_power_budget = Param.Float(80.0, "Power budget during landing (Watts)")
    surface_ops_power_budget = Param.Float(20.0, "Power budget during surface ops (Watts)")
    safe_mode_power_budget = Param.Float(10.0, "Power budget in safe mode (Watts)")
    eclipse_power_budget = Param.Float(30.0, "Power budget during eclipse (Watts)")
    
    # Emergency configuration
    auto_safe_mode_threshold = Param.Float(5.0, 
        "Power level below which to auto-enter safe mode (Watts)")
    emergency_response_latency = Param.Cycles(10, 
        "Latency to respond to emergency (cycles)")
    
    # Phase transition logging
    log_transitions = Param.Bool(True, "Log all phase transitions")





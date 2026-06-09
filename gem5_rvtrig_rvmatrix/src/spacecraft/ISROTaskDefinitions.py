# NOVA Processor - ISRO Task Definitions SimObject
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

class ISROTaskDefinitions(SimObject):
    """
    ISRO Spacecraft Task Definitions.
    
    Defines all spacecraft tasks for ISRO missions with appropriate
    timing, criticality, and resource requirements.
    
    Tasks defined:
    - GNC_Loop (ID=0): 100Hz GNC control loop (MISSION_CRITICAL)
    - Star_Tracker (ID=1): 10Hz star field recognition (OPERATIONAL)
    - Crater_Detection (ID=2): 10Hz crater matching (SAFETY_CRITICAL)
    - Hazard_Avoidance (ID=3): 5Hz hazard detection (SAFETY_CRITICAL)
    - Terrain_Relative_Nav (ID=4): 2Hz terrain matching (OPERATIONAL)
    - Orbit_Propagation (ID=5): 1Hz orbit prediction (OPERATIONAL)
    - Science_Image (ID=6): 0.1Hz science processing (SCIENCE)
    - Telemetry (ID=7): 1Hz housekeeping (HOUSEKEEPING)
    - Kalman_Filter (ID=8): 50Hz sensor fusion (MISSION_CRITICAL)
    - Attitude_Control (ID=9): 20Hz attitude control (MISSION_CRITICAL)
    """
    
    type = 'ISROTaskDefinitions'
    cxx_header = 'spacecraft/isro_task_definitions.hh'
    cxx_class = 'gem5::spacecraft::ISROTaskDefinitions'





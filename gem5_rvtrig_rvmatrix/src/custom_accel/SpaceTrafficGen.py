# Space Workload Traffic Generator SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class SpaceTrafficGen(ClockedObject):
    """
    Generates synthetic spacecraft workload traffic patterns.
    
    Workload Types:
    - 0: GNC_ATTITUDE - 100Hz attitude control (CORDIC-heavy)
    - 1: GNC_ORBIT - 1Hz orbital propagation (Matrix-heavy)
    - 2: GNC_NAVIGATION - 10Hz navigation filter (mixed)
    - 3: PAYLOAD_IMAGE - Imaging pipeline (burst)
    - 4: PAYLOAD_COMPRESS - Data compression (sustained)
    - 5: TELEMETRY - 1Hz telemetry
    - 6: COMMAND - Sporadic commands
    - 7: MIXED_REALTIME - Combined real-time tasks
    - 8: STRESS_TEST - Maximum load
    """
    type = 'SpaceTrafficGen'
    cxx_header = 'custom_accel/space_traffic_gen.hh'
    cxx_class = 'gem5::SpaceTrafficGen'
    
    # Traffic port for sending requests
    traffic_port = RequestPort("Traffic injection port")
    
    # Workload type (0-8)
    workload_type = Param.Int(7, "Workload type (7=MIXED_REALTIME)")
    
    # Injection rate in requests per microsecond
    injection_rate = Param.Float(1.0, "Injection rate (requests/us)")
    
    # Number of cores to distribute traffic across
    num_cores = Param.Int(4, "Number of cores")
    
    # Enable real-time deadline checking
    enable_deadlines = Param.Bool(True, "Enable deadline checking")
    
    # Base latency for requests
    base_latency = Param.Cycles(10, "Base request latency")
    
    # Accelerator MMIO base addresses
    matrix_accel_base = Param.Addr(0x42000000, "Matrix accelerator base")
    cordic_accel_base = Param.Addr(0x42010000, "CORDIC accelerator base")
    compression_base = Param.Addr(0x40050000, "Compression core base")
    image_comp_base = Param.Addr(0x40070000, "Image compression base")
    
    # Random seed
    seed = Param.UInt64(0, "Random seed")



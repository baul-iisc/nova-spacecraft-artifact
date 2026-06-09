# Space Benchmark Suite SimObject
# PhD Research: Chandraboul

from m5.params import *
from m5.objects.ClockedObject import ClockedObject

class SpaceBenchmark(ClockedObject):
    """
    Space Computing Benchmark Suite
    
    A representative benchmark suite for spacecraft computing including:
    - Attitude control (Kalman filter, quaternion rotation)
    - Navigation (orbit propagation, star tracking)
    - Image processing (compression, feature extraction)
    - Telecommunications (LDPC, CCSDS framing)
    
    Research Question 7: Representative benchmarks for space computing
    """
    type = 'SpaceBenchmark'
    cxx_header = "custom_accel/space_benchmark.hh"
    cxx_class = 'gem5::SpaceBenchmark'

    # Default number of iterations per benchmark
    default_iterations = Param.Int(100, "Default benchmark iterations")
    
    # Default data size
    default_data_size = Param.Int(256, "Default data size for benchmarks")
    
    # Enable result validation
    enable_validation = Param.Bool(True, "Enable correctness validation")
    
    # Use hardware accelerators when available
    use_hardware_accel = Param.Bool(True, 
        "Use hardware accelerators for benchmarks")


# NOVA Processor - Navigation-Optimized Vision-Augmented Processor SimObject
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class NOVAProcessor(ClockedObject):
    """
    NOVA (Navigation-Optimized Vision-Augmented) Processor
    
    A heterogeneous RISC-V system optimized for autonomous spacecraft
    with domain-specific accelerators for navigation, vision, and onboard intelligence.
    
    Architecture:
    - 4-8 RISC-V cores (RV64GC) with asymmetric multiprocessing
    - Navigation accelerators: TrigAccel (sin/cos/tan/atan2), MatAccel (3x3 matrices)
    - Vision Processing Unit (VPU): Edge detection, feature extraction, optical flow
    - Neural Processing Unit (NPU): TinyML inference with systolic array
    - Mesh NoC interconnect with priority-based routing
    - Radiation-hardened design with TMR on critical paths
    
    Key Features:
    - Adaptive accelerator allocation (shared/dedicated/hybrid)
    - Mission phase-aware scheduling
    - Energy-proportional design with DVFS
    - Vision-based autonomous navigation pipeline
    
    PhD Research Contributions:
    - First comprehensive space processor design integrating navigation + vision + ML
    - Gem5-based framework for space processor design exploration
    - Quantitative analysis of shared vs dedicated accelerator tradeoffs
    """
    
    type = 'NOVAProcessor'
    cxx_header = 'spacecraft/nova_processor.hh'
    cxx_class = 'gem5::spacecraft::NOVAProcessor'
    
    # Core configuration
    num_high_perf_cores = Param.Int(4, 
        "Number of high-performance cores for critical GNC")
    num_energy_eff_cores = Param.Int(0, 
        "Number of energy-efficient cores for background tasks")
    
    # Accelerator configuration
    num_trig_accels = Param.Int(2, "Number of trigonometric accelerators")
    num_mat_accels = Param.Int(2, "Number of 3x3 matrix accelerators")
    num_vpus = Param.Int(1, "Number of Vision Processing Units")
    num_npus = Param.Int(1, "Number of Neural Processing Units")
    
    # Cache configuration (KB/MB)
    l1i_cache_kb = Param.Int(32, "L1 Instruction cache size in KB")
    l1d_cache_kb = Param.Int(32, "L1 Data cache size in KB")
    l2_cache_kb = Param.Int(512, "L2 cache size in KB")
    l3_cache_mb = Param.Int(2, "L3 cache size in MB")
    
    # Accelerator memory
    vpu_scratchpad_kb = Param.Int(64, "VPU scratchpad size in KB")
    npu_weight_cache_kb = Param.Int(64, "NPU weight cache size in KB")
    npu_activation_spad_kb = Param.Int(32, "NPU activation scratchpad in KB")
    
    # Power configuration
    max_power_watts = Param.Float(100.0, "Maximum power budget (Watts)")
    nominal_power_watts = Param.Float(50.0, "Nominal power budget (Watts)")
    min_power_watts = Param.Float(10.0, "Minimum power (safe mode) (Watts)")
    
    # Initial mission phase
    initial_phase = Param.String("NORMAL_OPS", "Initial mission phase")
    
    # Sharing mode for accelerators
    accel_sharing_mode = Param.String("ADAPTIVE", 
        "Accelerator sharing: FULLY_SHARED, FULLY_DEDICATED, HYBRID_SHARED, ADAPTIVE")





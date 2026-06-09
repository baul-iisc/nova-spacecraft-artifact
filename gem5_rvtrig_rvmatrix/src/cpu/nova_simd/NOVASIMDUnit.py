"""
NOVA Processor - GPU-like SIMD Vision Accelerator
PhD Research: Futuristic Spacecraft Processor

This module defines the Python SimObject for the NOVA SIMD unit,
a GPU-like accelerator optimized for parallel vision processing.

Architecture:
    ┌─────────────────────────────────────────────────────────────┐
    │                  NOVA SIMD Vision Unit                       │
    ├─────────────────────────────────────────────────────────────┤
    │                                                              │
    │  ┌─────────────────────────────────────────────────────┐    │
    │  │              Instruction Scheduler                   │    │
    │  │         (Warp/Wavefront Management)                  │    │
    │  └───────────────────────┬─────────────────────────────┘    │
    │                          │                                   │
    │  ┌───────────────────────┴─────────────────────────────┐    │
    │  │              Processing Element Array                │    │
    │  │                                                      │    │
    │  │  ┌────┐ ┌────┐ ┌────┐ ┌────┐ ... ┌────┐ ┌────┐    │    │
    │  │  │PE 0│ │PE 1│ │PE 2│ │PE 3│     │PE30│ │PE31│    │    │
    │  │  │256b│ │256b│ │256b│ │256b│     │256b│ │256b│    │    │
    │  │  └────┘ └────┘ └────┘ └────┘     └────┘ └────┘    │    │
    │  │                                                      │    │
    │  └──────────────────────────────────────────────────────┘    │
    │                                                              │
    │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
    │  │ Local Memory │  │ Texture Unit │  │ Special Function │  │
    │  │   (64 KB)    │  │   (Vision)   │  │  Unit (SFU)      │  │
    │  └──────────────┘  └──────────────┘  └──────────────────┘  │
    │                                                              │
    └─────────────────────────────────────────────────────────────┘

Supported Vision Operations:
    - 2D Convolution (3x3, 5x5, 7x7 kernels)
    - Feature detection (Harris, FAST, ORB, SIFT)
    - Optical flow (Lucas-Kanade, Horn-Schunck)
    - Image transforms (FFT, DCT, Wavelet)
    - Histogram operations
    - Color space conversion
    - Morphological operations

Memory Hierarchy:
    - Registers: 256 x 256-bit per PE
    - Local (Shared) Memory: 64 KB per SIMD unit
    - Texture Cache: 16 KB (optimized for 2D access patterns)
    - L1 Cache: Unified with CPU
"""

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

class NOVASIMDUnit(SimObject):
    """
    NOVA GPU-like SIMD Unit for Vision Processing
    """
    type = 'NOVASIMDUnit'
    cxx_header = "cpu/nova_simd/nova_simd_unit.hh"
    cxx_class = 'gem5::NOVASIMDUnit'
    
    # ============== Core Configuration ==============
    
    # Number of processing elements (PEs) per SIMD unit
    # Each PE is a 256-bit SIMD lane
    num_pes = Param.Int(32, "Number of processing elements")
    
    # SIMD width in bits (256-bit = 8 x FP32 or 16 x INT16)
    simd_width = Param.Int(256, "SIMD width in bits")
    
    # Warp/Wavefront size (number of threads executed in lockstep)
    warp_size = Param.Int(32, "Warp/Wavefront size")
    
    # Maximum warps per SIMD unit
    max_warps = Param.Int(16, "Maximum concurrent warps")
    
    # ============== Memory Configuration ==============
    
    # Local (shared) memory size
    local_memory_size = Param.MemorySize('64kB', "Local scratchpad memory")
    
    # Register file size per PE (in 256-bit registers)
    registers_per_pe = Param.Int(256, "Registers per PE")
    
    # Texture cache size
    texture_cache_size = Param.MemorySize('16kB', "Texture cache size")
    
    # ============== Latency Configuration ==============
    
    # Arithmetic latency (in cycles)
    fp32_latency = Param.Int(4, "FP32 operation latency")
    fp64_latency = Param.Int(8, "FP64 operation latency")
    int_latency = Param.Int(1, "Integer operation latency")
    
    # Special function unit latency
    sfu_latency = Param.Int(8, "Special function unit latency (sin, cos, sqrt)")
    
    # Memory latency
    local_mem_latency = Param.Int(2, "Local memory access latency")
    texture_latency = Param.Int(4, "Texture cache access latency")
    
    # ============== Power Configuration ==============
    
    # Power states
    idle_power = Param.Float(0.5, "Idle power in Watts")
    active_power = Param.Float(5.0, "Active power in Watts")
    
    # ============== Vision-Specific Units ==============
    
    # Convolution accelerator
    conv_units = Param.Int(4, "Number of convolution units")
    conv_kernel_max = Param.Int(7, "Maximum convolution kernel size")
    
    # Feature detector units
    feature_units = Param.Int(2, "Number of feature detection units")
    
    # Optical flow units
    flow_units = Param.Int(2, "Number of optical flow units")
    
    # ============== Statistics ==============
    
    # Enable detailed statistics
    enable_stats = Param.Bool(True, "Enable detailed statistics")


class NOVASIMDCluster(SimObject):
    """
    Cluster of NOVA SIMD units sharing L2 cache.
    """
    type = 'NOVASIMDCluster'
    cxx_header = "cpu/nova_simd/nova_simd_cluster.hh"
    cxx_class = 'gem5::NOVASIMDCluster'
    abstract = False
    
    # Number of SIMD units in cluster
    num_simd_units = Param.Int(2, "Number of SIMD units per cluster")
    
    # Shared L2 cache size
    l2_size = Param.MemorySize('256kB', "Shared L2 cache size")
    
    # Cluster-level scheduler
    scheduler_policy = Param.String('round_robin', 
                                     "Workload scheduling policy")


class NOVAVisionDispatcher(SimObject):
    """
    Dispatcher for vision workloads to SIMD units.
    """
    type = 'NOVAVisionDispatcher'
    cxx_header = "cpu/nova_simd/nova_vision_dispatcher.hh"
    cxx_class = 'gem5::NOVAVisionDispatcher'
    abstract = False
    
    # Maximum queued tasks
    max_queue_depth = Param.Int(64, "Maximum queued vision tasks")
    
    # Task granularity (tile size for image processing)
    tile_width = Param.Int(64, "Tile width for processing")
    tile_height = Param.Int(64, "Tile height for processing")
    
    # Dispatch policy
    dispatch_policy = Param.String('load_balance',
                                    "Dispatch policy: load_balance, round_robin, affinity")


# Copyright (c) 2024 IISc Bangalore
# PhD Research: Futuristic Spacecraft Processor
# Author: Boul, PhD Scholar, CSA

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class SpaceGPU(ClockedObject):
    """
    Space-Grade Low-Power GPU for Spacecraft Applications
    
    A specialized GPU designed for spacecraft visualization and compute needs:
    - Terrain rendering for autonomous landing
    - Star field rendering for optical navigation
    - Sensor data visualization (thermal, hazard, depth maps)
    - GPGPU compute for parallel processing
    
    Features:
    - Configurable SIMD lanes (4-16)
    - Fixed-function rasterizer
    - Simple compute kernels
    - Ultra-low power operation
    - Power gating support
    """
    
    type = 'SpaceGPU'
    cxx_header = "spacecraft/space_gpu.hh"
    cxx_class = 'gem5::spacecraft::SpaceGPU'
    
    # Ports
    dma_port = RequestPort("DMA port for memory access")
    mmio_port = ResponsePort("MMIO port for register access")
    
    # Instance identification
    instance_id = Param.Int(0, "GPU instance ID")
    
    # Framebuffer configuration
    framebuffer_base = Param.Addr(0x80000000, "Framebuffer base address")
    framebuffer_width = Param.UInt32(640, "Framebuffer width in pixels")
    framebuffer_height = Param.UInt32(480, "Framebuffer height in pixels")
    
    # Processing configuration
    simd_lanes = Param.Int(8, "Number of SIMD processing lanes")
    max_triangles_per_batch = Param.Int(256, "Maximum triangles per batch")
    
    # Memory configuration
    texture_cache_size = Param.MemorySize("64KiB", "Texture cache size")
    
    # MMIO configuration
    mmio_base = Param.Addr(0x20000000, "MMIO register base address")
    mmio_size = Param.MemorySize("4KiB", "MMIO register space size")
    
    # Latency parameters (in cycles)
    power_up_latency = Param.Int(100, "Cycles to power up from off state")
    sleep_wake_latency = Param.Int(20, "Cycles to wake from sleep")
    triangle_setup_latency = Param.Int(4, "Cycles for triangle setup")
    pixel_latency = Param.Int(1, "Cycles per pixel shader")
    texture_latency = Param.Int(8, "Cycles for texture sample")
    compute_base_latency = Param.Int(10, "Base compute kernel latency")
    memory_latency = Param.Int(50, "Memory access latency")
    
    # Power configuration (in milliwatts)
    power_off = Param.Float(0.0, "Power when off (mW)")
    power_sleep = Param.Float(5.0, "Power in sleep mode (mW)")
    power_idle = Param.Float(50.0, "Power when idle (mW)")
    power_render = Param.Float(250.0, "Power during rendering (mW)")
    power_compute = Param.Float(300.0, "Power during compute (mW)")
    power_full = Param.Float(400.0, "Maximum power (mW)")

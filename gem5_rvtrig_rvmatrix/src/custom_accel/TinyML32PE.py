# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# TinyML Accelerator with 32 Processing Elements - Python Configuration

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class TinyML32PE(ClockedObject):
    """
    Enhanced TinyML Accelerator with 32 Processing Elements for spacecraft
    on-board AI/ML inference.
    
    Features:
    - 4x8 PE array (32 PEs total)
    - INT8/INT16/FP16 quantized inference
    - Weight-stationary dataflow
    - DMA engine for efficient data movement
    - Support for Conv2D, FC, Pooling, Activation layers
    
    Target applications:
    - Autonomous navigation (GNC)
    - Anomaly detection
    - Image classification
    - Predictive maintenance
    """
    type = 'TinyML32PE'
    cxx_header = "custom_accel/tiny_ml_32pe.hh"
    cxx_class = 'gem5::TinyML32PE'
    
    # Ports
    cpu_side = ResponsePort("CPU-side port for MMIO access")
    mem_side = RequestPort("Memory-side port for DMA")
    
    # Address range for memory-mapped registers
    addr_range = Param.AddrRange("Address range for MMIO")
    
    # Memory hierarchy configuration
    global_buffer_size = Param.MemorySize("128kB", 
        "Global buffer (activation) SRAM size")
    weight_buffer_size = Param.MemorySize("64kB",
        "Weight buffer SRAM size")
    accum_buffer_size = Param.MemorySize("32kB",
        "Accumulator buffer size")
    
    # PE array configuration (4x8 = 32 PEs is fixed in hardware)
    # These parameters affect performance modeling
    pe_latency = Param.Cycles(1, "Cycles per MAC operation in each PE")
    
    # DMA configuration
    dma_latency = Param.Cycles(5, "Cycles per DMA transfer")
    data_width = Param.Unsigned(16, "Data bus width in bytes")
    
    # System connection
    system = Param.System(Parent.any, "System this accelerator belongs to")




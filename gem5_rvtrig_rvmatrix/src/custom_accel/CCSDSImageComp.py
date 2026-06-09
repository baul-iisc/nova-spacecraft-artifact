# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# CCSDS 122.0 Image Compression Accelerator - Python Configuration

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class CCSDSImageComp(ClockedObject):
    """
    CCSDS 122.0-B-2 Image Data Compression Accelerator
    
    Implements the CCSDS image compression standard for spacecraft:
    - Discrete Wavelet Transform (DWT) using 9/7 or 5/3 filters
    - Bit-Plane Encoder (BPE) with segment-based coding
    - Configurable compression ratio
    - Lossless and lossy modes
    - Support for 8-16 bit samples
    
    Target applications:
    - Earth observation imagery
    - Science instrument data
    - Surveillance imagery
    - Planetary exploration data
    """
    type = 'CCSDSImageComp'
    cxx_header = "custom_accel/ccsds_image_comp.hh"
    cxx_class = 'gem5::CCSDSImageComp'
    
    # Ports
    cpu_side = ResponsePort("CPU-side port for MMIO access")
    mem_side = RequestPort("Memory-side port for DMA")
    
    # Address range for memory-mapped registers
    addr_range = Param.AddrRange("Address range for MMIO")
    
    # Latency parameters
    dwt_latency = Param.Cycles(2, 
        "Cycles per DWT coefficient computation")
    bpe_latency = Param.Cycles(50,
        "Cycles per BPE segment encoding")
    
    # Buffer sizes
    line_buffer_size = Param.Unsigned(4096,
        "Line buffer size for DWT (pixels)")
    max_tile_size = Param.Unsigned(65536,
        "Maximum tile size in pixels (256x256 default)")
    
    # System connection
    system = Param.System(Parent.any, "System this accelerator belongs to")




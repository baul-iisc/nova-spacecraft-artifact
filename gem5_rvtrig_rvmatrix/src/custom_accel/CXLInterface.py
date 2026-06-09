"""
CXLInterface.py - SimObject for CXL Interface Controller

PhD Research: Chandraboul

CXL (Compute Express Link) interface supporting:
- CXL.io: PCIe-like I/O protocol
- CXL.cache: Cache coherent access to host memory  
- CXL.mem: Memory access protocol
"""

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class CXLInterface(ClockedObject):
    """
    CXL Interface Controller with 4 ports
    
    Provides high-bandwidth, low-latency connectivity for:
    - Memory expansion (CXL.mem)
    - Cache-coherent accelerator access (CXL.cache)
    - Standard I/O operations (CXL.io)
    """
    type = 'CXLInterface'
    cxx_header = 'custom_accel/cxl_interface.hh'
    cxx_class = 'gem5::CXLInterface'

    # Memory-mapped register interface
    addr_range = Param.AddrRange(AddrRange(0x50010000, size='4kB'),
                                 "Address range for control registers")

    # Link configuration
    link_speed = Param.Unsigned(32, "Link speed in GT/s (CXL 2.0: 32 GT/s)")
    link_width = Param.Unsigned(16, "Link width (x1, x2, x4, x8, x16)")
    
    # Timing parameters
    access_latency = Param.Cycles(10, "Base access latency in cycles")

    # Ports
    cpu_side = ResponsePort("CPU-side port for control access")
    mem_side = VectorRequestPort("Memory-side ports for CXL memory access")


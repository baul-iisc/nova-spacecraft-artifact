"""
DebugInterface.py - SimObject for Hardware Debug Interface

PhD Research: Chandraboul

Hardware debug interface supporting:
- Breakpoint management
- Memory read/write
- Register read/write  
- Single-step execution
- Run/halt control
"""

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class DebugInterface(ClockedObject):
    """
    Hardware Debug Interface Controller
    
    Provides debugging capabilities:
    - Up to 8 hardware breakpoints
    - Memory access through debug port
    - Register file access
    - Execution control (halt, resume, single-step)
    """
    type = 'DebugInterface'
    cxx_header = 'custom_accel/debug_interface.hh'
    cxx_class = 'gem5::DebugInterface'

    # Memory-mapped register interface
    addr_range = Param.AddrRange(AddrRange(0x50020000, size='4kB'),
                                 "Address range for control registers")

    # Configuration
    num_breakpoints = Param.Unsigned(8, "Number of hardware breakpoints")
    num_cores = Param.Unsigned(8, "Number of cores to debug")

    # Ports
    cpu_side = ResponsePort("CPU-side port for control access")
    mem_side = RequestPort("Memory-side port for memory access")


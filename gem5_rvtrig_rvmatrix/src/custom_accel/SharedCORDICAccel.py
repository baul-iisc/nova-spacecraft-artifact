from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class SharedCORDICAccel(ClockedObject):
    """Shared CORDIC Trigonometric Accelerator with MMIO and Arbitration"""
    
    type = 'SharedCORDICAccel'
    cxx_header = "custom_accel/shared_cordic_accel.hh"
    cxx_class = 'gem5::SharedCORDICAccel'
    
    # MMIO port for CPU access
    mmio_port = ResponsePort("MMIO port for CPU access")
    
    # Number of cores sharing this accelerator
    num_cores = Param.Int(4, "Number of cores sharing this accelerator")
    
    # Latency parameters
    compute_latency = Param.Cycles(5, "Cycles to compute one trig operation")
    arbitration_latency = Param.Cycles(1, "Cycles for arbitration overhead")
    
    # MMIO address space
    mmio_base = Param.Addr(0x50010000, "MMIO base address")
    mmio_size = Param.Addr(0x1000, "MMIO address space size")

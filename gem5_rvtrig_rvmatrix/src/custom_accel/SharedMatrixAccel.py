from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class SharedMatrixAccel(ClockedObject):
    """Shared 3x3 Matrix Accelerator with MMIO and Arbitration"""
    
    type = 'SharedMatrixAccel'
    cxx_header = "custom_accel/shared_matrix_accel.hh"
    cxx_class = 'gem5::SharedMatrixAccel'
    
    # MMIO port for CPU access
    mmio_port = ResponsePort("MMIO port for CPU access")
    
    # Number of cores sharing this accelerator
    num_cores = Param.Int(4, "Number of cores sharing this accelerator")
    
    # Latency parameters
    compute_latency = Param.Cycles(10, "Cycles to compute one 3x3 matrix multiply")
    arbitration_latency = Param.Cycles(2, "Cycles for arbitration overhead")
    
    # MMIO address space
    mmio_base = Param.Addr(0x50000000, "MMIO base address")
    mmio_size = Param.Addr(0x1000, "MMIO address space size")

# NOVA Processor - MMIO Trigonometric Accelerator Python Definition
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.Device import BasicPioDevice


class MMIOTrigAccel(BasicPioDevice):
    """
    MMIO-based Trigonometric Accelerator with contention modeling.
    
    This accelerator can be shared among multiple cores. When multiple
    cores try to use it simultaneously, requests are queued and the
    contention is measured.
    
    Key parameters:
    - processing_latency: Cycles to complete one operation
    - max_queue_depth: Maximum number of pending requests
    
    Statistics exported:
    - totalQueuedRequests: Number of requests that had to wait
    - totalQueueWaitCycles: Total cycles spent waiting
    - maxQueueDepthObserved: Maximum contention observed
    """
    
    type = 'MMIOTrigAccel'
    cxx_header = 'spacecraft/mmio_trig_accel.hh'
    cxx_class = 'gem5::spacecraft::MMIOTrigAccel'
    
    # Processing latency per operation (in cycles)
    # CORDIC typically takes 10-20 iterations, each 1-2 cycles
    processing_latency = Param.Cycles(15, 
        "Number of cycles to complete one trigonometric operation")
    
    # Maximum queue depth for pending requests
    max_queue_depth = Param.Int(16, 
        "Maximum number of pending requests in queue")


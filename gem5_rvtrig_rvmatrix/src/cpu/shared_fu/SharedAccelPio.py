# NOVA Processor - Shared Accelerator PIO Device
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.Device import PioDevice


class SharedAccelPio(PioDevice):
    """
    Shared Accelerator as a PIO Device.
    
    This device models a shared hardware accelerator that can be accessed
    by multiple CPU cores. When multiple cores access it simultaneously:
    - The first request is served immediately (after operation latency)
    - Subsequent requests are queued
    - Contention causes increased response latency
    
    The device integrates with gem5's timing model to cause actual CPU
    stalls when the accelerator is busy.
    
    Register Map:
    - 0x00: CTRL      - Control (write: start op, read: accel type)
    - 0x08: STATUS    - Status (bit 0: busy, bits 8-15: queue depth)
    - 0x10: INPUT0    - Input operand 0
    - 0x18: INPUT1    - Input operand 1
    - 0x20: OUTPUT0   - Output result 0
    - 0x28: OUTPUT1   - Output result 1
    - 0x30: LATENCY   - Current operation latency
    - 0x38: STATS     - Request statistics
    
    Accelerator types:
    - 0: Trigonometric (CORDIC)
    - 1: Matrix (3x3)
    
    Operation types:
    - Trig: 0=sin, 1=cos, 2=tan, 3=atan, 4=atan2, 5=sincos
    - Matrix: 16=mul3x3, 17=transpose, 18=inverse, 19=matvec
    
    Usage example:
        # In Python config:
        system.trig_accel = SharedAccelPio(
            pio_addr = 0x40000000,
            accel_type = 0,
            trig_latency = 15,
            instance_id = 0
        )
        system.trig_accel.pio = system.membus.mem_side_ports
    """
    
    type = 'SharedAccelPio'
    cxx_header = 'cpu/shared_fu/shared_accel_pio.hh'
    cxx_class = 'gem5::SharedAccelPio'
    
    # Instance configuration
    instance_id = Param.Int(0, "Instance ID for this accelerator")
    
    # Accelerator type: 0=Trig, 1=Matrix
    accel_type = Param.Int(0, "Accelerator type (0=Trig, 1=Matrix)")
    
    # PIO address
    pio_addr = Param.Addr(0x40000000, "Base address for MMIO registers")
    
    # Operation latencies (in cycles)
    trig_latency = Param.Cycles(15, "Latency for trigonometric operations")
    mat_latency = Param.Cycles(50, "Latency for 3x3 matrix operations")
    
    # Queue configuration
    max_queue_depth = Param.Unsigned(16, "Maximum queue depth for requests")



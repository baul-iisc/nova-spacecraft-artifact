# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# CCSDS TM/TC Protocol Accelerator - Python Configuration

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class CCSDSTmTc(ClockedObject):
    """
    CCSDS Telemetry/Telecommand Protocol Accelerator
    
    Implements CCSDS space communication standards:
    - CCSDS 132.0-B-2 TM Space Data Link Protocol
    - CCSDS 232.0-B-3 TC Space Data Link Protocol
    - CCSDS 131.0-B-3 TM Synchronization and Channel Coding
    - CCSDS 231.0-B-3 TC Synchronization and Channel Coding
    - CCSDS 133.0-B-1 Space Packet Protocol
    
    Features:
    - TM frame encoding/decoding
    - TC frame encoding/decoding
    - Space packet assembly/extraction
    - Reed-Solomon RS(255,223) coding
    - Convolutional coding (rate 1/2, K=7)
    - CCSDS randomization
    - CRC-16 computation/verification
    """
    type = 'CCSDSTmTc'
    cxx_header = "custom_accel/ccsds_tmtc.hh"
    cxx_class = 'gem5::CCSDSTmTc'
    
    # Ports
    cpu_side = ResponsePort("CPU-side port for MMIO access")
    mem_side = RequestPort("Memory-side port for DMA")
    
    # Address range for memory-mapped registers
    addr_range = Param.AddrRange("Address range for MMIO")
    
    # Latency parameters
    frame_latency = Param.Cycles(100, 
        "Base latency for frame processing")
    rs_latency = Param.Cycles(200,
        "Latency for Reed-Solomon encoding/decoding per codeword")
    conv_latency = Param.Cycles(50,
        "Latency for convolutional encoding per frame")
    
    # System connection
    system = Param.System(Parent.any, "System this accelerator belongs to")




# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# SpaceWire Network Interface Controller - Python Configuration

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class SpaceWireNIC(ClockedObject):
    """
    SpaceWire Network Interface Controller
    
    Implements ECSS-E-ST-50-12C SpaceWire standard for spacecraft:
    - Point-to-point serial data links
    - Data-Strobe (DS) encoding
    - Credit-based flow control
    - RMAP (Remote Memory Access Protocol) support
    - Time-code distribution
    
    Target applications:
    - Spacecraft internal communication
    - Payload data handling
    - Remote memory access
    - Time synchronization
    """
    type = 'SpaceWireNIC'
    cxx_header = "custom_accel/spacewire_nic.hh"
    cxx_class = 'gem5::SpaceWireNIC'
    
    # Ports
    cpu_side = ResponsePort("CPU-side port for MMIO access")
    mem_side = RequestPort("Memory-side port for DMA")
    
    # Address range for memory-mapped registers
    addr_range = Param.AddrRange("Address range for MMIO")
    
    # Link configuration
    link_speed = Param.Unsigned(200, "Link speed in Mbps (2-400)")
    node_address = Param.UInt8(32, "SpaceWire logical address (0-255)")
    
    # FIFO configuration
    tx_fifo_depth = Param.Unsigned(16, "TX FIFO depth in packets")
    rx_fifo_depth = Param.Unsigned(16, "RX FIFO depth in packets")
    max_packet_size = Param.Unsigned(4096, "Maximum packet size in bytes")
    
    # RMAP configuration
    rmap_key = Param.UInt8(0, "RMAP destination key (0=disabled)")
    
    # System connection
    system = Param.System(Parent.any, "System this NIC belongs to")




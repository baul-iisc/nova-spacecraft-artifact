# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# TSN Ethernet Controller - Python Configuration

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class TSNEthernet(ClockedObject):
    """
    Time-Sensitive Networking (TSN) Ethernet Controller
    
    Implements IEEE 802.1 TSN standards for deterministic networking:
    - IEEE 802.1AS-2020 Time synchronization (gPTP)
    - IEEE 802.1Qbv-2015 Time-aware shaper (TAS)
    - IEEE 802.1Qci-2017 Per-stream filtering and policing
    - IEEE 802.1CB-2017 Frame replication and elimination
    
    Target applications:
    - Next-generation spacecraft avionics
    - Deterministic real-time communication
    - Time-synchronized distributed systems
    - Safety-critical data distribution
    """
    type = 'TSNEthernet'
    cxx_header = "custom_accel/tsn_ethernet.hh"
    cxx_class = 'gem5::TSNEthernet'
    
    # Ports
    cpu_side = ResponsePort("CPU-side port for MMIO access")
    mem_side = RequestPort("Memory-side port for DMA")
    
    # Address range for memory-mapped registers
    addr_range = Param.AddrRange("Address range for MMIO")
    
    # MAC configuration
    mac_address = Param.UInt64(0x001122334455, "MAC address (48 bits)")
    
    # Link configuration
    link_speed = Param.Unsigned(1000, "Link speed in Mbps (10/100/1000)")
    
    # FIFO configuration
    tx_fifo_depth = Param.Unsigned(64, "TX FIFO depth in frames")
    rx_fifo_depth = Param.Unsigned(64, "RX FIFO depth in frames")
    max_frame_size = Param.Unsigned(1522, "Maximum frame size (VLAN)")
    
    # TSN configuration
    gcl_max_entries = Param.Unsigned(256, "Maximum Gate Control List entries")
    
    # System connection
    system = Param.System(Parent.any, "System this controller belongs to")




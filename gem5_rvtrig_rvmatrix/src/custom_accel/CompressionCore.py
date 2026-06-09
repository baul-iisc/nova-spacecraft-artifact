# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# Python SimObject wrapper for CompressionCore

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class CompressionCore(ClockedObject):
    """
    CompressionCore - CCSDS Rice-like compression accelerator for spacecraft
    
    This accelerator provides hardware-accelerated compression/decompression
    for spacecraft telemetry and science data, helping to reduce downlink
    bandwidth requirements and save power.
    
    Features:
    - CCSDS Rice coding (used in space missions)
    - Configurable k-parameter for different data types
    - Configurable block size
    - DMA interface for efficient data transfer
    
    Memory-Mapped Register Interface:
    - 0x00: CTRL - Control (start, mode, reset)
    - 0x04: STATUS - Status (busy, done, error)
    - 0x08: SRC_ADDR - Source address
    - 0x0C: DST_ADDR - Destination address
    - 0x10: LENGTH - Input length
    - 0x14: OUT_LEN - Output length (read-only)
    - 0x18: CONFIG - Configuration (k-param, block_size)
    - 0x1C: STATS - Compression statistics
    """
    
    type = "CompressionCore"
    cxx_header = "custom_accel/compression_core.hh"
    cxx_class = "gem5::CompressionCore"

    # Ports
    cpu_side = ResponsePort("CPU side port for MMIO access")
    mem_side = RequestPort("Memory side port for DMA transfers")

    # Address range for memory-mapped registers
    addr_range = Param.AddrRange(
        AddrRange(0x10000000, size='256B'),
        "Address range for compression core registers"
    )

    # Compression parameters
    k_param = Param.Unsigned(4, 
        "Rice k-parameter (bits for remainder encoding). "
        "Typical values: 4-8 depending on data entropy"
    )
    
    block_size = Param.Unsigned(64,
        "Block size in bytes for compression. "
        "Larger blocks = better compression but more latency"
    )
    
    compression_latency = Param.Cycles(10,
        "Latency in cycles to compress/decompress one block"
    )

    # System connection (for DMA)
    system = Param.System(Parent.any, "System this core belongs to")


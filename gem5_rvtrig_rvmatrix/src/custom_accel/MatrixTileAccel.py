# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# DianNao-Style Matrix Tile Accelerator Python Wrapper

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class MatrixTileAccel(ClockedObject):
    """
    DianNao-inspired tiled matrix multiplication accelerator.
    
    Features:
    - Hardware-managed tiling for arbitrary matrix sizes
    - Input buffer (NBin-style) for A matrix tiles
    - Weight buffer (SB-style) for B matrix tiles
    - Accumulator buffer for partial sums
    - Output buffer (NBout-style) for results
    - DMA engine for efficient memory transfers
    - 3x3 tile-based systolic MAC array
    
    This accelerator eliminates software tiling overhead by handling
    tile extraction, accumulation, and writeback in hardware.
    """
    
    type = 'MatrixTileAccel'
    cxx_header = "custom_accel/matrix_tile_accel.hh"
    cxx_class = "gem5::MatrixTileAccel"
    
    # Ports
    cpu_side = ResponsePort("CPU side port for MMIO register access")
    mem_side = RequestPort("Memory side port for DMA transfers")
    
    # Address range for MMIO registers
    addr_range = Param.AddrRange(AddrRange(0x60000000, size='4kB'),
                                  "Address range for MMIO registers")
    
    # Buffer sizes (DianNao-style on-chip SRAM)
    input_buffer_size = Param.MemorySize('4kB',
        "Size of input buffer (NBin) for A matrix tiles")
    weight_buffer_size = Param.MemorySize('4kB', 
        "Size of weight buffer (SB) for B matrix tiles")
    accum_buffer_size = Param.MemorySize('2kB',
        "Size of accumulator buffer for partial sums")
    output_buffer_size = Param.MemorySize('2kB',
        "Size of output buffer (NBout) for results")
    
    # Latency parameters
    mac_latency = Param.Cycles(3, 
        "Cycles to execute one 3x3 tile MAC operation (27 FLOPs)")
    dma_latency = Param.Cycles(10,
        "Cycles per DMA transfer for one tile")
    
    # System connection
    system = Param.System(Parent.any, "System this accelerator is part of")


# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# TinyTPU - TPU-like Matrix Multiply Accelerator

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class TinyTPU(ClockedObject):
    """
    TinyTPU - A simplified TPU-like matrix multiply accelerator.
    
    Inspired by Google's Tensor Processing Unit (TPU), featuring:
    - 8x8 Systolic Array (Matrix Multiply Unit - MXU)
    - Weight Stationary Dataflow
    - Unified Buffer for Activations
    - Accumulator Buffer for Partial Sums
    - Activation Unit (ReLU, Sigmoid, Tanh, etc.)
    - INT8 Quantized Inference
    
    Key TPU concepts implemented:
    1. Systolic Array: Data flows through the array in a wave pattern
    2. Weight Stationary: Weights are loaded once and stay in PEs
    3. Tiled Execution: Large matrices are processed in tiles
    4. Fused Operations: GEMM + Activation in one pass
    
    Performance characteristics:
    - Peak throughput: 8x8x2 = 128 INT8 MACs/cycle
    - Tile size: 8x8 (matches MXU dimensions)
    - Supports matrices of any size via tiling
    
    Usage:
        system.tpu = TinyTPU()
        system.tpu.cpu_side = system.membus.mem_side_ports
        system.tpu.mem_side = system.membus.cpu_side_ports
    """
    
    type = 'TinyTPU'
    cxx_header = "custom_accel/tiny_tpu.hh"
    cxx_class = "gem5::TinyTPU"
    
    # Ports
    cpu_side = ResponsePort("CPU side port for MMIO register access")
    mem_side = RequestPort("Memory side port for DMA transfers")
    
    # Address range for MMIO registers
    addr_range = Param.AddrRange(AddrRange(0x70000000, size='4kB'),
                                  "Address range for MMIO registers")
    
    # On-chip buffer sizes
    unified_buffer_size = Param.MemorySize('64kB',
        "Size of unified buffer for activations (like TPU's 24MB unified buffer, scaled down)")
    weight_fifo_size = Param.MemorySize('32kB',
        "Size of weight FIFO for streaming weights to MXU")
    accum_buffer_size = Param.MemorySize('4kB',
        "Size of accumulator buffer for 32-bit partial sums")
    
    # Latency parameters
    mxu_latency = Param.Cycles(1,
        "Cycles per MXU tile operation (8x8 MAC)")
    dma_latency = Param.Cycles(5,
        "Cycles per DMA transfer for one tile")
    
    # System connection
    system = Param.System(Parent.any, "System this TPU is part of")


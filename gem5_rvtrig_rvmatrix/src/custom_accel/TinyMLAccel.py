# Copyright (c) 2024 Chandraboul - PhD Research
# Spacecraft Heterogeneous Multicore Processor
#
# Python SimObject wrapper for TinyMLAccel

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class TinyMLAccel(ClockedObject):
    """
    TinyMLAccel - Lightweight ML accelerator for spacecraft AI applications
    
    This accelerator enables on-board AI/ML processing for spacecraft,
    supporting tasks like:
    - Anomaly detection in telemetry
    - Image classification (cloud detection, feature extraction)
    - Autonomous navigation assistance
    - Predictive maintenance
    
    Features:
    - INT8 quantized inference for power efficiency
    - Configurable MAC (Multiply-Accumulate) array
    - On-chip SRAM for weights and activations
    - Built-in activation functions (ReLU, Sigmoid, TanH)
    - Support for Conv2D, MatMul, Pooling operations
    
    Memory-Mapped Register Interface:
    - 0x00: CTRL - Control (start, operation type, reset)
    - 0x04: STATUS - Status (busy, done, error)
    - 0x08: INPUT_ADDR - Input tensor address
    - 0x0C: WEIGHT_ADDR - Weight tensor address
    - 0x10: OUTPUT_ADDR - Output tensor address
    - 0x14: INPUT_DIM - Input dimensions (H, W, C)
    - 0x18: WEIGHT_DIM - Weight dimensions
    - 0x1C: CONFIG - Configuration (activation, stride, padding)
    - 0x20: MAC_CONFIG - MAC array configuration
    - 0x24: PERF_CNT - Performance counter
    """
    
    type = "TinyMLAccel"
    cxx_header = "custom_accel/tiny_ml_accel.hh"
    cxx_class = "gem5::TinyMLAccel"

    # Ports
    cpu_side = ResponsePort("CPU side port for MMIO access")
    mem_side = RequestPort("Memory side port for DMA transfers")

    # Address range for memory-mapped registers
    addr_range = Param.AddrRange(
        AddrRange(0x10001000, size='256B'),
        "Address range for ML accelerator registers"
    )

    # MAC Array Configuration
    mac_array_rows = Param.Unsigned(8,
        "Number of rows in the MAC array. "
        "Determines parallel computation capability"
    )
    
    mac_array_cols = Param.Unsigned(8,
        "Number of columns in the MAC array. "
        "Total MACs = rows * cols"
    )

    # On-chip memory
    sram_size = Param.MemorySize('64kB',
        "On-chip SRAM size for weights and activations. "
        "Larger SRAM reduces external memory accesses"
    )

    # Timing parameters
    mac_latency = Param.Cycles(1,
        "Latency per MAC operation in the array"
    )
    
    dma_latency = Param.Cycles(5,
        "Latency per DMA transfer (per 64 bytes)"
    )

    # System connection
    system = Param.System(Parent.any, "System this accelerator belongs to")


class TinyMLAccelConfig:
    """
    Helper class for common ML accelerator configurations
    """
    
    @staticmethod
    def tiny():
        """4x4 MAC array, 16KB SRAM - Ultra low power"""
        return {
            'mac_array_rows': 4,
            'mac_array_cols': 4,
            'sram_size': '16kB',
            'mac_latency': 1,
            'dma_latency': 5
        }
    
    @staticmethod
    def small():
        """8x8 MAC array, 64KB SRAM - Balanced"""
        return {
            'mac_array_rows': 8,
            'mac_array_cols': 8,
            'sram_size': '64kB',
            'mac_latency': 1,
            'dma_latency': 5
        }
    
    @staticmethod
    def medium():
        """16x16 MAC array, 256KB SRAM - Higher performance"""
        return {
            'mac_array_rows': 16,
            'mac_array_cols': 16,
            'sram_size': '256kB',
            'mac_latency': 1,
            'dma_latency': 4
        }


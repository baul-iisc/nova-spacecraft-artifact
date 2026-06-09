# NOVA Processor - Neural Processing Unit (NPU) SimObject
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class NeuralProcessingUnit(ClockedObject):
    """
    Neural Processing Unit (NPU) / TinyML Accelerator for spacecraft AI.
    
    Features:
    - Configurable systolic array (NxN MAC units)
    - INT8/INT4 quantized operations with sparsity support
    - Layer types: Conv, FC, Pooling, Activations
    - Weight cache and activation scratchpad
    - DVFS support for power management
    
    Target models: MobileNetV2/V3, EfficientNet-Lite, YOLOv4-tiny
    """
    
    type = 'NeuralProcessingUnit'
    cxx_header = 'spacecraft/neural_processing_unit.hh'
    cxx_class = 'gem5::spacecraft::NeuralProcessingUnit'
    
    # Instance identification
    instance_id = Param.Int(0, "NPU instance ID")
    
    # Ports
    dma_port = RequestPort("DMA port for memory access")
    mmio_port = ResponsePort("MMIO port for CPU control")
    
    # MMIO configuration
    mmio_base = Param.Addr(0xA2000000, "MMIO base address")
    mmio_size = Param.Addr(0x1000, "MMIO address range size")
    
    # Systolic array configuration
    array_size = Param.Int(8, "Systolic array size (NxN, e.g., 8 = 8x8 = 64 MACs)")
    
    # Memory configuration
    weight_cache_kb = Param.Int(64, "Weight cache size in KB")
    activation_spad_kb = Param.Int(32, "Activation scratchpad size in KB")
    
    # Feature support
    sparse_support = Param.Bool(True, "Enable zero-skipping for sparse networks")
    
    # Latency parameters (in cycles)
    dma_latency_per_byte = Param.Cycles(1, "DMA latency per byte transferred")
    activation_latency = Param.Cycles(10, "Activation function latency")
    pooling_latency = Param.Cycles(20, "Pooling layer latency")
    batch_norm_latency = Param.Cycles(15, "Batch normalization latency")
    
    # Power configuration
    power_up_latency = Param.Cycles(200, "Power-up latency from gated state")
    active_power_mw = Param.Float(500.0, "Active power in milliwatts")
    idle_power_mw = Param.Float(50.0, "Idle power in milliwatts")
    gated_power_mw = Param.Float(1.0, "Power-gated leakage in milliwatts")
    
    # DVFS configuration
    min_freq_scale = Param.Float(0.3, "Minimum frequency scale (30%)")
    max_freq_scale = Param.Float(1.0, "Maximum frequency scale (100%)")
    dvfs_transition_latency = Param.Cycles(50, "DVFS state transition latency")





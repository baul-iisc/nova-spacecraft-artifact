# NOVA Processor - Vision Processing Unit (VPU) SimObject
# PhD Research: Futuristic Spacecraft Processor

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class VisionProcessingUnit(ClockedObject):
    """
    Vision Processing Unit (VPU) for spacecraft image processing.
    
    Features:
    - Image preprocessing (debayering, tone mapping, gamma)
    - Edge detection (Sobel, Canny)
    - Feature extraction (Harris corners, ORB, optical flow)
    - 2D convolution engine with configurable kernel sizes
    """
    
    type = 'VisionProcessingUnit'
    cxx_header = 'spacecraft/vision_processing_unit.hh'
    cxx_class = 'gem5::spacecraft::VisionProcessingUnit'
    
    # Instance identification
    instance_id = Param.Int(0, "VPU instance ID")
    
    # Ports
    dma_port = RequestPort("DMA port for memory access")
    mmio_port = ResponsePort("MMIO port for CPU control")
    
    # MMIO configuration
    mmio_base = Param.Addr(0xA1000000, "MMIO base address")
    mmio_size = Param.Addr(0x1000, "MMIO address range size")
    
    # Hardware configuration
    num_mac_units = Param.Int(16, "Number of parallel MAC units for convolution")
    max_kernel_size = Param.Int(7, "Maximum supported kernel size (7x7)")
    scratchpad_size_kb = Param.Int(64, "Scratchpad/buffer size in KB")
    
    # Latency parameters (in cycles)
    debayer_latency = Param.Cycles(100, "Debayering latency")
    tone_mapping_latency = Param.Cycles(50, "Tone mapping latency")
    gamma_latency = Param.Cycles(30, "Gamma correction latency")
    sobel_latency = Param.Cycles(200, "Sobel edge detection latency")
    canny_latency = Param.Cycles(500, "Canny edge detection latency")
    harris_latency = Param.Cycles(1000, "Harris corner detection latency")
    orb_latency = Param.Cycles(2000, "ORB feature extraction latency")
    optical_flow_latency = Param.Cycles(3000, "Optical flow computation latency")
    conv_base_latency = Param.Cycles(50, "Base convolution overhead latency")
    
    # Power configuration
    power_up_latency = Param.Cycles(100, "Power-up latency from gated state")
    active_power_mw = Param.Float(100.0, "Active power in milliwatts")
    idle_power_mw = Param.Float(10.0, "Idle power in milliwatts")
    gated_power_mw = Param.Float(0.5, "Power-gated leakage in milliwatts")





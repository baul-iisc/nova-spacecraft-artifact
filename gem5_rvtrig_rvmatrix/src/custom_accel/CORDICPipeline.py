from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class CORDICPipeline(ClockedObject):
    """
    NOVA Processor - Pipelined CORDIC Accelerator

    A fully pipelined CORDIC (COordinate Rotation DIgital Computer) unit
    supporting circular (sin/cos/tan/asin/acos/atan/atan2), hyperbolic
    (exp/log/sinh/cosh/tanh), magnitude (hypot), and sqrt operations.

    Pipeline architecture:
      Stage 0:       Pre-processing (range reduction)
      Stages 1..N:   CORDIC micro-rotation iterations
      Stage N+1:     Post-processing (gain compensation)

    Throughput: 1 operation per cycle (fully pipelined)
    Latency:    N+2 cycles (configurable N, default 32)
    """

    type = 'CORDICPipeline'
    cxx_header = "custom_accel/cordic_pipeline.hh"
    cxx_class = 'gem5::CORDICPipeline'

    # MMIO port for CPU access
    mmio_port = ResponsePort("MMIO port for CPU access")

    # Number of cores sharing this CORDIC pipeline
    num_cores = Param.Int(4, "Number of cores sharing this pipeline")

    # CORDIC pipeline depth (number of micro-rotation iterations)
    # 16 iterations → ~4.8 decimal digits (float32 adequate)
    # 24 iterations → ~7.2 decimal digits (float32 full precision)
    # 32 iterations → ~9.6 decimal digits (float64 good)
    # 48 iterations → ~14.4 decimal digits (float64 full precision)
    # 64 iterations → ~19.2 decimal digits (beyond double)
    num_iterations = Param.Int(32,
        "Number of CORDIC iterations (pipeline stages). "
        "More iterations = higher precision but longer latency. "
        "32 is sufficient for double-precision (64-bit) accuracy.")

    # MMIO address space
    mmio_base = Param.Addr(0x50020000, "MMIO base address")
    mmio_size = Param.Addr(0x1000, "MMIO address space size")

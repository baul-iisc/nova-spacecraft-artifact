/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * Enhanced TinyML Accelerator with 32 Processing Elements
 * Designed for on-board AI/ML inference in spacecraft systems
 * 
 * Features:
 * - 32 Processing Elements (PEs) arranged in 4x8 array
 * - Supports INT8, INT16, and FP16 quantized inference
 * - Built-in weight stationary dataflow
 * - Support for convolution, fully-connected, pooling, and activation
 * - On-chip SRAM for weights and activations
 * - DMA engine for efficient data movement
 */

#ifndef __CUSTOM_ACCEL_TINY_ML_32PE_HH__
#define __CUSTOM_ACCEL_TINY_ML_32PE_HH__

#include <array>
#include <queue>
#include <vector>
#include <cstdint>
#include <cmath>

#include "mem/port.hh"
#include "params/TinyML32PE.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * TinyML32PE - Enhanced ML accelerator with 32 Processing Elements
 * 
 * Designed for spacecraft on-board AI applications:
 * - Autonomous navigation and GNC (Guidance, Navigation, Control)
 * - Anomaly detection in telemetry
 * - Image classification for Earth observation
 * - Predictive maintenance
 * - Autonomous rendezvous and docking
 * 
 * Processing Element (PE) Array Configuration:
 * - 4 rows x 8 columns = 32 PEs
 * - Each PE contains a MAC unit supporting INT8/INT16/FP16
 * - Weight-stationary dataflow for energy efficiency
 * 
 * Memory Hierarchy:
 * - Global Buffer: Configurable SRAM (default 128KB)
 * - Weight Buffer: Dedicated weight storage (default 64KB)
 * - Accumulator Buffer: For partial sums (default 32KB)
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x000: CTRL          - Control register
 * 0x004: STATUS        - Status register
 * 0x008: INPUT_BASE    - Input tensor base address
 * 0x00C: WEIGHT_BASE   - Weight tensor base address
 * 0x010: OUTPUT_BASE   - Output tensor base address
 * 0x014: BIAS_BASE     - Bias tensor base address
 * 0x018: INPUT_H       - Input height
 * 0x01C: INPUT_W       - Input width
 * 0x020: INPUT_C       - Input channels
 * 0x024: OUTPUT_C      - Output channels
 * 0x028: KERNEL_H      - Kernel height
 * 0x02C: KERNEL_W      - Kernel width
 * 0x030: STRIDE        - Stride (H<<8 | W)
 * 0x034: PADDING       - Padding (top<<24 | bottom<<16 | left<<8 | right)
 * 0x038: DATA_TYPE     - Data type (0=INT8, 1=INT16, 2=FP16)
 * 0x03C: OP_TYPE       - Operation type
 * 0x040: ACTIVATION    - Activation function
 * 0x044: QUANT_SCALE   - Quantization scale
 * 0x048: QUANT_ZERO    - Quantization zero point
 * 0x04C: PE_CONFIG     - PE array configuration
 * 0x050: DMA_STATUS    - DMA engine status
 * 0x054: PERF_CYCLES   - Performance counter (cycles)
 * 0x058: PERF_MACS     - Performance counter (MAC operations)
 * 0x05C: PERF_BYTES    - Performance counter (bytes transferred)
 * 0x060: INTERRUPT     - Interrupt status/control
 * 0x064: VERSION       - Accelerator version
 */
class TinyML32PE : public ClockedObject
{
  private:
    /**
     * CPU-side port for MMIO access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        TinyML32PE *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, TinyML32PE *owner);
        void sendPacket(PacketPtr pkt);
        AddrRangeList getAddrRanges() const override;
        void trySendRetry();

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    /**
     * Memory-side port for DMA
     */
    class MemSidePort : public RequestPort
    {
      private:
        TinyML32PE *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, TinyML32PE *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Data types supported
    enum class DataType : uint8_t {
        INT8 = 0,
        INT16 = 1,
        FP16 = 2,
        FP32 = 3   // For accumulation
    };

    // Operation types
    enum class OpType : uint8_t {
        CONV2D = 0,              // 2D Convolution
        DEPTHWISE_CONV = 1,      // Depthwise separable convolution
        POINTWISE_CONV = 2,      // 1x1 Convolution
        FULLY_CONNECTED = 3,     // Matrix multiplication
        MAX_POOL = 4,            // Max pooling
        AVG_POOL = 5,            // Average pooling
        GLOBAL_AVG_POOL = 6,     // Global average pooling
        ELEMENTWISE_ADD = 7,     // Element-wise addition
        ELEMENTWISE_MUL = 8,     // Element-wise multiplication
        SOFTMAX = 9,             // Softmax (approximated)
        BATCH_NORM = 10          // Batch normalization
    };

    // Activation types
    enum class ActivationType : uint8_t {
        NONE = 0,
        RELU = 1,
        RELU6 = 2,
        LEAKY_RELU = 3,
        SIGMOID = 4,
        TANH = 5,
        SWISH = 6,
        GELU = 7
    };

    // Processing Element state
    struct ProcessingElement {
        int32_t accumulator;
        int8_t weight;
        bool busy;
        uint32_t macCount;
    };

    // Register offsets
    static const Addr REG_CTRL          = 0x000;
    static const Addr REG_STATUS        = 0x004;
    static const Addr REG_INPUT_BASE    = 0x008;
    static const Addr REG_WEIGHT_BASE   = 0x00C;
    static const Addr REG_OUTPUT_BASE   = 0x010;
    static const Addr REG_BIAS_BASE     = 0x014;
    static const Addr REG_INPUT_H       = 0x018;
    static const Addr REG_INPUT_W       = 0x01C;
    static const Addr REG_INPUT_C       = 0x020;
    static const Addr REG_OUTPUT_C      = 0x024;
    static const Addr REG_KERNEL_H      = 0x028;
    static const Addr REG_KERNEL_W      = 0x02C;
    static const Addr REG_STRIDE        = 0x030;
    static const Addr REG_PADDING       = 0x034;
    static const Addr REG_DATA_TYPE     = 0x038;
    static const Addr REG_OP_TYPE       = 0x03C;
    static const Addr REG_ACTIVATION    = 0x040;
    static const Addr REG_QUANT_SCALE   = 0x044;
    static const Addr REG_QUANT_ZERO    = 0x048;
    static const Addr REG_PE_CONFIG     = 0x04C;
    static const Addr REG_DMA_STATUS    = 0x050;
    static const Addr REG_PERF_CYCLES   = 0x054;
    static const Addr REG_PERF_MACS     = 0x058;
    static const Addr REG_PERF_BYTES    = 0x05C;
    static const Addr REG_INTERRUPT     = 0x060;
    static const Addr REG_VERSION       = 0x064;

    // Control bits
    static const uint32_t CTRL_START    = 0x01;
    static const uint32_t CTRL_ABORT    = 0x02;
    static const uint32_t CTRL_RESET    = 0x80;
    static const uint32_t CTRL_IRQ_EN   = 0x100;

    // Status bits
    static const uint32_t STATUS_IDLE       = 0x00;
    static const uint32_t STATUS_BUSY       = 0x01;
    static const uint32_t STATUS_DONE       = 0x02;
    static const uint32_t STATUS_ERROR      = 0x04;
    static const uint32_t STATUS_DMA_BUSY   = 0x08;

    // Hardware configuration
    static constexpr unsigned NUM_PE_ROWS = 4;
    static constexpr unsigned NUM_PE_COLS = 8;
    static constexpr unsigned NUM_PES = NUM_PE_ROWS * NUM_PE_COLS;  // 32 PEs

    // Internal registers
    uint32_t regCtrl;
    uint32_t regStatus;
    Addr regInputBase;
    Addr regWeightBase;
    Addr regOutputBase;
    Addr regBiasBase;
    uint32_t regInputH, regInputW, regInputC;
    uint32_t regOutputC;
    uint32_t regKernelH, regKernelW;
    uint32_t regStride, regPadding;
    uint32_t regDataType;
    uint32_t regOpType;
    uint32_t regActivation;
    uint32_t regQuantScale, regQuantZero;
    uint32_t regPeConfig;
    uint32_t regDmaStatus;
    uint64_t regPerfCycles;
    uint64_t regPerfMacs;
    uint64_t regPerfBytes;
    uint32_t regInterrupt;

    // Hardware parameters (configurable)
    unsigned globalBufferSize;      // Global buffer size in bytes
    unsigned weightBufferSize;      // Weight buffer size in bytes
    unsigned accumBufferSize;       // Accumulator buffer size in bytes
    unsigned peLatency;             // Cycles per PE MAC operation
    unsigned dmaLatency;            // Cycles per DMA transfer
    unsigned dataWidth;             // Data bus width in bytes

    // Processing Element array
    std::array<ProcessingElement, NUM_PES> peArray;

    // On-chip buffers (simulating SRAM)
    std::vector<int8_t> globalBuffer;
    std::vector<int8_t> weightBuffer;
    std::vector<int32_t> accumBuffer;

    // DMA state
    bool dmaActive;
    unsigned dmaBytesRemaining;
    Addr dmaCurrentAddr;

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // Statistics
    struct TinyML32PEStats : public statistics::Group
    {
        TinyML32PEStats(TinyML32PE *parent);
        statistics::Scalar convOps;
        statistics::Scalar fcOps;
        statistics::Scalar poolOps;
        statistics::Scalar totalMacOps;
        statistics::Scalar totalCycles;
        statistics::Scalar inputBytesLoaded;
        statistics::Scalar weightBytesLoaded;
        statistics::Scalar outputBytesStored;
        statistics::Formula throughput;       // MACs per cycle
        statistics::Formula utilizationRate;  // PE utilization
    } stats;

    // Events
    EventFunctionWrapper computeEvent;
    EventFunctionWrapper dmaEvent;

    // Internal methods
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);

    // Computation methods
    void startOperation();
    void executeConv2D();
    void executeDepthwiseConv();
    void executeFullyConnected();
    void executePooling(bool isMax);
    void executeGlobalAvgPool();
    void executeElementwise(bool isAdd);
    void executeSoftmax();
    void executeBatchNorm();
    void completeOperation();

    // PE array operations
    void resetPEArray();
    void loadWeightsToPEs(unsigned startIdx, unsigned count);
    void computePETile(const std::vector<int8_t>& inputs, 
                       unsigned inputOffset, unsigned outputOffset);

    // Activation functions (INT8 approximations)
    int8_t applyActivation(int32_t accumulator);
    int8_t relu(int32_t x);
    int8_t relu6(int32_t x);
    int8_t leakyRelu(int32_t x);
    int8_t sigmoidApprox(int32_t x);
    int8_t tanhApprox(int32_t x);
    int8_t swishApprox(int32_t x);

    // Quantization helpers
    int8_t quantize(int32_t accumulator);
    int32_t dequantize(int8_t value);

    // DMA operations
    void startDmaRead(Addr addr, unsigned size);
    void startDmaWrite(Addr addr, unsigned size);
    void completeDma();

  public:
    PARAMS(TinyML32PE);
    TinyML32PE(const Params &params);

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();

    // Version information
    static const uint32_t VERSION = 0x01000000;  // v1.0.0.0
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_TINY_ML_32PE_HH__


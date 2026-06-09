/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * Tiny ML Accelerator for On-board AI/ML Processing
 * Supports INT8 quantized neural network inference
 */

#ifndef __CUSTOM_ACCEL_TINY_ML_ACCEL_HH__
#define __CUSTOM_ACCEL_TINY_ML_ACCEL_HH__

#include <queue>
#include <vector>
#include <cstdint>

#include "mem/port.hh"
#include "params/TinyMLAccel.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * TinyMLAccel - A lightweight ML accelerator for spacecraft applications
 * 
 * Designed for on-board AI tasks:
 * - Anomaly detection in telemetry
 * - Image classification (cloud detection, feature extraction)
 * - Autonomous navigation assistance
 * - Predictive maintenance
 * 
 * Features:
 * - INT8 quantized inference for power efficiency
 * - Configurable MAC array size
 * - Built-in activation functions (ReLU, Sigmoid approx)
 * - Efficient weight loading mechanism
 * 
 * Register Map (Memory-Mapped Interface):
 * 0x00: CTRL        - Control register
 * 0x04: STATUS      - Status register
 * 0x08: INPUT_ADDR  - Input tensor address
 * 0x0C: WEIGHT_ADDR - Weight tensor address
 * 0x10: OUTPUT_ADDR - Output tensor address
 * 0x14: INPUT_DIM   - Input dimensions (H<<16 | W<<8 | C)
 * 0x18: WEIGHT_DIM  - Weight dimensions (output_ch<<16 | kernel_size)
 * 0x1C: CONFIG      - Configuration (activation type, stride, padding)
 * 0x20: MAC_CONFIG  - MAC array configuration
 * 0x24: PERF_CNT    - Performance counter (MAC operations)
 */
class TinyMLAccel : public ClockedObject
{
  private:
    /**
     * CPU-side port for MMIO access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        TinyMLAccel *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, TinyMLAccel *owner);
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
        TinyMLAccel *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, TinyMLAccel *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // Operation types
    enum class OpType {
        MATMUL,          // Matrix multiplication
        CONV2D,          // 2D Convolution
        DEPTHWISE_CONV,  // Depthwise separable convolution
        POOLING,         // Max/Average pooling
        ACTIVATION,      // Activation only
        ELEMENTWISE      // Element-wise operations
    };

    // Activation types
    enum class ActivationType {
        NONE = 0,
        RELU = 1,
        RELU6 = 2,
        SIGMOID = 3,
        TANH = 4
    };

    // Register offsets
    static const Addr REG_CTRL        = 0x00;
    static const Addr REG_STATUS      = 0x04;
    static const Addr REG_INPUT_ADDR  = 0x08;
    static const Addr REG_WEIGHT_ADDR = 0x0C;
    static const Addr REG_OUTPUT_ADDR = 0x10;
    static const Addr REG_INPUT_DIM   = 0x14;
    static const Addr REG_WEIGHT_DIM  = 0x18;
    static const Addr REG_CONFIG      = 0x1C;
    static const Addr REG_MAC_CONFIG  = 0x20;
    static const Addr REG_PERF_CNT    = 0x24;
    static const Addr REG_BIAS_ADDR   = 0x28;
    static const Addr REG_SCALE       = 0x2C;

    // Control bits
    static const uint32_t CTRL_START    = 0x01;
    static const uint32_t CTRL_OP_MASK  = 0x70;  // Operation type
    static const uint32_t CTRL_OP_SHIFT = 4;
    static const uint32_t CTRL_RESET    = 0x80;

    // Status bits
    static const uint32_t STATUS_BUSY   = 0x01;
    static const uint32_t STATUS_DONE   = 0x02;
    static const uint32_t STATUS_ERROR  = 0x04;

    // Internal registers
    uint32_t regCtrl;
    uint32_t regStatus;
    Addr regInputAddr;
    Addr regWeightAddr;
    Addr regOutputAddr;
    uint32_t regInputDim;
    uint32_t regWeightDim;
    uint32_t regConfig;
    uint32_t regMacConfig;
    uint32_t regPerfCnt;
    Addr regBiasAddr;
    uint32_t regScale;

    // Hardware parameters
    unsigned macArrayRows;      // Number of MAC rows
    unsigned macArrayCols;      // Number of MAC columns
    unsigned sramSize;          // On-chip SRAM size (bytes)
    unsigned macLatency;        // Cycles per MAC operation
    unsigned dmaLatency;        // Cycles per DMA transfer

    // Internal state
    bool busy;
    OpType currentOp;
    ActivationType activation;
    uint64_t totalMacOps;

    // Buffers (simulating on-chip SRAM)
    std::vector<int8_t> inputBuffer;
    std::vector<int8_t> weightBuffer;
    std::vector<int32_t> accumBuffer;
    std::vector<int8_t> outputBuffer;

    // Ports
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // Statistics
    struct MLStats : public statistics::Group
    {
        MLStats(TinyMLAccel *parent);
        statistics::Scalar matmulOps;
        statistics::Scalar conv2dOps;
        statistics::Scalar totalMacOperations;
        statistics::Scalar totalCycles;
        statistics::Scalar inputBytesLoaded;
        statistics::Scalar weightBytesLoaded;
        statistics::Scalar outputBytesStored;
    } stats;

    // Completion event
    EventFunctionWrapper computeEvent;

    // Internal methods
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);
    
    void startOperation();
    void executeMatMul();
    void executeConv2D();
    void executePooling();
    void applyActivation();
    void completeOperation();

    // Activation functions (INT8 approximations)
    int8_t relu(int32_t x);
    int8_t relu6(int32_t x);
    int8_t sigmoid_approx(int32_t x);
    int8_t tanh_approx(int32_t x);

    // Quantization helpers
    int8_t quantize(int32_t accumulator, int32_t scale, int32_t zero_point);

  public:
    PARAMS(TinyMLAccel);
    TinyMLAccel(const Params &params);

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;
    void sendRangeChange();
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_TINY_ML_ACCEL_HH__


/*
 * Copyright (c) 2024 Chandraboul - PhD Research
 * Spacecraft Heterogeneous Multicore Processor
 * 
 * TinyTPU - A Simplified TPU-like Matrix Accelerator
 * 
 * Inspired by Google's Tensor Processing Unit (TPU), this implements:
 * - Systolic Array Matrix Multiply Unit (MXU)
 * - Weight Stationary Dataflow
 * - Unified Buffer for Activations
 * - Accumulator Buffer for Partial Sums
 * - Activation Unit (ReLU, Sigmoid, etc.)
 * 
 * Architecture:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                            TinyTPU                                       │
 * │  ┌─────────────────────────────────────────────────────────────────────┐ │
 * │  │                    Host Interface (MMIO)                            │ │
 * │  └─────────────────────────────────────────────────────────────────────┘ │
 * │           │                    │                      │                  │
 * │           ▼                    ▼                      ▼                  │
 * │  ┌─────────────┐      ┌─────────────┐        ┌─────────────┐            │
 * │  │   Weight    │      │  Unified    │        │ Instruction │            │
 * │  │   FIFO      │      │  Buffer     │        │   Buffer    │            │
 * │  │  (Weights)  │      │(Activations)│        │  (Commands) │            │
 * │  └──────┬──────┘      └──────┬──────┘        └──────┬──────┘            │
 * │         │                    │                      │                    │
 * │         ▼                    ▼                      ▼                    │
 * │  ┌─────────────────────────────────────────────────────────────────────┐ │
 * │  │              Matrix Multiply Unit (MXU) - 8x8 Systolic Array        │ │
 * │  │  ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐                   │ │
 * │  │  │PE │→│PE │→│PE │→│PE │→│PE │→│PE │→│PE │→│PE │→ Row 0            │ │
 * │  │  └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘                   │ │
 * │  │    ↓     ↓     ↓     ↓     ↓     ↓     ↓     ↓                     │ │
 * │  │  ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐                   │ │
 * │  │  │PE │→│PE │→│PE │→│PE │→│PE │→│PE │→│PE │→│PE │→ Row 1            │ │
 * │  │  └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘                   │ │
 * │  │    ↓     ↓     ↓     ↓     ↓     ↓     ↓     ↓                     │ │
 * │  │   ...   ...   ...   ...   ...   ...   ...   ...                    │ │
 * │  │    ↓     ↓     ↓     ↓     ↓     ↓     ↓     ↓                     │ │
 * │  │  ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐ ┌───┐                   │ │
 * │  │  │PE │→│PE │→│PE │→│PE │→│PE │→│PE │→│PE │→│PE │→ Row 7            │ │
 * │  │  └───┘ └───┘ └───┘ └───┘ └───┘ └───┘ └───┘ └───┘                   │ │
 * │  └─────────────────────────────────────────────────────────────────────┘ │
 * │                                    │                                      │
 * │                                    ▼                                      │
 * │  ┌─────────────────────────────────────────────────────────────────────┐ │
 * │  │                      Accumulator Buffer (32-bit)                    │ │
 * │  └─────────────────────────────────────────────────────────────────────┘ │
 * │                                    │                                      │
 * │                                    ▼                                      │
 * │  ┌─────────────────────────────────────────────────────────────────────┐ │
 * │  │                      Activation Unit                                │ │
 * │  │            (ReLU, ReLU6, Sigmoid, Tanh, Linear)                     │ │
 * │  └─────────────────────────────────────────────────────────────────────┘ │
 * │                                    │                                      │
 * │                                    ▼                                      │
 * │  ┌─────────────────────────────────────────────────────────────────────┐ │
 * │  │                      Normalize/Quantize Unit                        │ │
 * │  │                   (Scale, Bias, Quantize to INT8)                   │ │
 * │  └─────────────────────────────────────────────────────────────────────┘ │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 * Register Map (MMIO):
 * 0x00: CTRL           - Control register (start, stop, reset)
 * 0x04: STATUS         - Status register (busy, done, error)
 * 0x08: INSTR_ADDR     - Instruction buffer base address
 * 0x0C: WEIGHT_ADDR    - Weight data base address
 * 0x10: INPUT_ADDR     - Input activation base address
 * 0x14: OUTPUT_ADDR    - Output activation base address
 * 0x18: MATRIX_DIM_M   - M dimension (output rows)
 * 0x1C: MATRIX_DIM_N   - N dimension (output cols)
 * 0x20: MATRIX_DIM_K   - K dimension (reduction)
 * 0x24: ACTIVATION     - Activation function type
 * 0x28: QUANT_SCALE    - Quantization scale
 * 0x2C: QUANT_ZERO     - Quantization zero point
 * 0x30: PERF_CYCLES    - Performance: total cycles
 * 0x34: PERF_MACS      - Performance: MAC operations
 * 0x38: PERF_TILES     - Performance: tiles processed
 * 0x3C: MXU_CONFIG     - MXU configuration (array size, precision)
 */

#ifndef __CUSTOM_ACCEL_TINY_TPU_HH__
#define __CUSTOM_ACCEL_TINY_TPU_HH__

#include <queue>
#include <vector>
#include <array>
#include <cstdint>
#include <functional>

#include "mem/port.hh"
#include "params/TinyTPU.hh"
#include "sim/clocked_object.hh"
#include "sim/sim_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

/**
 * TinyTPU - TPU-inspired Matrix Multiply Accelerator
 */
class TinyTPU : public ClockedObject
{
  public:
    PARAMS(TinyTPU);
    TinyTPU(const Params &params);
    ~TinyTPU();

    void startup() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    AddrRangeList getAddrRanges() const;

  private:
    /**
     * CPU-side port for MMIO register access
     */
    class CPUSidePort : public ResponsePort
    {
      private:
        TinyTPU *owner;
        bool needRetry;
        PacketPtr blockedPacket;

      public:
        CPUSidePort(const std::string& name, TinyTPU *owner);
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
     * Memory-side port for DMA transfers
     */
    class MemSidePort : public RequestPort
    {
      private:
        TinyTPU *owner;
        PacketPtr blockedPacket;

      public:
        MemSidePort(const std::string& name, TinyTPU *owner);
        void sendPacket(PacketPtr pkt);

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    // =========================================================================
    // TPU Configuration
    // =========================================================================
    
    // MXU (Matrix Multiply Unit) dimensions
    static constexpr int MXU_SIZE = 8;        // 8x8 systolic array
    static constexpr int TILE_SIZE = 8;       // Tile size matches MXU
    
    // Activation function types
    enum class ActivationType : uint32_t {
        NONE = 0,
        RELU = 1,
        RELU6 = 2,
        SIGMOID = 3,
        TANH = 4,
        LEAKY_RELU = 5
    };

    // Data types
    enum class DataType : uint32_t {
        INT8 = 0,
        INT16 = 1,
        INT32 = 2,
        FP16 = 3,
        FP32 = 4,
        BF16 = 5
    };

    // TPU Instructions
    enum class TPUOpcode : uint32_t {
        NOP = 0,
        LOAD_WEIGHTS = 1,      // Load weights to weight FIFO
        LOAD_INPUT = 2,        // Load activations to unified buffer
        MATMUL = 3,            // Matrix multiply
        STORE_OUTPUT = 4,      // Store results
        ACTIVATE = 5,          // Apply activation function
        SYNC = 6               // Synchronization barrier
    };

    // Processing Element (PE) for systolic array
    struct ProcessingElement {
        int32_t weight;        // Stationary weight
        int32_t accumulator;   // Running sum
        bool weight_valid;
        
        void reset() {
            weight = 0;
            accumulator = 0;
            weight_valid = false;
        }
        
        // MAC operation: acc += input * weight
        void compute(int32_t input) {
            if (weight_valid) {
                accumulator += input * weight;
            }
        }
    };

    // =========================================================================
    // Register Definitions
    // =========================================================================
    
    static const Addr REG_CTRL         = 0x00;
    static const Addr REG_STATUS       = 0x04;
    static const Addr REG_INSTR_ADDR   = 0x08;
    static const Addr REG_WEIGHT_ADDR  = 0x0C;
    static const Addr REG_INPUT_ADDR   = 0x10;
    static const Addr REG_OUTPUT_ADDR  = 0x14;
    static const Addr REG_DIM_M        = 0x18;
    static const Addr REG_DIM_N        = 0x1C;
    static const Addr REG_DIM_K        = 0x20;
    static const Addr REG_ACTIVATION   = 0x24;
    static const Addr REG_QUANT_SCALE  = 0x28;
    static const Addr REG_QUANT_ZERO   = 0x2C;
    static const Addr REG_PERF_CYCLES  = 0x30;
    static const Addr REG_PERF_MACS    = 0x34;
    static const Addr REG_PERF_TILES   = 0x38;
    static const Addr REG_MXU_CONFIG   = 0x3C;
    static const Addr REG_BIAS_ADDR    = 0x40;
    static const Addr REG_DATA_TYPE    = 0x44;

    // Control bits
    static const uint32_t CTRL_START   = 0x01;
    static const uint32_t CTRL_STOP    = 0x02;
    static const uint32_t CTRL_RESET   = 0x04;

    // Status bits
    static const uint32_t STATUS_IDLE  = 0x00;
    static const uint32_t STATUS_BUSY  = 0x01;
    static const uint32_t STATUS_DONE  = 0x02;
    static const uint32_t STATUS_ERROR = 0x04;

    // =========================================================================
    // Hardware Configuration (from params)
    // =========================================================================
    
    unsigned unifiedBufferSize;     // Unified buffer size (bytes)
    unsigned weightFifoSize;        // Weight FIFO size (bytes)
    unsigned accumBufferSize;       // Accumulator buffer size (bytes)
    unsigned mxuLatency;            // Cycles per MXU operation
    unsigned dmaLatency;            // Cycles per DMA transfer

    // =========================================================================
    // Internal Registers
    // =========================================================================
    
    uint32_t regCtrl;
    uint32_t regStatus;
    Addr regInstrAddr;
    Addr regWeightAddr;
    Addr regInputAddr;
    Addr regOutputAddr;
    uint32_t regDimM;
    uint32_t regDimN;
    uint32_t regDimK;
    uint32_t regActivation;
    uint32_t regQuantScale;
    uint32_t regQuantZero;
    uint64_t regPerfCycles;
    uint64_t regPerfMacs;
    uint64_t regPerfTiles;
    uint32_t regMxuConfig;
    Addr regBiasAddr;
    uint32_t regDataType;

    // =========================================================================
    // On-chip Buffers
    // =========================================================================
    
    // Unified buffer (activations/inputs)
    std::vector<int8_t> unifiedBuffer;
    
    // Weight FIFO (weights loaded row by row)
    std::vector<int8_t> weightFifo;
    
    // Accumulator buffer (32-bit accumulators)
    std::vector<int32_t> accumBuffer;
    
    // Output buffer (after activation and quantization)
    std::vector<int8_t> outputBuffer;
    
    // 8x8 Systolic Array (MXU)
    std::array<std::array<ProcessingElement, MXU_SIZE>, MXU_SIZE> mxu;

    // =========================================================================
    // Execution State
    // =========================================================================
    
    bool busy;
    bool aborted;
    ActivationType activationType;
    DataType dataType;
    
    // Tiling state
    int numTilesM;
    int numTilesN;
    int numTilesK;
    int currentTileM;
    int currentTileN;
    int currentTileK;

    // =========================================================================
    // Ports
    // =========================================================================
    
    CPUSidePort cpuPort;
    MemSidePort memPort;
    AddrRange addrRange;

    // =========================================================================
    // Events
    // =========================================================================
    
    EventFunctionWrapper mxuComputeEvent;
    EventFunctionWrapper dmaCompleteEvent;
    EventFunctionWrapper operationCompleteEvent;

    // =========================================================================
    // Statistics
    // =========================================================================
    
    struct TPUStats : public statistics::Group
    {
        TPUStats(TinyTPU *parent);
        statistics::Scalar totalOperations;
        statistics::Scalar totalTilesProcessed;
        statistics::Scalar totalMacOperations;
        statistics::Scalar totalCycles;
        statistics::Scalar weightBytesLoaded;
        statistics::Scalar inputBytesLoaded;
        statistics::Scalar outputBytesStored;
        statistics::Scalar activationsComputed;
    } stats;

    // =========================================================================
    // Internal Methods
    // =========================================================================
    
    // Port handling
    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    
    // Register access
    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);

    // Operation control
    void startOperation();
    void stopOperation();
    void resetTPU();
    void completeOperation();

    // Systolic array operations
    void initializeMXU();
    void loadWeightsToMXU(int tileK, int tileN);
    void streamInputsThroughMXU(int tileM, int tileK);
    void drainAccumulators(int tileM, int tileN);
    void executeTileMultiply();
    
    // Activation functions
    int8_t applyActivation(int32_t value);
    int8_t relu(int32_t x);
    int8_t relu6(int32_t x);
    int8_t sigmoid_approx(int32_t x);
    int8_t tanh_approx(int32_t x);
    int8_t leaky_relu(int32_t x);

    // Quantization
    int8_t quantize(int32_t accumulator);

    // Tiling
    void initializeTiling();
    void scheduleNextTile();
    bool allTilesComplete();

    // DMA
    void loadWeightTile(int tileK, int tileN);
    void loadInputTile(int tileM, int tileK);
    void storeOutputTile(int tileM, int tileN);
    void handleDmaComplete();
};

} // namespace gem5

#endif // __CUSTOM_ACCEL_TINY_TPU_HH__


/*
 * NOVA Processor - Neural Processing Unit (NPU) / TinyML Accelerator
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * Features:
 * - Systolic array architecture (configurable NxN)
 * - INT8/INT4 quantized operations
 * - Sparsity support (zero-skipping)
 * - Layer types: Conv, FC, Pooling, Activation
 * - Weight cache and activation scratchpad
 * - DMA engine for efficient data movement
 */

#ifndef __SPACECRAFT_NPU_HH__
#define __SPACECRAFT_NPU_HH__

#include "mem/port.hh"
#include "params/NeuralProcessingUnit.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <queue>
#include <vector>
#include <cstdint>
#include <functional>
#include <string>

namespace gem5
{

namespace spacecraft
{

/**
 * NPU Layer Types
 */
enum class NPULayerType {
    CONV_STANDARD,      // Standard convolution
    CONV_DEPTHWISE,     // Depthwise separable convolution
    CONV_POINTWISE,     // 1x1 pointwise convolution
    FULLY_CONNECTED,    // Dense/FC layer
    POOLING_MAX,        // Max pooling
    POOLING_AVG,        // Average pooling
    ACTIVATION_RELU,    // ReLU activation
    ACTIVATION_RELU6,   // ReLU6 (capped at 6)
    ACTIVATION_HARDSWISH, // HardSwish activation
    ACTIVATION_SIGMOID, // Sigmoid
    SOFTMAX,            // Softmax (output layer)
    BATCH_NORM,         // Batch normalization
    ADD,                // Element-wise add (residual)
    CONCAT,             // Concatenation
    NOP
};

/**
 * NPU Data Precision
 */
enum class NPUPrecision {
    INT4,   // 4-bit quantized
    INT8,   // 8-bit quantized
    INT16,  // 16-bit quantized
    FP16,   // Half-precision float
    FP32    // Single-precision float
};

/**
 * Layer parameters structure
 */
struct LayerParams {
    NPULayerType type;
    NPUPrecision precision;
    
    // Input dimensions
    uint32_t inputH, inputW, inputC;
    
    // Output dimensions
    uint32_t outputH, outputW, outputC;
    
    // Kernel parameters
    uint32_t kernelH, kernelW;
    uint32_t strideH, strideW;
    uint32_t padH, padW;
    
    // Memory addresses
    Addr inputAddr;
    Addr outputAddr;
    Addr weightsAddr;
    Addr biasAddr;
    
    // Sparsity info (percentage of zeros)
    float sparsity;
};

/**
 * NPU Inference Request
 */
struct NPURequest {
    uint64_t requestId;
    int coreId;
    
    // Model info
    std::string modelName;
    uint32_t numLayers;
    std::vector<LayerParams> layers;
    
    // Input/output
    Addr inputAddr;
    Addr outputAddr;
    
    // Timing
    Tick requestTime;
    Tick deadline;
    int priority;
    
    // Callback
    std::function<void(bool, const std::vector<float>&)> callback;
};

/**
 * NPU Performance Counters
 */
struct NPUPerfCounters {
    uint64_t totalMACs;
    uint64_t totalCycles;
    uint64_t memoryStalls;
    uint64_t computeCycles;
    uint64_t activationCycles;
    uint64_t dmaCycles;
    float utilization;
    float sparsitySkipped;
};

/**
 * NPU Statistics
 */
struct NPUStats {
    uint64_t totalInferences;
    uint64_t completedInferences;
    uint64_t totalLayers;
    uint64_t convLayers;
    uint64_t fcLayers;
    uint64_t poolingLayers;
    uint64_t activationLayers;
    uint64_t totalMACs;
    uint64_t busyCycles;
    uint64_t idleCycles;
    double avgInferenceLatency;
    double avgUtilization;
    uint64_t maxQueueDepth;
    uint64_t weightCacheHits;
    uint64_t weightCacheMisses;
};

/**
 * Neural Processing Unit - Hardware accelerator for TinyML inference
 */
class NeuralProcessingUnit : public ClockedObject
{
  public:
    PARAMS(NeuralProcessingUnit);
    NeuralProcessingUnit(const Params &p);
    ~NeuralProcessingUnit();
    
    // Port interface
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    
    // Submit inference request
    bool submitInference(NPURequest &req);
    
    // Submit single layer execution
    bool executeLayer(const LayerParams &layer, int coreId);
    
    // Check status
    bool isBusy() const { return currentRequest != nullptr; }
    size_t getQueueDepth() const { return requestQueue.size(); }
    
    // Get statistics
    const NPUStats& getStats() const { return stats; }
    NPUPerfCounters getLastPerfCounters() const { return lastPerfCounters; }
    
    // Reset statistics
    void resetStats();
    
    // Power management
    void powerGate();
    void powerUp();
    bool isPoweredOn() const { return poweredOn; }
    
    // DVFS support
    void setFrequencyScale(float scale);
    float getFrequencyScale() const { return frequencyScale; }
    
    // Calculate layer latency
    Cycles calculateLayerLatency(const LayerParams &layer) const;
    
  private:
    /**
     * DMA Port for memory access
     */
    class NPUPort : public RequestPort
    {
      public:
        NPUPort(const std::string &name, NeuralProcessingUnit *owner)
            : RequestPort(name), npu(owner) {}
        
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        
      private:
        NeuralProcessingUnit *npu;
    };
    
    /**
     * MMIO Response Port
     */
    class MMIOPort : public ResponsePort
    {
      public:
        MMIOPort(const std::string &name, NeuralProcessingUnit *owner)
            : ResponsePort(name), npu(owner) {}
        
        AddrRangeList getAddrRanges() const override;
        
      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvRespRetry() override;
        
      private:
        NeuralProcessingUnit *npu;
    };
    
    /**
     * Systolic Array for matrix operations
     */
    class SystolicArray {
      public:
        SystolicArray(int rows, int cols)
            : numRows(rows), numCols(cols),
              weights(rows * cols, 0),
              accumulators(cols, 0) {}
        
        // Compute matrix multiplication
        void computeMatMul(const std::vector<int8_t>& input,
                          std::vector<int32_t>& output);
        
        // Load weights into array
        void loadWeights(const std::vector<int8_t>& w);
        
        // Get dimensions
        int getRows() const { return numRows; }
        int getCols() const { return numCols; }
        
      private:
        int numRows, numCols;
        std::vector<int8_t> weights;
        std::vector<int32_t> accumulators;
    };
    
    /**
     * Activation Unit
     */
    class ActivationUnit {
      public:
        void relu(std::vector<int32_t>& data);
        void relu6(std::vector<int32_t>& data);
        void hardswish(std::vector<int32_t>& data);
        void sigmoid(std::vector<int32_t>& data);
        void softmax(std::vector<int32_t>& data, std::vector<float>& output);
    };
    
    // Ports
    NPUPort dmaPort;
    MMIOPort mmioPort;
    
    // Systolic array
    std::unique_ptr<SystolicArray> systolicArray;
    ActivationUnit activationUnit;
    
    // Request queue
    std::queue<NPURequest> requestQueue;
    NPURequest *currentRequest;
    int currentLayerIndex;
    
    // Memory hierarchy
    std::vector<int8_t> weightCache;
    std::vector<int8_t> activationScratchpad;
    size_t weightCacheSize;
    size_t activationSpadSize;
    
    // Configuration
    int arraySize;  // NxN systolic array
    bool sparseSupport;
    
    // Latency parameters
    Cycles dmaLatencyPerByte;
    Cycles activationLatency;
    Cycles poolingLatency;
    Cycles batchNormLatency;
    
    // Power state
    bool poweredOn;
    Cycles powerUpLatency;
    float frequencyScale;
    
    // Statistics
    NPUStats stats;
    NPUPerfCounters lastPerfCounters;
    
    // Events
    EventFunctionWrapper processEvent;
    EventFunctionWrapper layerCompleteEvent;
    EventFunctionWrapper inferenceCompleteEvent;
    
    // Processing functions
    void startProcessing();
    void processLayer();
    void completeLayer();
    void completeInference();
    void processNextRequest();
    
    // Layer execution (latency calculation)
    Cycles executeConvLayerLatency(const LayerParams &layer) const;
    Cycles executeFCLayerLatency(const LayerParams &layer) const;
    
    // Helper functions
    uint64_t calculateConvMACs(const LayerParams &layer) const;
    uint64_t calculateFCMACs(const LayerParams &layer) const;
    Cycles estimateDMACycles(size_t bytes) const;
    
    // MMIO handling
    Tick handleMMIORead(PacketPtr pkt);
    Tick handleMMIOWrite(PacketPtr pkt);
    
    // Instance ID and MMIO
    int instanceId;
    Addr mmioBase;
    Addr mmioSize;
};

// MMIO Register offsets
namespace NPUReg {
    const Addr CONTROL       = 0x00;
    const Addr STATUS        = 0x08;
    const Addr LAYER_TYPE    = 0x10;
    const Addr INPUT_H       = 0x18;
    const Addr INPUT_W       = 0x20;
    const Addr INPUT_C       = 0x28;
    const Addr OUTPUT_H      = 0x30;
    const Addr OUTPUT_W      = 0x38;
    const Addr OUTPUT_C      = 0x40;
    const Addr KERNEL_H      = 0x48;
    const Addr KERNEL_W      = 0x50;
    const Addr STRIDE        = 0x58;
    const Addr PADDING       = 0x60;
    const Addr INPUT_ADDR    = 0x68;
    const Addr OUTPUT_ADDR   = 0x70;
    const Addr WEIGHTS_ADDR  = 0x78;
    const Addr BIAS_ADDR     = 0x80;
    const Addr PRECISION     = 0x88;
    const Addr SPARSITY      = 0x90;
    const Addr PERF_MACS     = 0x98;
    const Addr PERF_CYCLES   = 0xA0;
    const Addr PERF_UTIL     = 0xA8;
    const Addr STATS_INFER   = 0xB0;
    const Addr STATS_LAYERS  = 0xB8;
    const Addr DVFS_SCALE    = 0xC0;
}

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_NPU_HH__


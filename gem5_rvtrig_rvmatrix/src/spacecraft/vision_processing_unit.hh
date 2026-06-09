/*
 * NOVA Processor - Vision Processing Unit (VPU)
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * Features:
 * - Image preprocessing pipeline (debayering, tone mapping, gamma correction)
 * - Resize/crop hardware
 * - Edge detection (Sobel, Canny)
 * - Feature extraction (Harris corners, ORB, optical flow)
 * - 2D convolution engine with configurable kernel sizes
 */

#ifndef __SPACECRAFT_VPU_HH__
#define __SPACECRAFT_VPU_HH__

#include "mem/port.hh"
#include "params/VisionProcessingUnit.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <queue>
#include <vector>
#include <cstdint>
#include <functional>

namespace gem5
{

namespace spacecraft
{

// Forward declarations
struct KeyPoint;
struct ImageDescriptor;

/**
 * VPU Operation Types
 */
enum class VPUOpType {
    // Preprocessing
    DEBAYER,
    TONE_MAP,
    GAMMA_CORRECT,
    RESIZE,
    CROP,
    
    // Edge Detection
    SOBEL_X,
    SOBEL_Y,
    SOBEL_MAG,
    CANNY,
    
    // Feature Extraction
    HARRIS_CORNERS,
    ORB_FEATURES,
    OPTICAL_FLOW,
    
    // Convolution
    CONV_3X3,
    CONV_5X5,
    CONV_7X7,
    DEPTHWISE_CONV,
    
    // Utility
    HISTOGRAM,
    THRESHOLD,
    NOP
};

/**
 * Keypoint structure for feature detection
 */
struct KeyPoint {
    float x, y;           // Position
    float response;       // Corner response
    float angle;          // Orientation
    float scale;          // Scale
    uint32_t descriptor[8]; // 256-bit descriptor
};

/**
 * VPU Request structure
 */
struct VPURequest {
    uint64_t requestId;
    int coreId;
    VPUOpType operation;
    
    // Image parameters
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    
    // Kernel parameters (for convolution)
    uint32_t kernelSize;
    
    // Memory addresses
    Addr inputAddr;
    Addr outputAddr;
    Addr kernelAddr;
    
    // Timing
    Tick requestTime;
    Tick deadline;
    
    // Priority (for scheduling)
    int priority;
    
    // Callback
    std::function<void(bool)> callback;
};

/**
 * VPU Statistics
 */
struct VPUStats {
    uint64_t totalRequests;
    uint64_t completedRequests;
    uint64_t debayerOps;
    uint64_t edgeDetectOps;
    uint64_t featureExtractOps;
    uint64_t convolutionOps;
    uint64_t totalCycles;
    uint64_t busyCycles;
    uint64_t queuedCycles;
    uint64_t maxQueueDepth;
    double avgLatency;
};

/**
 * Vision Processing Unit - Hardware accelerator for image processing
 */
class VisionProcessingUnit : public ClockedObject
{
  public:
    PARAMS(VisionProcessingUnit);
    VisionProcessingUnit(const Params &p);
    ~VisionProcessingUnit();
    
    // Port interface for memory access
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    
    // Submit a VPU request
    bool submitRequest(VPURequest &req);
    
    // Check if VPU is busy
    bool isBusy() const { return currentRequest != nullptr; }
    
    // Get queue depth
    size_t getQueueDepth() const { return requestQueue.size(); }
    
    // Get statistics
    const VPUStats& getStats() const { return stats; }
    
    // Reset statistics
    void resetStats();
    
    // Power management
    void powerGate();
    void powerUp();
    bool isPoweredOn() const { return poweredOn; }
    
    // Calculate operation latency (in cycles)
    Cycles calculateLatency(const VPURequest &req) const;
    
  private:
    /**
     * Memory port for DMA operations
     */
    class VPUPort : public RequestPort
    {
      public:
        VPUPort(const std::string &name, VisionProcessingUnit *owner)
            : RequestPort(name), vpu(owner) {}
        
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        
      private:
        VisionProcessingUnit *vpu;
    };
    
    // MMIO response port
    class MMIOPort : public ResponsePort
    {
      public:
        MMIOPort(const std::string &name, VisionProcessingUnit *owner)
            : ResponsePort(name), vpu(owner) {}
        
        AddrRangeList getAddrRanges() const override;
        
      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvRespRetry() override;
        
      private:
        VisionProcessingUnit *vpu;
    };
    
    // Ports
    VPUPort dmaPort;
    MMIOPort mmioPort;
    
    // Request queue
    std::queue<VPURequest> requestQueue;
    VPURequest *currentRequest;
    
    // Ping-pong image buffers (simulated)
    static const size_t BUFFER_SIZE = 640 * 480;
    std::vector<uint8_t> imageBufferA;
    std::vector<uint8_t> imageBufferB;
    int activeBuffer;
    
    // Convolution engine parameters
    int numMACUnits;
    int maxKernelSize;
    
    // Latency parameters (in cycles)
    Cycles debayerLatency;
    Cycles toneMappingLatency;
    Cycles gammaLatency;
    Cycles sobelLatency;
    Cycles cannyLatency;
    Cycles harrisLatency;
    Cycles orbLatency;
    Cycles opticalFlowLatency;
    Cycles convBaseLatency;
    
    // Scratchpad size
    size_t scratchpadSize;
    
    // Power state
    bool poweredOn;
    Cycles powerUpLatency;
    
    // Statistics
    VPUStats stats;
    
    // Events
    EventFunctionWrapper processEvent;
    EventFunctionWrapper completionEvent;
    
    // Processing functions
    void startProcessing();
    void completeProcessing();
    void processNextRequest();
    
    // Operation implementations
    Cycles performDebayer(const VPURequest &req);
    Cycles performEdgeDetection(const VPURequest &req);
    Cycles performFeatureExtraction(const VPURequest &req);
    Cycles performConvolution(const VPURequest &req);
    
    // Helper functions
    Cycles calculateConvCycles(uint32_t H, uint32_t W, 
                               uint32_t Cin, uint32_t Cout,
                               uint32_t K) const;
    
    // MMIO handling
    Tick handleMMIORead(PacketPtr pkt);
    Tick handleMMIOWrite(PacketPtr pkt);
    
    // Instance ID
    int instanceId;
    
    // MMIO base address
    Addr mmioBase;
    Addr mmioSize;
};

// MMIO Register offsets
namespace VPUReg {
    const Addr CONTROL     = 0x00;  // Control register
    const Addr STATUS      = 0x08;  // Status register
    const Addr OP_TYPE     = 0x10;  // Operation type
    const Addr IMG_WIDTH   = 0x18;  // Image width
    const Addr IMG_HEIGHT  = 0x20;  // Image height
    const Addr IMG_CHANNELS= 0x28;  // Image channels
    const Addr KERNEL_SIZE = 0x30;  // Kernel size
    const Addr INPUT_ADDR  = 0x38;  // Input buffer address
    const Addr OUTPUT_ADDR = 0x40;  // Output buffer address
    const Addr KERNEL_ADDR = 0x48;  // Kernel weights address
    const Addr RESULT      = 0x50;  // Result/keypoints count
    const Addr LATENCY     = 0x58;  // Last operation latency
    const Addr STATS_TOTAL = 0x60;  // Total operations
    const Addr STATS_BUSY  = 0x68;  // Busy cycles
}

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_VPU_HH__





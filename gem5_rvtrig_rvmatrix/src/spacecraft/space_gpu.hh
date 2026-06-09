/*
 * NOVA Processor - Space-Grade GPU (SpaceGPU)
 * PhD Research: Futuristic Spacecraft Processor
 * Author: Boul, PhD Scholar, CSA, IISc Bangalore
 *
 * Features:
 * - Terrain rendering for autonomous landing
 * - Star field rendering for optical navigation
 * - Sensor data visualization (thermal, hazard, depth maps)
 * - GPGPU compute for parallel processing (reduce, histogram)
 * - Configurable SIMD lanes (4-16)
 * - Fixed-function rasterizer
 * - Ultra-low power operation with power gating
 */

#ifndef __SPACECRAFT_SPACE_GPU_HH__
#define __SPACECRAFT_SPACE_GPU_HH__

#include "mem/port.hh"
#include "params/SpaceGPU.hh"
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

/**
 * GPU Operation Types
 */
enum class GPUOpType {
    // Rendering
    CLEAR,
    DRAW_TRIANGLES,
    DRAW_POINTS,
    DRAW_LINES,
    DRAW_TERRAIN,
    DRAW_STARS,

    // Visualization
    THERMAL_MAP,
    HAZARD_MAP,
    DEPTH_MAP,

    // Compute
    COMPUTE_DISPATCH,
    PARALLEL_REDUCE,
    HISTOGRAM,

    // Utility
    ALPHA_BLEND,
    BUFFER_SWAP,
    NOP
};

/**
 * GPU Power State
 */
enum class GPUPowerState {
    OFF,
    SLEEP,
    IDLE,
    RENDER,
    COMPUTE,
    FULL
};

/**
 * GPU Request structure
 */
struct GPURequest {
    uint64_t requestId;
    int coreId;
    GPUOpType operation;

    // Framebuffer parameters
    uint32_t width;
    uint32_t height;

    // Geometry parameters
    uint32_t vertexCount;
    uint32_t triangleCount;

    // Memory addresses
    Addr inputAddr;
    Addr outputAddr;
    Addr vertexAddr;

    // Compute parameters
    uint32_t workgroupX;
    uint32_t workgroupY;

    // Timing
    Tick requestTime;
    Tick deadline;

    // Priority
    int priority;

    // Callback
    std::function<void(bool)> callback;
};

/**
 * GPU Statistics
 */
struct GPUStats {
    uint64_t totalRequests;
    uint64_t completedRequests;
    uint64_t renderOps;
    uint64_t computeOps;
    uint64_t vizOps;
    uint64_t totalCycles;
    uint64_t busyCycles;
    uint64_t queuedCycles;
    uint64_t maxQueueDepth;
    double avgLatency;
};

/**
 * Space-Grade GPU - Hardware accelerator for spacecraft visualization & compute
 */
class SpaceGPU : public ClockedObject
{
  public:
    PARAMS(SpaceGPU);
    SpaceGPU(const Params &p);
    ~SpaceGPU();

    // Port interface for memory access
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    // Submit a GPU request
    bool submitRequest(GPURequest &req);

    // Check if GPU is busy
    bool isBusy() const { return currentRequest != nullptr; }

    // Get queue depth
    size_t getQueueDepth() const { return requestQueue.size(); }

    // Get statistics
    const GPUStats& getStats() const { return stats; }

    // Reset statistics
    void resetStats();

    // Power management
    void powerOff();
    void sleepMode();
    void powerUp();
    GPUPowerState getPowerState() const { return powerState; }

    // Calculate operation latency (in cycles)
    Cycles calculateLatency(const GPURequest &req) const;

  private:
    /**
     * DMA port for memory access
     */
    class GPUDmaPort : public RequestPort
    {
      public:
        GPUDmaPort(const std::string &name, SpaceGPU *owner)
            : RequestPort(name), gpu(owner) {}

      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;

      private:
        SpaceGPU *gpu;
    };

    /**
     * MMIO response port for register access
     */
    class GPUMMIOPort : public ResponsePort
    {
      public:
        GPUMMIOPort(const std::string &name, SpaceGPU *owner)
            : ResponsePort(name), gpu(owner) {}

        AddrRangeList getAddrRanges() const override;

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        void recvRespRetry() override;

      private:
        SpaceGPU *gpu;
    };

    // Ports
    GPUDmaPort dmaPort;
    GPUMMIOPort mmioPort;

    // Request queue
    std::queue<GPURequest> requestQueue;
    GPURequest *currentRequest;

    // Framebuffer (simulated)
    uint32_t fbWidth;
    uint32_t fbHeight;
    Addr fbBase;

    // Processing configuration
    int simdLanes;
    int maxTrianglesPerBatch;
    size_t textureCacheSize;

    // Latency parameters (in cycles)
    Cycles powerUpLatencyCycles;
    Cycles sleepWakeLatency;
    Cycles triangleSetupLatency;
    Cycles pixelLatency;
    Cycles textureLatency;
    Cycles computeBaseLatency;
    Cycles memoryLatency;

    // Power state
    GPUPowerState powerState;

    // Power consumption (mW)
    double powerOffMW;
    double powerSleepMW;
    double powerIdleMW;
    double powerRenderMW;
    double powerComputeMW;
    double powerFullMW;

    // Statistics
    GPUStats stats;

    // Events
    EventFunctionWrapper processEvent;
    EventFunctionWrapper completionEvent;

    // Processing functions
    void startProcessing();
    void completeProcessing();
    void processNextRequest();

    // MMIO handling
    Tick handleMMIORead(PacketPtr pkt);
    Tick handleMMIOWrite(PacketPtr pkt);

    // Instance ID
    int instanceId;

    // MMIO base address
    Addr mmioBase;
    Addr mmioSize;
};

// MMIO Register offsets for SpaceGPU
namespace GPUReg {
    const Addr CONTROL      = 0x000;
    const Addr STATUS       = 0x008;
    const Addr FB_WIDTH     = 0x030;
    const Addr FB_HEIGHT    = 0x038;
    const Addr CLEAR_COLOR  = 0x050;
    const Addr DRAW_CMD     = 0x200;
    const Addr VERTEX_BASE  = 0x208;
    const Addr VERTEX_CNT   = 0x210;
    const Addr COMPUTE_CMD  = 0x280;
    const Addr WORKGROUP_X  = 0x288;
    const Addr WORKGROUP_Y  = 0x290;
    const Addr STATS_TOTAL  = 0x300;
    const Addr STATS_BUSY   = 0x308;
    const Addr LATENCY      = 0x310;
}

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_SPACE_GPU_HH__

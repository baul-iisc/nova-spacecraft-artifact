/**
 * NOVA Processor - GPU-like SIMD Vision Accelerator
 * PhD Research: Futuristic Spacecraft Processor
 *
 * This header defines the NOVA SIMD unit, a GPU-like accelerator
 * optimized for parallel vision processing in spacecraft applications.
 */

#ifndef __CPU_NOVA_SIMD_NOVA_SIMD_UNIT_HH__
#define __CPU_NOVA_SIMD_NOVA_SIMD_UNIT_HH__

#include <vector>
#include <queue>
#include <cstdint>

#include "params/NOVASIMDUnit.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

namespace gem5
{

/**
 * NOVA SIMD Processing Element
 */
struct NOVAProcessingElement
{
    int peId;
    bool busy;
    int currentWarpId;
    
    std::vector<uint64_t> registers;
    
    uint64_t fp32Ops;
    uint64_t fp64Ops;
    uint64_t intOps;
    uint64_t memOps;
    
    NOVAProcessingElement(int id) : 
        peId(id), busy(false), currentWarpId(-1),
        fp32Ops(0), fp64Ops(0), intOps(0), memOps(0)
    {
        registers.resize(256 * 4, 0);
    }
};

/**
 * NOVA Warp/Wavefront
 */
struct NOVAWarp
{
    int warpId;
    int programCounter;
    bool active;
    bool waiting;
    uint32_t activeMask;
    std::vector<uint64_t> sharedRegisters;
    
    NOVAWarp(int id) :
        warpId(id), programCounter(0), active(false),
        waiting(false), activeMask(0xFFFFFFFF)
    {
        sharedRegisters.resize(64, 0);
    }
};

/**
 * Vision Operation Types
 */
enum class VisionOpType
{
    CONV_3x3,
    CONV_5x5,
    CONV_7x7,
    HARRIS_CORNER,
    FAST_CORNER,
    ORB_FEATURE,
    OPTICAL_FLOW_LK,
    SOBEL_EDGE,
    GAUSSIAN_BLUR,
    HISTOGRAM,
    THRESHOLD,
    MORPHOLOGY,
    COLOR_CONVERT
};

/**
 * Vision Task Descriptor
 */
struct VisionTask
{
    VisionOpType opType;
    uint64_t srcAddr;
    uint64_t dstAddr;
    uint32_t width;
    uint32_t height;
    uint32_t kernelSize;
    int priority;
    
    VisionTask() : 
        opType(VisionOpType::CONV_3x3),
        srcAddr(0), dstAddr(0),
        width(0), height(0), kernelSize(3),
        priority(0) {}
};

/**
 * NOVA SIMD Unit - Main GPU-like accelerator for vision processing.
 */
class NOVASIMDUnit : public SimObject
{
  private:
    const int numPEs;
    const int simdWidth;
    const int warpSize;
    const int maxWarps;
    const size_t localMemorySize;
    
    std::vector<NOVAProcessingElement> processingElements;
    std::vector<NOVAWarp> warps;
    int activeWarps;
    std::vector<uint8_t> localMemory;
    std::queue<VisionTask> taskQueue;
    
    struct NOVASIMDStats : public statistics::Group
    {
        NOVASIMDStats(NOVASIMDUnit *parent);
        
        statistics::Scalar totalTasks;
        statistics::Scalar convolutionOps;
        statistics::Scalar featureOps;
        statistics::Scalar flowOps;
        statistics::Scalar filterOps;
        statistics::Scalar totalCycles;
        statistics::Scalar idleCycles;
        statistics::Scalar stallCycles;
        statistics::Formula utilization;
        statistics::Formula avgTaskLatency;
    } stats;
    
  public:
    PARAMS(NOVASIMDUnit);
    
    NOVASIMDUnit(const Params &p);
    ~NOVASIMDUnit();
    
    bool submitTask(const VisionTask &task);
    bool isReady() const { return taskQueue.size() < 64; }
    
    int allocateWarp();
    void freeWarp(int warpId);
    
    void tick();
    void executeWarp(int warpId);
    
    void executeConvolution(const VisionTask &task);
    void executeHarrisCorner(const VisionTask &task);
    void executeOpticalFlow(const VisionTask &task);
    void executeSobelEdge(const VisionTask &task);
    
    void loadToLocalMemory(uint64_t addr, size_t size);
    void storeFromLocalMemory(uint64_t addr, size_t size);
    
    void printStats() const;
    uint64_t getTotalOps() const;
    double getUtilization() const;
};

} // namespace gem5

#endif // __CPU_NOVA_SIMD_NOVA_SIMD_UNIT_HH__

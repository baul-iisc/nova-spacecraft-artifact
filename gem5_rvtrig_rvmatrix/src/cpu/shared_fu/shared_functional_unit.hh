/*
 * NOVA Processor - Shared Functional Unit
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * This SimObject models a shared accelerator that can be used by multiple
 * CPU cores. It tracks:
 * - Busy state (which core is using it)
 * - Queue of waiting requests
 * - Latency for each operation
 * - Contention statistics
 * 
 * When a core requests a busy shared FU:
 * - The core is stalled
 * - Request is queued
 * - When FU becomes free, next request is processed
 */

#ifndef __CPU_SHARED_FU_SHARED_FUNCTIONAL_UNIT_HH__
#define __CPU_SHARED_FU_SHARED_FUNCTIONAL_UNIT_HH__

#include <queue>
#include <vector>
#include <functional>
#include <string>
#include <map>

#include "params/SharedFunctionalUnit.hh"
#include "params/SharedAcceleratorPool.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"
#include "base/statistics.hh"

namespace gem5
{

// Forward declaration
class ThreadContext;

/**
 * Shared Functional Unit - models an accelerator shared among cores
 */
class SharedFunctionalUnit : public ClockedObject
{
  public:
    PARAMS(SharedFunctionalUnit);
    
    SharedFunctionalUnit(const Params &p);
    ~SharedFunctionalUnit();
    
    /** Accelerator types */
    enum class AccelType {
        TRIG_ACCEL,     // Trigonometric (CORDIC)
        MAT_ACCEL,      // 3x3 Matrix
        VPU,            // Vision Processing
        NPU             // Neural Processing
    };
    
    /** Operation types for TrigAccel */
    enum class TrigOp {
        SIN, COS, TAN, ATAN, ATAN2, SQRT, SINCOS
    };
    
    /** Operation types for MatAccel */
    enum class MatOp {
        MUL3x3, TRANSPOSE, INVERSE, MATVEC, ADD, SCALE
    };
    
    /** Request structure */
    struct FURequest {
        int cpuId;              // Requesting CPU ID
        int threadId;           // Thread ID
        AccelType accelType;    // Accelerator type
        int opType;             // Operation type (cast to TrigOp or MatOp)
        Tick requestTime;       // When request was made
        Cycles latency;         // Operation latency
        std::function<void()> callback;  // Completion callback
    };
    
    /**
     * Request access to the shared functional unit.
     * 
     * @param cpuId The requesting CPU's ID
     * @param accelType Type of accelerator operation
     * @param opType Specific operation
     * @param callback Function to call when operation completes
     * @return true if request accepted, false if must stall
     */
    bool requestAccess(int cpuId, int threadId, AccelType accelType, 
                       int opType, std::function<void()> callback);
    
    /**
     * Check if the functional unit is currently busy.
     */
    bool isBusy() const { return busy; }
    
    /**
     * Check if there are requests waiting.
     */
    bool hasWaitingRequests() const { return !waitQueue.empty(); }
    
    /**
     * Get the current owner CPU ID (-1 if not busy).
     */
    int getCurrentOwner() const { return currentOwnerCpuId; }
    
    /**
     * Get queue depth.
     */
    size_t getQueueDepth() const { return waitQueue.size(); }
    
    /**
     * Get accelerator type.
     */
    AccelType getAccelType() const { return accelType; }
    
    /**
     * Configure latencies for different operations.
     */
    void setOpLatency(int opType, Cycles latency);
    
    /**
     * Release the functional unit (called when operation completes).
     */
    void release();
    
    /**
     * Initialize the unit.
     */
    void init() override;
    
  private:
    /** Configuration */
    AccelType accelType;
    int instanceId;
    Cycles defaultLatency;
    size_t maxQueueDepth;
    
    /** State */
    bool busy;
    int currentOwnerCpuId;
    int currentOwnerThreadId;
    FURequest currentRequest;
    
    /** Queue for waiting requests */
    std::queue<FURequest> waitQueue;
    
    /** Latency table for different operations */
    std::map<int, Cycles> opLatencies;
    
    /** Completion event */
    EventFunctionWrapper completionEvent;
    void handleCompletion();
    
    /** Process next request from queue */
    void processNextRequest();
    
    /** Get latency for an operation */
    Cycles getOpLatency(int opType) const;
    
    /** Statistics */
    struct SharedFUStats : public statistics::Group
    {
        SharedFUStats(SharedFunctionalUnit *parent);
        
        statistics::Scalar totalRequests;
        statistics::Scalar acceptedRequests;
        statistics::Scalar queuedRequests;
        statistics::Scalar totalBusyCycles;
        statistics::Scalar totalQueueWaitCycles;
        statistics::Scalar maxQueueDepthSeen;
        statistics::Distribution queueDepthDist;
        statistics::Histogram requestLatencyHist;
        
        // Per-CPU statistics
        statistics::Vector requestsPerCpu;
        statistics::Vector stallsPerCpu;
    } stats;
};

/**
 * Pool of shared functional units
 * Manages multiple shared accelerators across the system
 */
class SharedAcceleratorPool : public SimObject
{
  public:
    PARAMS(SharedAcceleratorPool);
    
    SharedAcceleratorPool(const Params &p);
    ~SharedAcceleratorPool();
    
    /** Sharing modes */
    enum class SharingMode {
        FULLY_SHARED,       // All cores share all accelerators
        FULLY_DEDICATED,    // Each core has private accelerators (no sharing)
        HYBRID,             // Some shared, some dedicated
        ADAPTIVE            // Dynamically switch based on load
    };
    
    /**
     * Request a functional unit of given type.
     * 
     * @param cpuId Requesting CPU
     * @param accelType Type of accelerator needed
     * @param opType Operation type
     * @param callback Completion callback
     * @return Pointer to assigned FU, or nullptr if none available
     */
    SharedFunctionalUnit* requestFU(int cpuId, int threadId,
                                    SharedFunctionalUnit::AccelType accelType,
                                    int opType,
                                    std::function<void()> callback);
    
    /**
     * Check if an accelerator of given type is available.
     */
    bool isAvailable(int cpuId, SharedFunctionalUnit::AccelType accelType) const;
    
    /**
     * Get total queue depth for an accelerator type.
     */
    size_t getTotalQueueDepth(SharedFunctionalUnit::AccelType accelType) const;
    
    /**
     * Register a shared functional unit with the pool.
     */
    void registerFU(SharedFunctionalUnit* fu);
    
    /**
     * Set sharing mode.
     */
    void setSharingMode(SharingMode mode);
    
    /**
     * Get current sharing mode.
     */
    SharingMode getSharingMode() const { return sharingMode; }
    
    /**
     * Configure dedicated FU assignment for a CPU (for dedicated/hybrid modes).
     */
    void assignDedicatedFU(int cpuId, SharedFunctionalUnit* fu);
    
    void init() override;
    
  private:
    /** Configuration */
    SharingMode sharingMode;
    int numCpus;
    
    /** Registered functional units by type */
    std::map<SharedFunctionalUnit::AccelType, 
             std::vector<SharedFunctionalUnit*>> fusByType;
    
    /** Dedicated FU assignments (cpuId -> list of FUs) */
    std::map<int, std::vector<SharedFunctionalUnit*>> dedicatedFUs;
    
    /** Select best FU for a request (load balancing) */
    SharedFunctionalUnit* selectFU(int cpuId, 
                                   SharedFunctionalUnit::AccelType accelType);
    
    /** Statistics */
    struct PoolStats : public statistics::Group
    {
        PoolStats(SharedAcceleratorPool *parent);
        
        statistics::Scalar totalRequests;
        statistics::Scalar successfulRequests;
        statistics::Scalar stalledRequests;
    } stats;
};

} // namespace gem5

#endif // __CPU_SHARED_FU_SHARED_FUNCTIONAL_UNIT_HH__


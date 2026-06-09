/*
 * NOVA Processor - Accelerator Resource Manager
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * Manages accelerator allocation with multiple sharing strategies:
 * - FULLY_SHARED: All cores share all accelerators
 * - FULLY_DEDICATED: Each core has its own accelerators
 * - HYBRID_SHARED: Critical tasks get dedicated, others share
 * - ADAPTIVE: Dynamically switch based on contention
 * 
 * Features:
 * - Priority-based arbitration
 * - Deadline-aware scheduling
 * - Power gating for unused accelerators
 * - Contention monitoring and adaptation
 */

#ifndef __SPACECRAFT_ACCEL_RESOURCE_MANAGER_HH__
#define __SPACECRAFT_ACCEL_RESOURCE_MANAGER_HH__

#include "params/AcceleratorResourceManager.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "spacecraft/global_task_scheduler.hh"
#include "spacecraft/mission_phase_manager.hh"

#include <queue>
#include <vector>
#include <map>
#include <functional>
#include <string>

namespace gem5
{

namespace spacecraft
{

/**
 * Sharing Mode for accelerators
 */
enum class SharingMode {
    FULLY_SHARED,       // All cores share all accelerators
    FULLY_DEDICATED,    // Each core has its own set
    HYBRID_SHARED,      // Some shared, some dedicated
    ADAPTIVE            // Dynamically switch based on load
};

/**
 * Arbitration Policy
 */
enum class ArbitrationPolicy {
    FCFS,               // First-Come-First-Served
    PRIORITY_BASED,     // Based on task criticality
    DEADLINE_AWARE,     // Earliest deadline first
    PROPORTIONAL_SHARE  // Fair sharing based on quotas
};

/**
 * Accelerator Power State
 */
enum class AccelPowerState {
    ACTIVE,         // Fully powered and running
    IDLE,           // Powered but not processing
    POWER_GATED     // Fully powered off
};

/**
 * Request for accelerator access
 */
struct AccelRequest {
    int taskId;
    int coreId;
    AccelType type;
    CriticalityLevel criticality;
    Tick requestTime;
    Tick deadline;
    std::function<void(bool)> callback;
};

/**
 * Accelerator Instance
 */
struct AccelInstance {
    AccelType type;
    int instanceId;
    bool isBusy;
    int currentOwnerTask;
    int dedicatedToCoreId;  // -1 if shared
    
    // Queue for waiting requests
    std::queue<AccelRequest> waitQueue;
    
    // Performance counters
    uint64_t totalRequests;
    uint64_t totalBusyCycles;
    Tick lastUsedTime;
    float avgLatency;
    
    // Power state
    AccelPowerState powerState;
    float currentPowerMW;
    
    AccelInstance()
        : type(AccelType::NONE), instanceId(-1), isBusy(false),
          currentOwnerTask(-1), dedicatedToCoreId(-1),
          totalRequests(0), totalBusyCycles(0), lastUsedTime(0), avgLatency(0),
          powerState(AccelPowerState::IDLE), currentPowerMW(0) {}
};

/**
 * Manager Statistics
 */
struct AccelManagerStats {
    std::map<AccelType, uint64_t> totalRequestsPerType;
    std::map<AccelType, uint64_t> totalBusyCyclesPerType;
    std::map<AccelType, float> avgUtilizationPerType;
    std::map<AccelType, float> avgQueueDepthPerType;
    std::map<AccelType, Tick> avgWaitTimePerType;
    uint64_t totalContentionEvents;
    uint64_t adaptiveModeChanges;
    float totalPowerConsumed;
    uint64_t powerGatingEvents;
};

/**
 * Accelerator Resource Manager
 */
class AcceleratorResourceManager : public ClockedObject
{
  public:
    PARAMS(AcceleratorResourceManager);
    AcceleratorResourceManager(const Params &p);
    ~AcceleratorResourceManager();
    
    // Accelerator acquisition
    bool tryAcquire(AccelType type, int taskId, int coreId, CriticalityLevel criticality);
    bool tryAcquireWithDeadline(AccelType type, int taskId, int coreId, 
                                 CriticalityLevel criticality, Tick deadline);
    void release(AccelType type, int taskId);
    
    // Queue-based acquisition (for async)
    void queueRequest(AccelRequest &req);
    
    // Query status
    bool isAccelAvailable(AccelType type) const;
    size_t getQueueDepth(AccelType type) const;
    float getContention(AccelType type) const;
    float getCurrentPower() const;
    
    // Mode configuration
    void setSharingMode(SharingMode mode);
    SharingMode getSharingMode() const { return currentMode; }
    void setArbitrationPolicy(ArbitrationPolicy policy);
    ArbitrationPolicy getArbitrationPolicy() const { return arbitrationPolicy; }
    
    // Mission phase integration
    void reconfigure(MissionPhase phase);
    
    // Power management
    void powerGateAccel(AccelType type, int instanceId);
    void powerUpAccel(AccelType type, int instanceId);
    void powerGateNonEssential();
    void powerUpAll();
    
    // Statistics
    const AccelManagerStats& getStats() const { return stats; }
    void resetStats();
    void printStatistics();
    
    // Instance configuration
    void addAccelInstance(AccelType type, int instanceId);
    void setDedicated(AccelType type, int instanceId, int coreId);
    
  private:
    // Configuration
    SharingMode currentMode;
    ArbitrationPolicy arbitrationPolicy;
    int numCores;
    
    // Accelerator instances per type
    std::map<AccelType, std::vector<AccelInstance>> accelerators;
    
    // Configuration for each type
    struct AccelConfig {
        int numInstances;
        float activePowerMW;
        float idlePowerMW;
        Cycles typicalLatency;
    };
    std::map<AccelType, AccelConfig> accelConfigs;
    
    // Contention thresholds
    float highContentionThreshold;
    float lowContentionThreshold;
    Tick idleThreshold;  // Time before power gating
    
    // Statistics
    AccelManagerStats stats;
    
    // Events
    EventFunctionWrapper contentionCheckEvent;
    EventFunctionWrapper powerManagementEvent;
    
    // Acquisition strategies
    bool tryAcquireShared(AccelType type, int taskId, int coreId, CriticalityLevel criticality);
    bool tryAcquireDedicated(AccelType type, int taskId, int coreId, CriticalityLevel criticality);
    bool tryAcquireHybrid(AccelType type, int taskId, int coreId, CriticalityLevel criticality);
    bool tryAcquireAdaptive(AccelType type, int taskId, int coreId, CriticalityLevel criticality);
    
    // Queue processing
    void processWaitQueue(AccelInstance &inst);
    void processFCFS(AccelInstance &inst);
    void processPriorityBased(AccelInstance &inst);
    void processDeadlineAware(AccelInstance &inst);
    void processProportionalShare(AccelInstance &inst);
    
    // Helper functions
    void allocate(AccelInstance &inst, int taskId);
    AccelInstance& findLeastLoadedInstance(std::vector<AccelInstance> &instances);
    int findMostLoadedCore();
    float calculateContention(AccelType type) const;
    void updateContentionStats();
    
    // Adaptive mode helpers
    void checkContention();
    void adaptSharingMode();
    void powerUpDedicatedInstance(AccelType type);
    void powerGateUnderutilizedInstances(AccelType type);
    
    // Power calculations
    float getActivePower(AccelType type) const;
    float getIdlePower(AccelType type) const;
    
    // String conversion
    static std::string accelTypeToString(AccelType type);
    static std::string sharingModeToString(SharingMode mode);
};

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_ACCEL_RESOURCE_MANAGER_HH__





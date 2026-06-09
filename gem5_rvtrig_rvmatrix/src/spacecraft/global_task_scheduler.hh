/*
 * NOVA Processor - Global Task Scheduler
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * Features:
 * - Criticality-aware scheduling (MISSION_CRITICAL > SAFETY_CRITICAL > ...)
 * - Multiple scheduling policies (Fixed Priority, EDF, Energy-Aware)
 * - Preemption support with context switch overhead modeling
 * - Deadline monitoring and miss tracking
 * - Per-core ready queues with load balancing
 */

#ifndef __SPACECRAFT_GLOBAL_TASK_SCHEDULER_HH__
#define __SPACECRAFT_GLOBAL_TASK_SCHEDULER_HH__

#include "params/GlobalTaskScheduler.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "spacecraft/mission_phase_manager.hh"

#include <queue>
#include <vector>
#include <map>
#include <set>
#include <functional>
#include <string>

namespace gem5
{

namespace spacecraft
{

// Forward declarations
class AcceleratorResourceManager;

/**
 * Task Criticality Levels (ISRO-inspired)
 */
enum class CriticalityLevel {
    MISSION_CRITICAL = 0,   // GNC, life support - cannot miss
    SAFETY_CRITICAL = 1,    // Hazard avoidance, fault detection
    OPERATIONAL = 2,        // Navigation, communication
    SCIENCE = 3,            // Payload processing
    HOUSEKEEPING = 4        // Telemetry, diagnostics
};

/**
 * Accelerator types required by tasks
 */
enum class AccelType {
    TRIG_ACCEL,
    MAT_ACCEL,
    VPU,
    NPU,
    NONE
};

/**
 * Task State
 */
enum class TaskState {
    READY,          // Ready to run
    RUNNING,        // Currently executing
    WAITING_ACCEL,  // Waiting for accelerator
    BLOCKED,        // Blocked on I/O or other
    SUSPENDED,      // Suspended by scheduler
    COMPLETED       // Finished execution
};

/**
 * Task definition structure
 */
struct SpacecraftTask {
    int id;
    std::string name;
    CriticalityLevel criticality;
    
    // Temporal parameters
    Tick period;            // For periodic tasks (0 = aperiodic)
    Tick deadline;          // Relative deadline
    Tick wcet;              // Worst-case execution time
    Tick nextRelease;       // Next release time
    Tick remainingTime;     // Remaining execution time
    
    // State
    TaskState state;
    bool isPreemptible;
    int coreAffinity;       // -1 for any core
    
    // Resource requirements
    std::vector<AccelType> accelNeeds;
    uint64_t memoryKB;
    float powerWatts;
    
    // Active in which phases
    std::set<MissionPhase> activePhases;
    
    // Statistics
    uint64_t numReleases;
    uint64_t numDeadlineMisses;
    Tick totalExecutionTime;
    Tick maxResponseTime;
    Tick minResponseTime;
    
    SpacecraftTask()
        : id(-1), criticality(CriticalityLevel::HOUSEKEEPING),
          period(0), deadline(0), wcet(0), nextRelease(0), remainingTime(0),
          state(TaskState::READY), isPreemptible(true), coreAffinity(-1),
          memoryKB(0), powerWatts(0.0),
          numReleases(0), numDeadlineMisses(0), totalExecutionTime(0),
          maxResponseTime(0), minResponseTime(UINT64_MAX) {}
};

/**
 * Scheduler Statistics
 */
struct SchedulerStats {
    uint64_t totalContextSwitches;
    uint64_t totalPreemptions;
    uint64_t totalDeadlineMisses;
    uint64_t missionCriticalMisses;  // These are fatal
    uint64_t tasksCompleted;
    uint64_t tasksShed;              // Dropped due to power constraints
    Tick totalSchedulingOverhead;
    std::map<int, float> perCoreUtilization;
    std::map<CriticalityLevel, uint64_t> missesPerCriticality;
};

/**
 * Global Task Scheduler - Manages task scheduling across all cores
 */
class GlobalTaskScheduler : public ClockedObject
{
  public:
    PARAMS(GlobalTaskScheduler);
    GlobalTaskScheduler(const Params &p);
    ~GlobalTaskScheduler();
    
    // Task management
    void registerTask(const SpacecraftTask &task);
    void unregisterTask(int taskId);
    SpacecraftTask* getTask(int taskId);
    
    // Scheduling control
    void runScheduler();
    void setPolicy(SchedPolicy policy);
    SchedPolicy getPolicy() const { return currentPolicy; }
    
    // Power management
    void setPowerBudget(float watts);
    float getPowerBudget() const { return powerBudget; }
    float getCurrentPower() const;
    
    // Task set management
    void setActiveTaskSet(const std::set<int> &taskIds);
    void suspendTask(int taskId);
    void resumeTask(int taskId);
    
    // Core management
    int getNumCores() const { return numCores; }
    bool isCoreIdle(int coreId) const;
    SpacecraftTask* getRunningTask(int coreId);
    
    // Deadline monitoring
    void checkDeadlines();
    bool hasMissionCriticalMiss() const;
    
    // Statistics
    const SchedulerStats& getStats() const { return stats; }
    void resetStats();
    void printStatistics();
    
    // Callbacks
    void onTaskComplete(int taskId, int coreId);
    void onAccelAvailable(AccelType type);
    
    // Connect to accelerator manager
    void setAcceleratorManager(AcceleratorResourceManager *mgr);
    
  private:
    // Configuration
    int numCores;
    SchedPolicy currentPolicy;
    float powerBudget;
    Cycles contextSwitchOverhead;
    Cycles schedulingOverhead;
    
    // Task storage
    std::map<int, SpacecraftTask> tasks;
    std::set<int> activeTaskIds;
    
    // Per-core state
    std::vector<std::vector<SpacecraftTask*>> readyQueues;  // Per-core ready queues
    std::vector<SpacecraftTask*> runningTasks;              // Currently running on each core
    
    // Accelerator manager
    AcceleratorResourceManager *accelManager;
    
    // Statistics
    SchedulerStats stats;
    
    // Events
    EventFunctionWrapper scheduleEvent;
    EventFunctionWrapper periodicReleaseEvent;
    EventFunctionWrapper deadlineCheckEvent;
    
    // Scheduling policy implementations
    void scheduleFixedPriority();
    void scheduleMixedCriticalityEDF();
    void scheduleEnergyAwareEDF();
    void scheduleMinimal();
    void scheduleRoundRobin();
    
    // Helper functions
    int getPriority(const SpacecraftTask *task) const;
    int selectCore();
    int selectCoreForTask(const SpacecraftTask *task);
    void dispatch(SpacecraftTask *task, int coreId);
    void preempt(SpacecraftTask *task, int coreId);
    void releasePeriodicTasks();
    void shedLowPriorityTasks();
    bool tryAcquireAccelerators(SpacecraftTask *task);
    void releaseAccelerators(SpacecraftTask *task);
    
    // Task comparators
    struct PriorityComparator {
        const GlobalTaskScheduler *scheduler;
        bool operator()(SpacecraftTask* a, SpacecraftTask* b) const;
    };
    
    struct EDFComparator {
        bool operator()(SpacecraftTask* a, SpacecraftTask* b) const;
    };
    
    // Scheduling period
    Tick schedulingPeriod;
};

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_GLOBAL_TASK_SCHEDULER_HH__


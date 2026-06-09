/*
 * NOVA Spacecraft Processor - Mission-Aware Scheduling Framework
 * PhD Research Extension: Phase-Based Task Scheduling & Accelerator Optimization
 * 
 * Key Features:
 * 1. Priority-based scheduling with mission criticality
 * 2. Deadline-aware accelerator allocation
 * 3. Phase-specific scheduling policies
 * 4. Dynamic accelerator reservation
 * 5. Real-time guarantee analysis
 */

#ifndef __CPU_SIMPLE_NOVA_SCHEDULER_HH__
#define __CPU_SIMPLE_NOVA_SCHEDULER_HH__

#include <queue>
#include <vector>
#include <map>
#include <set>
#include <mutex>
#include <string>
#include <cstdint>

#include "base/types.hh"
#include "sim/cur_tick.hh"

namespace gem5
{

// Forward declaration
class TimingSimpleCPU;

// ============================================================================
// Mission Phase Definitions (matching timing.hh)
// ============================================================================

enum class NOVAMissionPhase {
    LAUNCH = 0,       // Heavy ADCS, sensor fusion - high power available
    CRUISE = 1,       // Low activity - minimal power, conservation mode
    APPROACH = 2,     // Vision, navigation, AI - moderate power
    LANDING = 3,      // Real-time control - maximum power needed
    SURFACE_OPS = 4,  // Science processing - solar dependent
    ECLIPSE = 5,      // Battery only - minimum power mode
    SAFE_MODE = 6,    // Emergency - critical operations only
    IDLE = 7          // Idle state
};

// ============================================================================
// Task Priority and Criticality Levels
// ============================================================================

enum class TaskCriticality {
    CRITICAL_SAFETY = 0,     // Must complete (e.g., attitude control)
    CRITICAL_NAVIGATION = 1, // High priority (e.g., orbital calculations)
    IMPORTANT_SCIENCE = 2,   // Science data collection
    NORMAL_TELEMETRY = 3,    // Routine communications
    LOW_HOUSEKEEPING = 4,    // Background tasks
    BEST_EFFORT = 5          // No guarantees
};

enum class SchedulingPolicy {
    POLICY_FIFO,                    // First-In-First-Out
    POLICY_PRIORITY,                // Static priority
    POLICY_EDF,                     // Earliest Deadline First
    POLICY_RATE_MONOTONIC,          // Rate Monotonic (shorter period = higher priority)
    POLICY_PHASE_AWARE,             // Mission-phase-specific
    POLICY_POWER_AWARE,             // Consider power budget
    POLICY_HYBRID                   // Combination of multiple policies
};

// ============================================================================
// Accelerator Configuration Structure
// ============================================================================

struct NOVAAcceleratorConfig {
    int numTrig;
    int numMatrix;
    int numVPU;
    int numNPU;
    int numGPU;
    std::string configName;
    double estimatedPower;
    double estimatedArea;
    double estimatedLatency;
    
    int getTotalAccelerators() const {
        return numTrig + numMatrix + numVPU + numNPU + numGPU;
    }
};

// ============================================================================
// Task Descriptor
// ============================================================================

struct TaskDescriptor {
    // Task identification
    int taskID = 0;
    std::string taskName;
    int cpuID = 0;                  // Which CPU is running this task
    
    // Timing constraints
    Tick arrivalTime = 0;           // When task arrived
    Tick deadline = 0;              // Absolute deadline (0 = no deadline)
    Tick period = 0;                // For periodic tasks (0 = aperiodic)
    Tick estimatedExecutionTime = 0;// WCET estimate
    Tick actualExecutionTime = 0;   // Measured execution time
    Tick startTime = 0;             // When task started executing
    Tick completionTime = 0;        // When task completed
    
    // Criticality and priority
    TaskCriticality criticality = TaskCriticality::BEST_EFFORT;
    int staticPriority = 0;         // Higher number = higher priority
    int dynamicPriority = 0;        // Can change based on conditions
    
    // Accelerator requirements
    std::map<int, int> accelRequirements;  // accelType -> count needed
    bool needsExclusiveAccess = false;     // Needs dedicated accelerator?
    Tick accelHoldTime = 0;                // How long will it hold accelerator
    
    // Power requirements
    double powerBudget = 0.0;       // Maximum power this task can use
    double actualPowerUsed = 0.0;   // Measured power consumption
    
    // Mission phase affinity
    std::set<NOVAMissionPhase> compatiblePhases;  // Which phases can run this
    NOVAMissionPhase preferredPhase = NOVAMissionPhase::IDLE;    // Best phase for this task
    
    // Status tracking
    bool isWaiting = false;         // Waiting for accelerator
    bool isExecuting = false;       // Currently executing
    bool isCompleted = false;       // Finished
    bool missedDeadline = false;    // Failed to meet deadline
    int waitingForAccelType = -1;   // Which accelerator type (-1 if none)
    
    // Statistics
    uint64_t timesPreempted = 0;
    uint64_t totalWaitTime = 0;
    uint64_t totalBlockedTime = 0;
};

// ============================================================================
// Accelerator Reservation System
// ============================================================================

struct AcceleratorReservation {
    int accelType = 0;
    int accelInstanceID = 0;        // Which specific instance
    TaskDescriptor* reservedBy = nullptr;     // Task holding reservation
    Tick reservationStart = 0;
    Tick reservationEnd = 0;
    bool isExclusive = false;       // Exclusive or shared access
};

class AcceleratorReservationManager {
private:
    // Reservation table: accelType -> list of reservations
    std::map<int, std::vector<AcceleratorReservation>> reservations;
    
    // Available accelerators: accelType -> count
    std::map<int, int> availableAccels;
    
    // Mutex for thread safety
    std::mutex reservationMutex;
    
public:
    AcceleratorReservationManager();
    
    // Initialize with available accelerators
    void setAvailableAccelerators(int trig, int matrix, int vpu, int npu, int gpu = 1);
    
    // Reserve accelerator for task
    bool reserveAccelerator(int accelType, TaskDescriptor* task, 
                           Tick duration, bool exclusive = false);
    
    // Release accelerator reservation
    void releaseAccelerator(int accelType, TaskDescriptor* task);
    
    // Check if accelerator is available
    bool isAcceleratorAvailable(int accelType, Tick currentTime);
    
    // Get number of available accelerators
    int getAvailableCount(int accelType, Tick currentTime);
    
    // Reserve multiple accelerators atomically
    bool reserveMultiple(const std::map<int, int>& requirements,
                        TaskDescriptor* task, Tick duration);
    
    // Statistics
    void printReservationStats();
};

// ============================================================================
// Task Queue Management
// ============================================================================

class TaskQueue {
private:
    std::vector<TaskDescriptor*> tasks;
    SchedulingPolicy policy;
    NOVAMissionPhase currentPhase;
    
public:
    TaskQueue(SchedulingPolicy pol = SchedulingPolicy::POLICY_PRIORITY);
    
    // Add task to queue
    void enqueue(TaskDescriptor* task);
    
    // Remove and return highest priority task
    TaskDescriptor* dequeue();
    
    // Peek at highest priority task without removing
    TaskDescriptor* peek();
    
    // Check if queue is empty
    bool isEmpty() const { return tasks.empty(); }
    
    // Get queue size
    size_t size() const { return tasks.size(); }
    
    // Update priorities based on current conditions
    void updatePriorities(Tick currentTime, NOVAMissionPhase phase);
    
    // Set scheduling policy
    void setPolicy(SchedulingPolicy pol) { policy = pol; }
    
    // Set current mission phase
    void setPhase(NOVAMissionPhase phase) { currentPhase = phase; }
    
    // Remove specific task
    bool removeTask(TaskDescriptor* task);
    
private:
    // Comparison functions for different policies
    bool compareFIFO(TaskDescriptor* t1, TaskDescriptor* t2);
    bool comparePriority(TaskDescriptor* t1, TaskDescriptor* t2);
    bool compareEDF(TaskDescriptor* t1, TaskDescriptor* t2);
    bool compareRateMonotonic(TaskDescriptor* t1, TaskDescriptor* t2);
    bool comparePhaseAware(TaskDescriptor* t1, TaskDescriptor* t2);
    
    // Sort queue based on current policy
    void sortQueue();
};

// ============================================================================
// Mission-Phase-Aware Scheduler
// ============================================================================

class MissionPhaseScheduler {
private:
    // Current scheduling configuration
    SchedulingPolicy currentPolicy;
    NOVAMissionPhase currentPhase;
    
    // Task queues (one per CPU or global)
    std::vector<TaskQueue> perCPUQueues;
    TaskQueue globalQueue;
    bool useGlobalQueue;
    
    // Active tasks
    std::map<int, TaskDescriptor*> activeTasks;
    
    // Waiting tasks (blocked on accelerator)
    std::vector<TaskDescriptor*> waitingTasks;
    
    // Completed tasks
    std::vector<TaskDescriptor*> completedTasks;
    
    // Accelerator reservation system
    AcceleratorReservationManager reservationManager;
    
    // Phase-specific policies
    std::map<NOVAMissionPhase, SchedulingPolicy> phasePolicies;
    
    // Phase-specific accelerator allocations
    std::map<NOVAMissionPhase, NOVAAcceleratorConfig> phaseAccelConfigs;
    
    // Statistics
    struct SchedulerStats {
        uint64_t totalTasksScheduled = 0;
        uint64_t totalTasksCompleted = 0;
        uint64_t totalDeadlinesMissed = 0;
        uint64_t totalPreemptions = 0;
        uint64_t totalContextSwitches = 0;
        uint64_t totalAccelWaitTime = 0;
        
        // Per-priority statistics
        std::map<TaskCriticality, uint64_t> tasksPerCriticality;
        std::map<TaskCriticality, uint64_t> deadlineMissesPerCriticality;
        
        // Per-phase statistics
        std::map<NOVAMissionPhase, uint64_t> tasksPerPhase;
        std::map<NOVAMissionPhase, Tick> executionTimePerPhase;
    } stats;
    
public:
    MissionPhaseScheduler(int numCPUs = 1, bool globalQ = true);
    ~MissionPhaseScheduler();
    
    // Initialize scheduler
    void initialize(const NOVAAcceleratorConfig& initialConfig);
    
    // Task management
    void submitTask(TaskDescriptor* task);
    void completeTask(TaskDescriptor* task);
    TaskDescriptor* getNextTask(int cpuID);
    
    // Scheduling decisions
    void schedule(Tick currentTime);
    void reschedule(Tick currentTime);
    
    // Phase management
    void transitionToPhase(NOVAMissionPhase newPhase, Tick currentTime);
    void updatePhasePolicy(NOVAMissionPhase phase, SchedulingPolicy policy);
    void updatePhaseAccelConfig(NOVAMissionPhase phase, const NOVAAcceleratorConfig& config);
    
    // Accelerator management
    bool requestAccelerator(TaskDescriptor* task, int accelType);
    void releaseAccelerator(TaskDescriptor* task, int accelType);
    bool canAcquireAccelerators(TaskDescriptor* task);
    
    // Admission control
    bool admitTask(TaskDescriptor* task);
    double calculateCPUUtilization();
    bool isSchedulable(const std::vector<TaskDescriptor*>& taskSet);
    
    // Real-time analysis
    Tick calculateWCRT(TaskDescriptor* task);  // Worst-Case Response Time
    double calculateUtilization(const std::vector<TaskDescriptor*>& taskSet);
    bool rateMonotonicTest(const std::vector<TaskDescriptor*>& taskSet);
    bool edfSchedulabilityTest(const std::vector<TaskDescriptor*>& taskSet);
    
    // Optimization
    NOVAAcceleratorConfig optimizeAccelConfig(const std::vector<TaskDescriptor*>& taskSet,
                                              NOVAMissionPhase phase);
    void balanceLoad();
    void migrateTask(TaskDescriptor* task, int fromCPU, int toCPU);
    
    // Statistics and reporting
    void printSchedulerStats();
    void exportSchedulingTrace(const std::string& filename);
    void analyzeSchedulability();
    
    // Getters
    SchedulingPolicy getCurrentPolicy() const { return currentPolicy; }
    NOVAMissionPhase getCurrentPhase() const { return currentPhase; }
    const SchedulerStats& getStats() const { return stats; }
    
    // Phase name helper
    static const char* getPhaseName(NOVAMissionPhase phase);
};

// ============================================================================
// Accelerator Optimizer
// ============================================================================

class AcceleratorOptimizer {
private:
    // Workload characterization
    std::vector<TaskDescriptor*> taskSet;
    std::map<NOVAMissionPhase, std::vector<TaskDescriptor*>> tasksPerPhase;
    
    // Constraints
    double maxPowerBudget;
    double maxAreaBudget;
    int maxTotalAccelerators;
    
public:
    // Optimization objectives
    enum class OptimizationObjective {
        MINIMIZE_EXECUTION_TIME,
        MINIMIZE_POWER,
        MINIMIZE_AREA,
        MINIMIZE_COST,
        MAXIMIZE_SCHEDULABILITY,
        PARETO_OPTIMAL
    };
    
private:
    OptimizationObjective objective;
    
public:
    AcceleratorOptimizer();
    
    // Set task set and constraints
    void setTaskSet(const std::vector<TaskDescriptor*>& tasks);
    void setConstraints(double maxPower, double maxArea, int maxAccels);
    void setObjective(OptimizationObjective obj) { objective = obj; }
    
    // Optimization algorithms
    NOVAAcceleratorConfig optimizeGreedy(NOVAMissionPhase phase);
    NOVAAcceleratorConfig optimizeILP(NOVAMissionPhase phase);
    NOVAAcceleratorConfig optimizeGenetic(NOVAMissionPhase phase);
    
    // Analysis functions
    double estimateExecutionTime(const NOVAAcceleratorConfig& config, 
                                 const std::vector<TaskDescriptor*>& tasks);
    double estimatePower(const NOVAAcceleratorConfig& config);
    double estimateArea(const NOVAAcceleratorConfig& config);
    
    // Schedulability-driven optimization
    NOVAAcceleratorConfig findMinimalSchedulableConfig(NOVAMissionPhase phase);
    std::vector<NOVAAcceleratorConfig> findParetoFrontier(NOVAMissionPhase phase);
    
    // Sensitivity analysis
    void sensitivityAnalysis(const NOVAAcceleratorConfig& baseConfig);
    
    // Export results
    void exportOptimizationResults(const std::string& filename);
};

// ============================================================================
// Scheduling Policy Implementations
// ============================================================================

class SchedulingPolicyManager {
public:
    // Rate Monotonic Scheduling
    static void applyRateMonotonic(std::vector<TaskDescriptor*>& tasks);
    
    // Earliest Deadline First
    static void applyEDF(std::vector<TaskDescriptor*>& tasks, Tick currentTime);
    
    // Priority-based with aging
    static void applyPriorityWithAging(std::vector<TaskDescriptor*>& tasks, 
                                       Tick currentTime);
    
    // Phase-aware scheduling
    static void applyPhaseAware(std::vector<TaskDescriptor*>& tasks,
                               NOVAMissionPhase phase);
    
    // Power-aware scheduling
    static void applyPowerAware(std::vector<TaskDescriptor*>& tasks,
                               double availablePower);
    
    // Hybrid policy
    static void applyHybrid(std::vector<TaskDescriptor*>& tasks,
                           NOVAMissionPhase phase, Tick currentTime,
                           double availablePower);
};

// ============================================================================
// Helper Functions
// ============================================================================

// Print task information
void printTaskInfo(const TaskDescriptor* task);

// Get criticality name
const char* getCriticalityName(TaskCriticality crit);

// Get policy name
const char* getPolicyName(SchedulingPolicy policy);

} // namespace gem5

#endif // __CPU_SIMPLE_NOVA_SCHEDULER_HH__

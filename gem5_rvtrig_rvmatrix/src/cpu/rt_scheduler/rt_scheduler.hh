/*
 * Copyright (c) 2024 NOVA Processor Research
 * Real-Time Scheduling for Spacecraft Accelerators
 *
 * RTScheduler: Contention-aware real-time scheduler for heterogeneous
 * accelerators in spacecraft processors.
 *
 * Implements:
 * - Contention-Aware Rate Monotonic (CA-RM)
 * - Contention-Aware Earliest Deadline First (CA-EDF)
 * - Accelerator Reservation Protocol (ARP)
 */

#ifndef __CPU_RT_SCHEDULER_RT_SCHEDULER_HH__
#define __CPU_RT_SCHEDULER_RT_SCHEDULER_HH__

#include <vector>
#include <queue>
#include <string>
#include <functional>
#include <cstdint>

#include "contention_predictor.hh"

namespace gem5
{

/**
 * Task criticality levels for mixed-criticality scheduling
 */
enum class Criticality {
    SAFETY_CRITICAL = 0,   // Must never miss deadline (GNC, ADCS)
    MISSION_CRITICAL = 1,  // Should rarely miss deadline
    BEST_EFFORT = 2        // Can be degraded under overload
};

/**
 * Accelerator types that can be requested by tasks
 */
enum class AcceleratorType {
    NONE = 0,
    TRIG_ACCEL = 1,
    MAT_ACCEL = 2,
    VPU = 3,
    NPU = 4
};

/**
 * Real-time task structure
 */
struct RTTask {
    int taskId;
    std::string name;
    
    // Timing parameters (in cycles)
    uint64_t period;           // Task period
    uint64_t deadline;         // Relative deadline
    uint64_t baseWCET;         // Base WCET without contention
    uint64_t adjustedWCET;     // WCET with contention prediction
    
    // Real-time parameters
    int priority;              // Static priority (lower = higher priority)
    Criticality criticality;
    
    // Accelerator requirements
    std::vector<AcceleratorType> requiredAccelerators;
    int accelOpsPerPeriod;     // Number of accelerator ops per period
    
    // Runtime state
    uint64_t nextRelease;      // Next release time
    uint64_t currentDeadline;  // Absolute deadline of current instance
    bool isActive;             // Currently executing
    bool isReady;              // Ready to execute
    
    // Statistics
    uint64_t completedInstances;
    uint64_t deadlineMisses;
    uint64_t totalResponseTime;
    uint64_t worstResponseTime;
    
    // Constructor
    RTTask() : taskId(0), period(0), deadline(0), baseWCET(0), adjustedWCET(0),
               priority(0), criticality(Criticality::BEST_EFFORT),
               accelOpsPerPeriod(0), nextRelease(0), currentDeadline(0),
               isActive(false), isReady(false), completedInstances(0),
               deadlineMisses(0), totalResponseTime(0), worstResponseTime(0) {}
};

/**
 * Scheduling policies
 */
enum class SchedulingPolicy {
    CA_RM,      // Contention-Aware Rate Monotonic
    CA_EDF,     // Contention-Aware Earliest Deadline First
    CA_LLF,     // Contention-Aware Least Laxity First
    RM,         // Standard Rate Monotonic (baseline)
    EDF         // Standard EDF (baseline)
};

/**
 * RTScheduler - Main real-time scheduler class
 */
class RTScheduler
{
  public:
    RTScheduler(int numCores, SchedulingPolicy policy = SchedulingPolicy::CA_RM);
    ~RTScheduler();

    // Task management
    void addTask(RTTask& task);
    void removeTask(int taskId);
    RTTask* getTask(int taskId);
    
    // Scheduling
    RTTask* selectNextTask(uint64_t currentTime);
    void releaseTask(int taskId, uint64_t releaseTime);
    void completeTask(int taskId, uint64_t completionTime);
    
    // Contention management
    void updateContentionPredictions();
    double getCurrentUtilization() const;
    double getSystemLoad() const;
    
    // Schedulability analysis
    bool checkSchedulability() const;
    double getSchedulabilityBound() const;
    double getUtilizationBound() const;
    
    // Accelerator reservation
    bool requestAccelerator(int taskId, AcceleratorType accel);
    void releaseAccelerator(int taskId, AcceleratorType accel);
    int getAcceleratorOwner(AcceleratorType accel) const;
    
    // Statistics
    void printStatistics() const;
    uint64_t getTotalDeadlineMisses() const;
    double getDeadlineMissRate() const;
    
    // Configuration
    void setPolicy(SchedulingPolicy policy) { m_policy = policy; }
    SchedulingPolicy getPolicy() const { return m_policy; }
    void setNumCores(int numCores) { m_numCores = numCores; }
    
  private:
    // Core scheduling methods
    RTTask* selectByRM(uint64_t currentTime);
    RTTask* selectByEDF(uint64_t currentTime);
    RTTask* selectByLLF(uint64_t currentTime);
    
    // Priority calculations
    void assignRMPriorities();
    int calculateDynamicPriority(RTTask* task, uint64_t currentTime);
    
    // WCET adjustment
    void adjustWCETForContention(RTTask& task);
    
    // Schedulability tests
    bool rmSchedulabilityTest() const;
    bool edfSchedulabilityTest() const;
    
    // Priority inheritance for accelerators
    void applyPriorityInheritance(int waitingTaskId, AcceleratorType accel);
    void restorePriority(int taskId);
    
    // Member variables
    std::vector<RTTask> m_tasks;
    SchedulingPolicy m_policy;
    int m_numCores;
    double m_currentUtilization;
    
    // Accelerator ownership (-1 = free)
    int m_trigAccelOwner;
    int m_matAccelOwner;
    int m_vpuOwner;
    int m_npuOwner;
    
    // Priority inheritance tracking
    std::vector<int> m_originalPriorities;
    
    // Statistics
    uint64_t m_totalSchedulingDecisions;
    uint64_t m_totalContextSwitches;
    uint64_t m_totalPreemptions;
};

/**
 * Comparison functors for priority queues
 */
struct RMComparator {
    bool operator()(const RTTask* a, const RTTask* b) const {
        return a->priority > b->priority; // Lower priority value = higher priority
    }
};

struct EDFComparator {
    bool operator()(const RTTask* a, const RTTask* b) const {
        return a->currentDeadline > b->currentDeadline;
    }
};

struct LLFComparator {
    uint64_t currentTime;
    LLFComparator(uint64_t t) : currentTime(t) {}
    
    bool operator()(const RTTask* a, const RTTask* b) const {
        // Laxity = deadline - current_time - remaining_execution
        int64_t laxityA = a->currentDeadline - currentTime - a->adjustedWCET;
        int64_t laxityB = b->currentDeadline - currentTime - b->adjustedWCET;
        return laxityA > laxityB;
    }
};

} // namespace gem5

#endif // __CPU_RT_SCHEDULER_RT_SCHEDULER_HH__


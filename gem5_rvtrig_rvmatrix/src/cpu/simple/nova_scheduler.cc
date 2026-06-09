/*
 * NOVA Spacecraft Processor - Mission-Aware Scheduler Implementation
 * PhD Research: Phase-Based Task Scheduling & Accelerator Optimization
 */

#include "cpu/simple/nova_scheduler.hh"
#include "base/logging.hh"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <fstream>

namespace gem5
{

// ============================================================================
// Accelerator Reservation Manager Implementation
// ============================================================================

AcceleratorReservationManager::AcceleratorReservationManager()
{
    availableAccels[0] = 1;  // Trig (default)
    availableAccels[1] = 1;  // Matrix
    availableAccels[2] = 1;  // VPU
    availableAccels[3] = 1;  // NPU
    availableAccels[4] = 1;  // GPU
}

void
AcceleratorReservationManager::setAvailableAccelerators(int trig, int matrix, 
                                                        int vpu, int npu,
                                                        int gpu)
{
    std::lock_guard<std::mutex> lock(reservationMutex);
    availableAccels[0] = trig;
    availableAccels[1] = matrix;
    availableAccels[2] = vpu;
    availableAccels[3] = npu;
    availableAccels[4] = gpu;
}

bool
AcceleratorReservationManager::reserveAccelerator(int accelType, 
                                                  TaskDescriptor* task,
                                                  Tick duration, bool exclusive)
{
    std::lock_guard<std::mutex> lock(reservationMutex);
    
    Tick currentTime = curTick();
    
    // Count available accelerators at current time
    int available = availableAccels[accelType];
    for (const auto& res : reservations[accelType]) {
        if (res.reservationEnd > currentTime) {
            available--;
        }
    }
    
    if (available <= 0) {
        return false;  // No accelerators available
    }
    
    // Create reservation
    AcceleratorReservation reservation;
    reservation.accelType = accelType;
    reservation.accelInstanceID = availableAccels[accelType] - available;
    reservation.reservedBy = task;
    reservation.reservationStart = currentTime;
    reservation.reservationEnd = currentTime + duration;
    reservation.isExclusive = exclusive;
    
    reservations[accelType].push_back(reservation);
    
    return true;
}

void
AcceleratorReservationManager::releaseAccelerator(int accelType, 
                                                  TaskDescriptor* task)
{
    std::lock_guard<std::mutex> lock(reservationMutex);
    
    auto& resList = reservations[accelType];
    resList.erase(
        std::remove_if(resList.begin(), resList.end(),
                      [task](const AcceleratorReservation& res) {
                          return res.reservedBy == task;
                      }),
        resList.end()
    );
}

bool
AcceleratorReservationManager::isAcceleratorAvailable(int accelType, Tick currentTime)
{
    return getAvailableCount(accelType, currentTime) > 0;
}

int
AcceleratorReservationManager::getAvailableCount(int accelType, Tick currentTime)
{
    std::lock_guard<std::mutex> lock(reservationMutex);
    
    int available = availableAccels[accelType];
    for (const auto& res : reservations[accelType]) {
        if (res.reservationEnd > currentTime) {
            available--;
        }
    }
    
    return std::max(0, available);
}

bool
AcceleratorReservationManager::reserveMultiple(
    const std::map<int, int>& requirements,
    TaskDescriptor* task, Tick duration)
{
    // Note: Already locked in calling function or use separate lock
    Tick currentTime = curTick();
    
    // First, check if all requirements can be met
    for (const auto& req : requirements) {
        int accelType = req.first;
        int needed = req.second;
        
        int available = getAvailableCount(accelType, currentTime);
        if (available < needed) {
            return false;  // Cannot satisfy all requirements
        }
    }
    
    // All requirements can be met, make reservations
    for (const auto& req : requirements) {
        int accelType = req.first;
        int needed = req.second;
        
        for (int i = 0; i < needed; i++) {
            reserveAccelerator(accelType, task, duration, false);
        }
    }
    
    return true;
}

void
AcceleratorReservationManager::printReservationStats()
{
    std::lock_guard<std::mutex> lock(reservationMutex);
    
    const char* accelNames[] = {"TrigAccel", "MatAccel", "VPU", "NPU"};
    
    warn("=== Accelerator Reservation Status ===\n");
    for (int i = 0; i < 5; i++) {
        int total = availableAccels[i];
        int activeReservations = 0;
        Tick now = curTick();
        
        for (const auto& res : reservations[i]) {
            if (res.reservationEnd > now) {
                activeReservations++;
            }
        }
        
        warn("  %s: %d total, %d reserved, %d available\n",
             accelNames[i], total, activeReservations, total - activeReservations);
    }
}

// ============================================================================
// Task Queue Implementation
// ============================================================================

TaskQueue::TaskQueue(SchedulingPolicy pol)
    : policy(pol), currentPhase(NOVAMissionPhase::IDLE)
{
}

void
TaskQueue::enqueue(TaskDescriptor* task)
{
    tasks.push_back(task);
    sortQueue();
}

TaskDescriptor*
TaskQueue::dequeue()
{
    if (tasks.empty()) return nullptr;
    
    sortQueue();
    TaskDescriptor* task = tasks.front();
    tasks.erase(tasks.begin());
    return task;
}

TaskDescriptor*
TaskQueue::peek()
{
    if (tasks.empty()) return nullptr;
    sortQueue();
    return tasks.front();
}

bool
TaskQueue::removeTask(TaskDescriptor* task)
{
    auto it = std::find(tasks.begin(), tasks.end(), task);
    if (it != tasks.end()) {
        tasks.erase(it);
        return true;
    }
    return false;
}

void
TaskQueue::updatePriorities(Tick currentTime, NOVAMissionPhase phase)
{
    currentPhase = phase;
    
    // Apply priority aging for waiting tasks
    for (auto* task : tasks) {
        Tick waitTime = currentTime - task->arrivalTime;
        
        // Increase dynamic priority based on wait time (aging)
        int agingBonus = static_cast<int>(waitTime / 1000000);  // Per million ticks
        task->dynamicPriority = task->staticPriority + agingBonus;
        
        // Boost priority if deadline is approaching
        if (task->deadline > 0) {
            Tick timeToDeadline = task->deadline - currentTime;
            if (timeToDeadline < task->estimatedExecutionTime * 2) {
                task->dynamicPriority += 50;  // Urgent boost
            }
        }
    }
    
    sortQueue();
}

void
TaskQueue::sortQueue()
{
    switch (policy) {
    case SchedulingPolicy::POLICY_FIFO:
        // Already in FIFO order
        break;
        
    case SchedulingPolicy::POLICY_PRIORITY:
        std::sort(tasks.begin(), tasks.end(),
                 [this](TaskDescriptor* t1, TaskDescriptor* t2) {
                     return comparePriority(t1, t2);
                 });
        break;
        
    case SchedulingPolicy::POLICY_EDF:
        std::sort(tasks.begin(), tasks.end(),
                 [this](TaskDescriptor* t1, TaskDescriptor* t2) {
                     return compareEDF(t1, t2);
                 });
        break;
        
    case SchedulingPolicy::POLICY_RATE_MONOTONIC:
        std::sort(tasks.begin(), tasks.end(),
                 [this](TaskDescriptor* t1, TaskDescriptor* t2) {
                     return compareRateMonotonic(t1, t2);
                 });
        break;
        
    case SchedulingPolicy::POLICY_PHASE_AWARE:
        std::sort(tasks.begin(), tasks.end(),
                 [this](TaskDescriptor* t1, TaskDescriptor* t2) {
                     return comparePhaseAware(t1, t2);
                 });
        break;
        
    case SchedulingPolicy::POLICY_HYBRID:
        // Combine multiple policies
        std::sort(tasks.begin(), tasks.end(),
                 [this](TaskDescriptor* t1, TaskDescriptor* t2) {
                     // First check phase compatibility
                     bool t1Compat = t1->compatiblePhases.count(currentPhase) > 0;
                     bool t2Compat = t2->compatiblePhases.count(currentPhase) > 0;
                     if (t1Compat != t2Compat) return t1Compat;
                     
                     // Then check criticality
                     if (t1->criticality != t2->criticality)
                         return t1->criticality < t2->criticality;
                     
                     // Finally use dynamic priority
                     return t1->dynamicPriority > t2->dynamicPriority;
                 });
        break;
        
    default:
        break;
    }
}

bool
TaskQueue::compareFIFO(TaskDescriptor* t1, TaskDescriptor* t2)
{
    return t1->arrivalTime < t2->arrivalTime;
}

bool
TaskQueue::comparePriority(TaskDescriptor* t1, TaskDescriptor* t2)
{
    // Higher priority first
    if (t1->dynamicPriority != t2->dynamicPriority) {
        return t1->dynamicPriority > t2->dynamicPriority;
    }
    // If same priority, higher criticality first
    return t1->criticality < t2->criticality;  // Lower enum value = higher criticality
}

bool
TaskQueue::compareEDF(TaskDescriptor* t1, TaskDescriptor* t2)
{
    // Earlier deadline first
    if (t1->deadline == 0) return false;  // No deadline = lowest priority
    if (t2->deadline == 0) return true;
    return t1->deadline < t2->deadline;
}

bool
TaskQueue::compareRateMonotonic(TaskDescriptor* t1, TaskDescriptor* t2)
{
    // Shorter period = higher priority
    if (t1->period == 0) return false;
    if (t2->period == 0) return true;
    return t1->period < t2->period;
}

bool
TaskQueue::comparePhaseAware(TaskDescriptor* t1, TaskDescriptor* t2)
{
    // Prefer tasks compatible with current phase
    bool t1Compatible = t1->compatiblePhases.count(currentPhase) > 0;
    bool t2Compatible = t2->compatiblePhases.count(currentPhase) > 0;
    
    if (t1Compatible != t2Compatible) {
        return t1Compatible;  // Compatible tasks first
    }
    
    // Among compatible tasks, use priority
    return comparePriority(t1, t2);
}

// ============================================================================
// Mission Phase Scheduler Implementation
// ============================================================================

const char*
MissionPhaseScheduler::getPhaseName(NOVAMissionPhase phase)
{
    switch (phase) {
        case NOVAMissionPhase::LAUNCH: return "LAUNCH";
        case NOVAMissionPhase::CRUISE: return "CRUISE";
        case NOVAMissionPhase::APPROACH: return "APPROACH";
        case NOVAMissionPhase::LANDING: return "LANDING";
        case NOVAMissionPhase::SURFACE_OPS: return "SURFACE_OPS";
        case NOVAMissionPhase::ECLIPSE: return "ECLIPSE";
        case NOVAMissionPhase::SAFE_MODE: return "SAFE_MODE";
        case NOVAMissionPhase::IDLE: return "IDLE";
        default: return "UNKNOWN";
    }
}

MissionPhaseScheduler::MissionPhaseScheduler(int numCPUs, bool globalQ)
    : currentPolicy(SchedulingPolicy::POLICY_HYBRID),
      currentPhase(NOVAMissionPhase::IDLE),
      useGlobalQueue(globalQ)
{
    if (!useGlobalQueue) {
        perCPUQueues.resize(numCPUs);
    }
    
    // Initialize phase-specific policies
    phasePolicies[NOVAMissionPhase::LAUNCH] = SchedulingPolicy::POLICY_PRIORITY;
    phasePolicies[NOVAMissionPhase::CRUISE] = SchedulingPolicy::POLICY_FIFO;
    phasePolicies[NOVAMissionPhase::APPROACH] = SchedulingPolicy::POLICY_EDF;
    phasePolicies[NOVAMissionPhase::LANDING] = SchedulingPolicy::POLICY_PRIORITY;
    phasePolicies[NOVAMissionPhase::SURFACE_OPS] = SchedulingPolicy::POLICY_PHASE_AWARE;
    phasePolicies[NOVAMissionPhase::ECLIPSE] = SchedulingPolicy::POLICY_POWER_AWARE;
    phasePolicies[NOVAMissionPhase::SAFE_MODE] = SchedulingPolicy::POLICY_PRIORITY;
    phasePolicies[NOVAMissionPhase::IDLE] = SchedulingPolicy::POLICY_FIFO;
    
    // Initialize phase-specific accelerator configurations
    phaseAccelConfigs[NOVAMissionPhase::LAUNCH] = {4, 4, 2, 1, 1, "launch", 0, 0, 0};
    phaseAccelConfigs[NOVAMissionPhase::CRUISE] = {1, 1, 0, 0, 0, "cruise", 0, 0, 0};
    phaseAccelConfigs[NOVAMissionPhase::APPROACH] = {3, 3, 2, 2, 1, "approach", 0, 0, 0};
    phaseAccelConfigs[NOVAMissionPhase::LANDING] = {4, 4, 1, 1, 1, "landing", 0, 0, 0};
    phaseAccelConfigs[NOVAMissionPhase::SURFACE_OPS] = {2, 3, 2, 2, 1, "surface", 0, 0, 0};
    phaseAccelConfigs[NOVAMissionPhase::ECLIPSE] = {2, 2, 1, 0, 0, "eclipse", 0, 0, 0};
    phaseAccelConfigs[NOVAMissionPhase::SAFE_MODE] = {1, 1, 0, 0, 0, "safe", 0, 0, 0};
    phaseAccelConfigs[NOVAMissionPhase::IDLE] = {1, 1, 1, 1, 1, "idle", 0, 0, 0};
}

MissionPhaseScheduler::~MissionPhaseScheduler()
{
    // Clean up tasks
    for (auto task : completedTasks) {
        delete task;
    }
    for (auto task : waitingTasks) {
        delete task;
    }
}

void
MissionPhaseScheduler::initialize(const NOVAAcceleratorConfig& initialConfig)
{
    reservationManager.setAvailableAccelerators(
        initialConfig.numTrig,
        initialConfig.numMatrix,
        initialConfig.numVPU,
        initialConfig.numNPU,
        initialConfig.numGPU
    );
    
    warn("NOVA Scheduler: Initialized with %d Trig, %d Matrix, %d VPU, %d NPU\n",
         initialConfig.numTrig, initialConfig.numMatrix,
         initialConfig.numVPU, initialConfig.numNPU);
}

void
MissionPhaseScheduler::submitTask(TaskDescriptor* task)
{
    task->arrivalTime = curTick();
    task->isWaiting = true;
    task->isExecuting = false;
    task->isCompleted = false;
    
    // Update statistics
    stats.totalTasksScheduled++;
    stats.tasksPerCriticality[task->criticality]++;
    stats.tasksPerPhase[currentPhase]++;
    
    // Add to appropriate queue
    if (useGlobalQueue) {
        globalQueue.enqueue(task);
    } else if (task->cpuID >= 0 && task->cpuID < (int)perCPUQueues.size()) {
        perCPUQueues[task->cpuID].enqueue(task);
    } else {
        globalQueue.enqueue(task);
    }
    
    warn("NOVA Scheduler: Task %s submitted (criticality=%d, deadline=%lu)\n",
         task->taskName.c_str(), static_cast<int>(task->criticality), task->deadline);
}

TaskDescriptor*
MissionPhaseScheduler::getNextTask(int cpuID)
{
    TaskDescriptor* task = nullptr;
    
    if (useGlobalQueue) {
        task = globalQueue.dequeue();
    } else if (cpuID >= 0 && cpuID < (int)perCPUQueues.size()) {
        task = perCPUQueues[cpuID].dequeue();
    }
    
    if (task) {
        task->isWaiting = false;
        task->isExecuting = true;
        task->startTime = curTick();
        task->cpuID = cpuID;
        activeTasks[cpuID] = task;
        
        warn("NOVA Scheduler: Task %s scheduled on CPU %d\n",
             task->taskName.c_str(), cpuID);
    }
    
    return task;
}

void
MissionPhaseScheduler::completeTask(TaskDescriptor* task)
{
    task->isExecuting = false;
    task->isCompleted = true;
    task->completionTime = curTick();
    task->actualExecutionTime = task->completionTime - task->startTime;
    
    // Check deadline
    if (task->deadline > 0 && task->completionTime > task->deadline) {
        task->missedDeadline = true;
        stats.totalDeadlinesMissed++;
        stats.deadlineMissesPerCriticality[task->criticality]++;
        
        warn("NOVA Scheduler: Task %s MISSED DEADLINE (completed at %lu, deadline was %lu)\n",
             task->taskName.c_str(), task->completionTime, task->deadline);
    } else {
        warn("NOVA Scheduler: Task %s completed successfully in %lu ticks\n", 
             task->taskName.c_str(), task->actualExecutionTime);
    }
    
    // Release accelerators
    for (const auto& req : task->accelRequirements) {
        reservationManager.releaseAccelerator(req.first, task);
    }
    
    // Update statistics
    stats.totalTasksCompleted++;
    stats.executionTimePerPhase[currentPhase] += task->actualExecutionTime;
    
    // Move to completed list
    activeTasks.erase(task->cpuID);
    completedTasks.push_back(task);
}

void
MissionPhaseScheduler::schedule(Tick currentTime)
{
    // Update priorities based on current time and phase
    if (useGlobalQueue) {
        globalQueue.updatePriorities(currentTime, currentPhase);
    } else {
        for (auto& queue : perCPUQueues) {
            queue.updatePriorities(currentTime, currentPhase);
        }
    }
    
    // Check waiting tasks to see if accelerators are now available
    auto it = waitingTasks.begin();
    while (it != waitingTasks.end()) {
        TaskDescriptor* task = *it;
        if (canAcquireAccelerators(task)) {
            task->isWaiting = false;
            task->waitingForAccelType = -1;
            
            // Re-queue the task
            if (useGlobalQueue) {
                globalQueue.enqueue(task);
            } else if (task->cpuID >= 0 && task->cpuID < (int)perCPUQueues.size()) {
                perCPUQueues[task->cpuID].enqueue(task);
            }
            
            it = waitingTasks.erase(it);
        } else {
            ++it;
        }
    }
}

void
MissionPhaseScheduler::reschedule(Tick currentTime)
{
    stats.totalContextSwitches++;
    schedule(currentTime);
}

void
MissionPhaseScheduler::transitionToPhase(NOVAMissionPhase newPhase, Tick currentTime)
{
    if (newPhase == currentPhase) return;
    
    warn("NOVA Scheduler: Phase transition %s -> %s at tick %lu\n",
         getPhaseName(currentPhase), getPhaseName(newPhase), currentTime);
    
    currentPhase = newPhase;
    
    // Update scheduling policy for new phase
    if (phasePolicies.count(newPhase) > 0) {
        currentPolicy = phasePolicies[newPhase];
    }
    
    if (useGlobalQueue) {
        globalQueue.setPolicy(currentPolicy);
        globalQueue.setPhase(newPhase);
    } else {
        for (auto& queue : perCPUQueues) {
            queue.setPolicy(currentPolicy);
            queue.setPhase(newPhase);
        }
    }
    
    // Reconfigure accelerators if needed
    if (phaseAccelConfigs.count(newPhase) > 0) {
        const NOVAAcceleratorConfig& config = phaseAccelConfigs[newPhase];
        reservationManager.setAvailableAccelerators(
            config.numTrig, config.numMatrix, config.numVPU, config.numNPU,
            config.numGPU
        );
        
        warn("NOVA Scheduler: Reconfigured accelerators for %s: "
             "%d Trig, %d Matrix, %d VPU, %d NPU\n",
             getPhaseName(newPhase), config.numTrig, config.numMatrix, 
             config.numVPU, config.numNPU);
    }
}

void
MissionPhaseScheduler::updatePhasePolicy(NOVAMissionPhase phase, SchedulingPolicy policy)
{
    phasePolicies[phase] = policy;
}

void
MissionPhaseScheduler::updatePhaseAccelConfig(NOVAMissionPhase phase, 
                                              const NOVAAcceleratorConfig& config)
{
    phaseAccelConfigs[phase] = config;
}

bool
MissionPhaseScheduler::requestAccelerator(TaskDescriptor* task, int accelType)
{
    Tick duration = task->accelHoldTime;
    if (duration == 0) {
        duration = task->estimatedExecutionTime;
    }
    
    bool reserved = reservationManager.reserveAccelerator(
        accelType, task, duration, task->needsExclusiveAccess
    );
    
    if (!reserved) {
        task->isWaiting = true;
        task->waitingForAccelType = accelType;
        waitingTasks.push_back(task);
        stats.totalAccelWaitTime++;
    }
    
    return reserved;
}

void
MissionPhaseScheduler::releaseAccelerator(TaskDescriptor* task, int accelType)
{
    reservationManager.releaseAccelerator(accelType, task);
}

bool
MissionPhaseScheduler::canAcquireAccelerators(TaskDescriptor* task)
{
    Tick currentTime = curTick();
    
    for (const auto& req : task->accelRequirements) {
        int available = reservationManager.getAvailableCount(req.first, currentTime);
        if (available < req.second) {
            return false;
        }
    }
    
    return true;
}

bool
MissionPhaseScheduler::admitTask(TaskDescriptor* task)
{
    // Check if admitting this task would violate schedulability
    std::vector<TaskDescriptor*> testSet;
    
    // Add active and queued tasks
    for (const auto& pair : activeTasks) {
        testSet.push_back(pair.second);
    }
    
    // Add the new task
    testSet.push_back(task);
    
    // Test schedulability
    return isSchedulable(testSet);
}

double
MissionPhaseScheduler::calculateCPUUtilization()
{
    return calculateUtilization(completedTasks);
}

bool
MissionPhaseScheduler::isSchedulable(const std::vector<TaskDescriptor*>& taskSet)
{
    // Use both RM and EDF tests
    return rateMonotonicTest(taskSet) || edfSchedulabilityTest(taskSet);
}

// ============================================================================
// Real-Time Schedulability Analysis
// ============================================================================

bool
MissionPhaseScheduler::rateMonotonicTest(const std::vector<TaskDescriptor*>& taskSet)
{
    // Liu & Layland utilization bound test
    double utilization = calculateUtilization(taskSet);
    size_t n = taskSet.size();
    if (n == 0) return true;
    
    double bound = n * (std::pow(2.0, 1.0/n) - 1.0);
    
    return utilization <= bound;
}

double
MissionPhaseScheduler::calculateUtilization(const std::vector<TaskDescriptor*>& taskSet)
{
    double util = 0.0;
    for (const auto* task : taskSet) {
        if (task->period > 0) {
            util += (double)task->estimatedExecutionTime / task->period;
        }
    }
    return util;
}

bool
MissionPhaseScheduler::edfSchedulabilityTest(const std::vector<TaskDescriptor*>& taskSet)
{
    // EDF schedulability: utilization <= 1.0
    return calculateUtilization(taskSet) <= 1.0;
}

Tick
MissionPhaseScheduler::calculateWCRT(TaskDescriptor* task)
{
    // Simplified WCRT calculation
    // In real implementation, would consider interference from higher priority tasks
    
    Tick wcrt = task->estimatedExecutionTime;
    
    // Add accelerator wait time estimate
    for (const auto& req : task->accelRequirements) {
        int accelType = req.first;
        int available = reservationManager.getAvailableCount(accelType, curTick());
        
        if (available < req.second) {
            // Estimate wait time based on contention
            Tick avgWaitTime = task->estimatedExecutionTime / 2;  // Simplified
            wcrt += avgWaitTime;
        }
    }
    
    return wcrt;
}

NOVAAcceleratorConfig
MissionPhaseScheduler::optimizeAccelConfig(const std::vector<TaskDescriptor*>& taskSet,
                                           NOVAMissionPhase phase)
{
    NOVAAcceleratorConfig config = {1, 1, 1, 1, 1, "optimized", 0, 0, 0};
    
    // Count accelerator requirements from task set
    std::map<int, int> totalRequirements;
    for (const auto* task : taskSet) {
        for (const auto& req : task->accelRequirements) {
            totalRequirements[req.first] = std::max(totalRequirements[req.first], req.second);
        }
    }
    
    // Allocate based on demand (with upper limit)
    config.numTrig = std::min(totalRequirements[0] + 1, 4);
    config.numMatrix = std::min(totalRequirements[1] + 1, 4);
    config.numVPU = std::min(totalRequirements[2], 3);
    config.numNPU = std::min(totalRequirements[3], 3);
    
    return config;
}

void
MissionPhaseScheduler::balanceLoad()
{
    // Simple load balancing: move tasks from overloaded to underloaded CPUs
    if (useGlobalQueue || perCPUQueues.size() <= 1) return;
    
    size_t avgLoad = 0;
    for (const auto& queue : perCPUQueues) {
        avgLoad += queue.size();
    }
    avgLoad /= perCPUQueues.size();
    
    // Find overloaded and underloaded CPUs
    for (size_t i = 0; i < perCPUQueues.size(); i++) {
        if (perCPUQueues[i].size() > avgLoad + 1) {
            // Find an underloaded CPU
            for (size_t j = 0; j < perCPUQueues.size(); j++) {
                if (i != j && perCPUQueues[j].size() < avgLoad) {
                    // Migrate a task
                    TaskDescriptor* task = perCPUQueues[i].dequeue();
                    if (task) {
                        perCPUQueues[j].enqueue(task);
                        stats.totalContextSwitches++;
                    }
                    break;
                }
            }
        }
    }
}

void
MissionPhaseScheduler::migrateTask(TaskDescriptor* task, int fromCPU, int toCPU)
{
    if (useGlobalQueue) return;
    if (fromCPU < 0 || fromCPU >= (int)perCPUQueues.size()) return;
    if (toCPU < 0 || toCPU >= (int)perCPUQueues.size()) return;
    
    if (perCPUQueues[fromCPU].removeTask(task)) {
        task->cpuID = toCPU;
        perCPUQueues[toCPU].enqueue(task);
        stats.totalContextSwitches++;
    }
}

// ============================================================================
// Statistics and Reporting
// ============================================================================

void
MissionPhaseScheduler::printSchedulerStats()
{
    warn("\n============ NOVA Scheduler Statistics ============\n");
    warn("Total Tasks: scheduled=%lu, completed=%lu\n",
         stats.totalTasksScheduled, stats.totalTasksCompleted);
    warn("Deadlines: missed=%lu (%.1f%%)\n",
         stats.totalDeadlinesMissed,
         stats.totalTasksCompleted > 0 ? 
         (double)stats.totalDeadlinesMissed / stats.totalTasksCompleted * 100.0 : 0.0);
    warn("Preemptions: %lu\n", stats.totalPreemptions);
    warn("Context Switches: %lu\n", stats.totalContextSwitches);
    warn("Accelerator Wait Events: %lu\n", stats.totalAccelWaitTime);
    
    warn("\nTasks by Criticality:\n");
    for (int i = 0; i <= static_cast<int>(TaskCriticality::BEST_EFFORT); i++) {
        TaskCriticality crit = static_cast<TaskCriticality>(i);
        uint64_t count = stats.tasksPerCriticality[crit];
        uint64_t misses = stats.deadlineMissesPerCriticality[crit];
        if (count > 0) {
            warn("  %s: %lu tasks, %lu deadline misses (%.1f%%)\n",
                 getCriticalityName(crit), count, misses,
                 (double)misses / count * 100.0);
        }
    }
    
    warn("\nTasks by Phase:\n");
    for (int i = 0; i <= static_cast<int>(NOVAMissionPhase::IDLE); i++) {
        NOVAMissionPhase phase = static_cast<NOVAMissionPhase>(i);
        uint64_t count = stats.tasksPerPhase[phase];
        Tick time = stats.executionTimePerPhase[phase];
        if (count > 0) {
            warn("  %s: %lu tasks, %lu ticks\n",
                 getPhaseName(phase), count, time);
        }
    }
    
    warn("===================================================\n");
}

void
MissionPhaseScheduler::exportSchedulingTrace(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        warn("NOVA Scheduler: Failed to open trace file: %s\n", filename.c_str());
        return;
    }
    
    // Header
    file << "taskID,taskName,criticality,arrivalTime,startTime,completionTime,"
         << "deadline,missedDeadline,waitTime,executionTime,cpuID\n";
    
    // Export completed tasks
    for (const auto* task : completedTasks) {
        file << task->taskID << ","
             << task->taskName << ","
             << static_cast<int>(task->criticality) << ","
             << task->arrivalTime << ","
             << task->startTime << ","
             << task->completionTime << ","
             << task->deadline << ","
             << (task->missedDeadline ? 1 : 0) << ","
             << (task->startTime - task->arrivalTime) << ","
             << task->actualExecutionTime << ","
             << task->cpuID << "\n";
    }
    
    file.close();
    warn("NOVA Scheduler: Trace exported to %s\n", filename.c_str());
}

void
MissionPhaseScheduler::analyzeSchedulability()
{
    warn("\n=== Schedulability Analysis ===\n");
    
    // Collect all periodic tasks
    std::vector<TaskDescriptor*> periodicTasks;
    for (auto* task : completedTasks) {
        if (task->period > 0) {
            periodicTasks.push_back(task);
        }
    }
    
    if (periodicTasks.empty()) {
        warn("No periodic tasks to analyze\n");
        return;
    }
    
    double utilization = calculateUtilization(periodicTasks);
    bool rmSchedulable = rateMonotonicTest(periodicTasks);
    bool edfSchedulable = edfSchedulabilityTest(periodicTasks);
    
    warn("Periodic Tasks: %zu\n", periodicTasks.size());
    warn("Total Utilization: %.4f\n", utilization);
    warn("Rate Monotonic Schedulable: %s\n", rmSchedulable ? "YES" : "NO");
    warn("EDF Schedulable: %s\n", edfSchedulable ? "YES" : "NO");
    warn("================================\n");
}

// ============================================================================
// Accelerator Optimizer Implementation
// ============================================================================

AcceleratorOptimizer::AcceleratorOptimizer()
    : maxPowerBudget(50.0),
      maxAreaBudget(100.0),
      maxTotalAccelerators(16),
      objective(OptimizationObjective::MAXIMIZE_SCHEDULABILITY)
{
}

void
AcceleratorOptimizer::setTaskSet(const std::vector<TaskDescriptor*>& tasks)
{
    taskSet = tasks;
    
    // Organize by phase
    tasksPerPhase.clear();
    for (auto* task : tasks) {
        for (auto phase : task->compatiblePhases) {
            tasksPerPhase[phase].push_back(task);
        }
    }
}

void
AcceleratorOptimizer::setConstraints(double maxPower, double maxArea, int maxAccels)
{
    maxPowerBudget = maxPower;
    maxAreaBudget = maxArea;
    maxTotalAccelerators = maxAccels;
}

NOVAAcceleratorConfig
AcceleratorOptimizer::optimizeGreedy(NOVAMissionPhase phase)
{
    // Greedy algorithm: add accelerators where they help most
    
    NOVAAcceleratorConfig config = {0, 0, 0, 0, 0, "Greedy", 0.0, 0.0, 0.0};
    
    if (tasksPerPhase.count(phase) == 0) {
        config = {1, 1, 1, 1, 1, "Greedy-Default", 0.0, 0.0, 0.0};
        return config;
    }
    
    const auto& phaseTasks = tasksPerPhase[phase];
    
    // Count accelerator requirements
    std::map<int, int> totalRequirements;
    for (const auto* task : phaseTasks) {
        for (const auto& req : task->accelRequirements) {
            totalRequirements[req.first] += req.second;
        }
    }
    
    // Allocate based on demand
    config.numTrig = std::min(std::max(totalRequirements[0], 1), 4);
    config.numMatrix = std::min(std::max(totalRequirements[1], 1), 4);
    config.numVPU = std::min(totalRequirements[2], 3);
    config.numNPU = std::min(totalRequirements[3], 3);
    
    // Estimate power and area
    config.estimatedPower = estimatePower(config);
    config.estimatedArea = estimateArea(config);
    
    return config;
}

NOVAAcceleratorConfig
AcceleratorOptimizer::optimizeILP(NOVAMissionPhase phase)
{
    // Placeholder for ILP-based optimization
    // Would require an external ILP solver
    return optimizeGreedy(phase);
}

NOVAAcceleratorConfig
AcceleratorOptimizer::optimizeGenetic(NOVAMissionPhase phase)
{
    // Placeholder for genetic algorithm optimization
    return optimizeGreedy(phase);
}

NOVAAcceleratorConfig
AcceleratorOptimizer::findMinimalSchedulableConfig(NOVAMissionPhase phase)
{
    // Binary search for minimal config that achieves schedulability
    
    NOVAAcceleratorConfig config = {1, 1, 0, 0, 0, "Minimal", 0.0, 0.0, 0.0};
    
    if (tasksPerPhase.count(phase) == 0) {
        return config;
    }
    
    const auto& phaseTasks = tasksPerPhase[phase];
    
    // Start with minimal and incrementally add
    for (int trig = 1; trig <= 4; trig++) {
        for (int mat = 1; mat <= 4; mat++) {
            config.numTrig = trig;
            config.numMatrix = mat;
            
            double execTime = estimateExecutionTime(config, phaseTasks);
            if (execTime < 1.0) {  // Schedulable
                config.configName = "Minimal-Found";
                config.estimatedPower = estimatePower(config);
                config.estimatedArea = estimateArea(config);
                return config;
            }
        }
    }
    
    // Return greedy if minimal not found
    return optimizeGreedy(phase);
}

std::vector<NOVAAcceleratorConfig>
AcceleratorOptimizer::findParetoFrontier(NOVAMissionPhase phase)
{
    std::vector<NOVAAcceleratorConfig> frontier;
    
    // Generate candidate configurations
    std::vector<NOVAAcceleratorConfig> candidates;
    for (int t = 1; t <= 4; t++) {
        for (int m = 1; m <= 4; m++) {
            for (int v = 0; v <= 2; v++) {
                for (int n = 0; n <= 2; n++) {
                    NOVAAcceleratorConfig cfg = {t, m, v, n, 0, "Candidate", 0, 0, 0};
                    cfg.estimatedPower = estimatePower(cfg);
                    cfg.estimatedArea = estimateArea(cfg);
                    
                    if (tasksPerPhase.count(phase) > 0) {
                        cfg.estimatedLatency = estimateExecutionTime(cfg, tasksPerPhase[phase]);
                    }
                    
                    candidates.push_back(cfg);
                }
            }
        }
    }
    
    // Find Pareto-optimal configurations
    for (const auto& cfg : candidates) {
        bool isDominated = false;
        
        for (const auto& other : candidates) {
            // Check if 'other' dominates 'cfg'
            bool otherBetterOrEqual = 
                (other.estimatedPower <= cfg.estimatedPower) &&
                (other.estimatedArea <= cfg.estimatedArea) &&
                (other.estimatedLatency <= cfg.estimatedLatency);
            bool otherStrictlyBetter = 
                (other.estimatedPower < cfg.estimatedPower) ||
                (other.estimatedArea < cfg.estimatedArea) ||
                (other.estimatedLatency < cfg.estimatedLatency);
            
            if (otherBetterOrEqual && otherStrictlyBetter) {
                isDominated = true;
                break;
            }
        }
        
        if (!isDominated) {
            frontier.push_back(cfg);
        }
    }
    
    return frontier;
}

double
AcceleratorOptimizer::estimateExecutionTime(const NOVAAcceleratorConfig& config,
                                           const std::vector<TaskDescriptor*>& tasks)
{
    double totalTime = 0.0;
    
    for (const auto* task : tasks) {
        Tick taskTime = task->estimatedExecutionTime;
        
        // Adjust based on accelerator availability
        for (const auto& req : task->accelRequirements) {
            int accelType = req.first;
            int needed = req.second;
            int available = 0;
            
            switch (accelType) {
                case 0: available = config.numTrig; break;
                case 1: available = config.numMatrix; break;
                case 2: available = config.numVPU; break;
                case 3: available = config.numNPU; break;
            }
            
            if (available < needed) {
                // Will experience contention
                taskTime *= 1.5;  // Simplified penalty
            }
        }
        
        totalTime += taskTime;
    }
    
    return totalTime;
}

double
AcceleratorOptimizer::estimatePower(const NOVAAcceleratorConfig& config)
{
    // Power estimates in mW
    double power = 0.0;
    power += config.numTrig * 150.0;    // Trig accelerator
    power += config.numMatrix * 200.0;  // Matrix accelerator
    power += config.numVPU * 350.0;     // VPU
    power += config.numNPU * 500.0;     // NPU
    return power;
}

double
AcceleratorOptimizer::estimateArea(const NOVAAcceleratorConfig& config)
{
    // Relative area units
    double area = 0.0;
    area += config.numTrig * 1.0;
    area += config.numMatrix * 1.2;
    area += config.numVPU * 2.0;
    area += config.numNPU * 3.0;
    return area;
}

void
AcceleratorOptimizer::sensitivityAnalysis(const NOVAAcceleratorConfig& baseConfig)
{
    warn("\n=== Sensitivity Analysis ===\n");
    warn("Base Configuration: %d Trig, %d Mat, %d VPU, %d NPU\n",
         baseConfig.numTrig, baseConfig.numMatrix, 
         baseConfig.numVPU, baseConfig.numNPU);
    warn("Base Power: %.1f mW, Area: %.1f units\n",
         estimatePower(baseConfig), estimateArea(baseConfig));
    
    // Test variations
    const char* accelNames[] = {"Trig", "Matrix", "VPU", "NPU"};
    int* counts[] = {nullptr, nullptr, nullptr, nullptr};  // Would need mutable config
    
    warn("\nSensitivity to accelerator count changes:\n");
    for (int delta = -1; delta <= 1; delta += 2) {
        NOVAAcceleratorConfig testConfig = baseConfig;
        
        // Test Trig variation
        testConfig.numTrig = std::max(1, baseConfig.numTrig + delta);
        warn("  Trig %+d: Power=%.1f, Area=%.1f\n", 
             delta, estimatePower(testConfig), estimateArea(testConfig));
        testConfig.numTrig = baseConfig.numTrig;
        
        // Test Matrix variation
        testConfig.numMatrix = std::max(1, baseConfig.numMatrix + delta);
        warn("  Matrix %+d: Power=%.1f, Area=%.1f\n",
             delta, estimatePower(testConfig), estimateArea(testConfig));
    }
    
    warn("=============================\n");
}

void
AcceleratorOptimizer::exportOptimizationResults(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        warn("AcceleratorOptimizer: Failed to open file: %s\n", filename.c_str());
        return;
    }
    
    file << "phase,numTrig,numMatrix,numVPU,numNPU,power,area,latency\n";
    
    for (int p = 0; p <= static_cast<int>(NOVAMissionPhase::IDLE); p++) {
        NOVAMissionPhase phase = static_cast<NOVAMissionPhase>(p);
        NOVAAcceleratorConfig config = optimizeGreedy(phase);
        
        file << p << ","
             << config.numTrig << ","
             << config.numMatrix << ","
             << config.numVPU << ","
             << config.numNPU << ","
             << config.estimatedPower << ","
             << config.estimatedArea << ","
             << config.estimatedLatency << "\n";
    }
    
    file.close();
    warn("AcceleratorOptimizer: Results exported to %s\n", filename.c_str());
}

// ============================================================================
// Scheduling Policy Implementations
// ============================================================================

void
SchedulingPolicyManager::applyRateMonotonic(std::vector<TaskDescriptor*>& tasks)
{
    std::sort(tasks.begin(), tasks.end(),
             [](TaskDescriptor* t1, TaskDescriptor* t2) {
                 if (t1->period == 0) return false;
                 if (t2->period == 0) return true;
                 return t1->period < t2->period;
             });
    
    // Assign priorities based on period (shorter = higher priority)
    int priority = 100;
    for (auto* task : tasks) {
        task->dynamicPriority = priority--;
    }
}

void
SchedulingPolicyManager::applyEDF(std::vector<TaskDescriptor*>& tasks, Tick currentTime)
{
    std::sort(tasks.begin(), tasks.end(),
             [](TaskDescriptor* t1, TaskDescriptor* t2) {
                 if (t1->deadline == 0) return false;
                 if (t2->deadline == 0) return true;
                 return t1->deadline < t2->deadline;
             });
}

void
SchedulingPolicyManager::applyPriorityWithAging(std::vector<TaskDescriptor*>& tasks,
                                                Tick currentTime)
{
    for (auto* task : tasks) {
        Tick waitTime = currentTime - task->arrivalTime;
        int agingBonus = static_cast<int>(waitTime / 1000000);
        task->dynamicPriority = task->staticPriority + agingBonus;
    }
    
    std::sort(tasks.begin(), tasks.end(),
             [](TaskDescriptor* t1, TaskDescriptor* t2) {
                 return t1->dynamicPriority > t2->dynamicPriority;
             });
}

void
SchedulingPolicyManager::applyPhaseAware(std::vector<TaskDescriptor*>& tasks,
                                         NOVAMissionPhase phase)
{
    std::sort(tasks.begin(), tasks.end(),
             [phase](TaskDescriptor* t1, TaskDescriptor* t2) {
                 bool t1Compat = t1->compatiblePhases.count(phase) > 0;
                 bool t2Compat = t2->compatiblePhases.count(phase) > 0;
                 if (t1Compat != t2Compat) return t1Compat;
                 return t1->dynamicPriority > t2->dynamicPriority;
             });
}

void
SchedulingPolicyManager::applyPowerAware(std::vector<TaskDescriptor*>& tasks,
                                         double availablePower)
{
    // Sort by power efficiency (lower power requirement first for limited budget)
    std::sort(tasks.begin(), tasks.end(),
             [](TaskDescriptor* t1, TaskDescriptor* t2) {
                 return t1->powerBudget < t2->powerBudget;
             });
}

void
SchedulingPolicyManager::applyHybrid(std::vector<TaskDescriptor*>& tasks,
                                     NOVAMissionPhase phase, Tick currentTime,
                                     double availablePower)
{
    // Apply aging first
    applyPriorityWithAging(tasks, currentTime);
    
    // Then consider phase compatibility and power
    std::sort(tasks.begin(), tasks.end(),
             [phase, availablePower](TaskDescriptor* t1, TaskDescriptor* t2) {
                 // Phase compatibility
                 bool t1Compat = t1->compatiblePhases.count(phase) > 0;
                 bool t2Compat = t2->compatiblePhases.count(phase) > 0;
                 if (t1Compat != t2Compat) return t1Compat;
                 
                 // Criticality
                 if (t1->criticality != t2->criticality)
                     return t1->criticality < t2->criticality;
                 
                 // Power constraint
                 if (t1->powerBudget <= availablePower && t2->powerBudget > availablePower)
                     return true;
                 
                 // Dynamic priority
                 return t1->dynamicPriority > t2->dynamicPriority;
             });
}

// ============================================================================
// Helper Functions
// ============================================================================

void
printTaskInfo(const TaskDescriptor* task)
{
    warn("Task: %s (ID=%d)\n", task->taskName.c_str(), task->taskID);
    warn("  Criticality: %s, Priority: %d\n", 
         getCriticalityName(task->criticality), task->staticPriority);
    warn("  Deadline: %lu, WCET: %lu\n", 
         task->deadline, task->estimatedExecutionTime);
    warn("  Accelerator Requirements:");
    for (const auto& req : task->accelRequirements) {
        const char* accelNames[] = {"Trig", "Matrix", "VPU", "NPU"};
        if (req.first >= 0 && req.first < 4) {
            warn(" %s=%d", accelNames[req.first], req.second);
        }
    }
    warn("\n");
}

const char*
getCriticalityName(TaskCriticality crit)
{
    switch (crit) {
        case TaskCriticality::CRITICAL_SAFETY: return "SAFETY";
        case TaskCriticality::CRITICAL_NAVIGATION: return "NAVIGATION";
        case TaskCriticality::IMPORTANT_SCIENCE: return "SCIENCE";
        case TaskCriticality::NORMAL_TELEMETRY: return "TELEMETRY";
        case TaskCriticality::LOW_HOUSEKEEPING: return "HOUSEKEEPING";
        case TaskCriticality::BEST_EFFORT: return "BEST_EFFORT";
        default: return "UNKNOWN";
    }
}

const char*
getPolicyName(SchedulingPolicy policy)
{
    switch (policy) {
        case SchedulingPolicy::POLICY_FIFO: return "FIFO";
        case SchedulingPolicy::POLICY_PRIORITY: return "PRIORITY";
        case SchedulingPolicy::POLICY_EDF: return "EDF";
        case SchedulingPolicy::POLICY_RATE_MONOTONIC: return "RATE_MONOTONIC";
        case SchedulingPolicy::POLICY_PHASE_AWARE: return "PHASE_AWARE";
        case SchedulingPolicy::POLICY_POWER_AWARE: return "POWER_AWARE";
        case SchedulingPolicy::POLICY_HYBRID: return "HYBRID";
        default: return "UNKNOWN";
    }
}

} // namespace gem5

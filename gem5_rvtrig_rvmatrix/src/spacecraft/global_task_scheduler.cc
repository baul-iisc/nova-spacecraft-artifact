/*
 * NOVA Processor - Global Task Scheduler Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/global_task_scheduler.hh"
#include "base/trace.hh"
#include "debug/TaskScheduler.hh"

#include <algorithm>

namespace gem5
{

namespace spacecraft
{

GlobalTaskScheduler::GlobalTaskScheduler(const Params &p)
    : ClockedObject(p),
      numCores(p.num_cores),
      currentPolicy(SchedPolicy::MIXED_CRITICALITY_EDF),
      powerBudget(p.power_budget),
      contextSwitchOverhead(p.context_switch_overhead),
      schedulingOverhead(p.scheduling_overhead),
      readyQueues(p.num_cores),
      runningTasks(p.num_cores, nullptr),
      accelManager(nullptr),
      scheduleEvent([this]{ runScheduler(); }, name() + ".scheduleEvent"),
      periodicReleaseEvent([this]{ releasePeriodicTasks(); }, name() + ".periodicReleaseEvent"),
      deadlineCheckEvent([this]{ checkDeadlines(); }, name() + ".deadlineCheckEvent"),
      schedulingPeriod(p.scheduling_period)
{
    resetStats();
    
    // Schedule periodic events
    schedule(periodicReleaseEvent, curTick() + schedulingPeriod);
    schedule(deadlineCheckEvent, curTick() + schedulingPeriod * 10);
    
    DPRINTF(TaskScheduler, "Global Task Scheduler initialized: %d cores, policy=%d, power=%.1fW\n",
            numCores, static_cast<int>(currentPolicy), powerBudget);
}

GlobalTaskScheduler::~GlobalTaskScheduler()
{
}

void GlobalTaskScheduler::registerTask(const SpacecraftTask &task)
{
    tasks[task.id] = task;
    activeTaskIds.insert(task.id);
    
    DPRINTF(TaskScheduler, "Registered task %d: %s, criticality=%d, period=%lu, wcet=%lu\n",
            task.id, task.name.c_str(), static_cast<int>(task.criticality),
            task.period, task.wcet);
}

void GlobalTaskScheduler::unregisterTask(int taskId)
{
    tasks.erase(taskId);
    activeTaskIds.erase(taskId);
}

SpacecraftTask* GlobalTaskScheduler::getTask(int taskId)
{
    auto it = tasks.find(taskId);
    if (it != tasks.end()) {
        return &it->second;
    }
    return nullptr;
}

void GlobalTaskScheduler::runScheduler()
{
    Tick startTick = curTick();
    
    switch (currentPolicy) {
        case SchedPolicy::FIXED_PRIORITY_PREEMPTIVE:
            scheduleFixedPriority();
            break;
        case SchedPolicy::MIXED_CRITICALITY_EDF:
            scheduleMixedCriticalityEDF();
            break;
        case SchedPolicy::ENERGY_AWARE_EDF:
            scheduleEnergyAwareEDF();
            break;
        case SchedPolicy::MINIMAL_OPERATIONS:
            scheduleMinimal();
            break;
        case SchedPolicy::ROUND_ROBIN:
            scheduleRoundRobin();
            break;
    }
    
    stats.totalSchedulingOverhead += curTick() - startTick;
}

void GlobalTaskScheduler::setPolicy(SchedPolicy policy)
{
    if (policy != currentPolicy) {
        DPRINTF(TaskScheduler, "Scheduling policy changed: %d -> %d\n",
                static_cast<int>(currentPolicy), static_cast<int>(policy));
        currentPolicy = policy;
        runScheduler();
    }
}

void GlobalTaskScheduler::setPowerBudget(float watts)
{
    powerBudget = watts;
    DPRINTF(TaskScheduler, "Power budget set to %.1fW\n", watts);
    
    // Check if we need to shed tasks
    if (getCurrentPower() > powerBudget) {
        shedLowPriorityTasks();
    }
}

float GlobalTaskScheduler::getCurrentPower() const
{
    float total = 0.0f;
    for (int i = 0; i < numCores; i++) {
        if (runningTasks[i]) {
            total += runningTasks[i]->powerWatts;
        }
    }
    // Add accelerator power if manager is connected
    // (would call accelManager->getCurrentPower())
    return total;
}

void GlobalTaskScheduler::setActiveTaskSet(const std::set<int> &taskIds)
{
    activeTaskIds = taskIds;
    
    // Suspend tasks not in active set
    for (auto &[id, task] : tasks) {
        if (activeTaskIds.find(id) == activeTaskIds.end()) {
            if (task.state == TaskState::READY || task.state == TaskState::RUNNING) {
                task.state = TaskState::SUSPENDED;
            }
        } else {
            if (task.state == TaskState::SUSPENDED) {
                task.state = TaskState::READY;
            }
        }
    }
    
    runScheduler();
}

void GlobalTaskScheduler::suspendTask(int taskId)
{
    SpacecraftTask *task = getTask(taskId);
    if (task) {
        task->state = TaskState::SUSPENDED;
        activeTaskIds.erase(taskId);
        
        // If running, need to preempt
        for (int i = 0; i < numCores; i++) {
            if (runningTasks[i] == task) {
                preempt(task, i);
                break;
            }
        }
    }
}

void GlobalTaskScheduler::resumeTask(int taskId)
{
    SpacecraftTask *task = getTask(taskId);
    if (task && task->state == TaskState::SUSPENDED) {
        task->state = TaskState::READY;
        activeTaskIds.insert(taskId);
        
        // Add to ready queue
        int coreId = selectCoreForTask(task);
        readyQueues[coreId].push_back(task);
        
        runScheduler();
    }
}

bool GlobalTaskScheduler::isCoreIdle(int coreId) const
{
    return runningTasks[coreId] == nullptr;
}

SpacecraftTask* GlobalTaskScheduler::getRunningTask(int coreId)
{
    return runningTasks[coreId];
}

void GlobalTaskScheduler::checkDeadlines()
{
    Tick currentTime = curTick();
    
    for (auto &[id, task] : tasks) {
        if (task.state != TaskState::COMPLETED && 
            task.state != TaskState::SUSPENDED &&
            task.deadline > 0) {
            
            Tick absoluteDeadline = task.nextRelease + task.deadline;
            
            if (currentTime > absoluteDeadline && task.remainingTime > 0) {
                // Deadline miss!
                task.numDeadlineMisses++;
                stats.totalDeadlineMisses++;
                stats.missesPerCriticality[task.criticality]++;
                
                DPRINTF(TaskScheduler, "DEADLINE MISS: Task %s (criticality=%d) at tick %lu\n",
                        task.name.c_str(), static_cast<int>(task.criticality), currentTime);
                
                if (task.criticality == CriticalityLevel::MISSION_CRITICAL) {
                    stats.missionCriticalMisses++;
                    // In real system, this would trigger emergency handling
                    DPRINTF(TaskScheduler, "CRITICAL: Mission-critical task missed deadline!\n");
                }
            }
        }
    }
    
    // Reschedule deadline check
    schedule(deadlineCheckEvent, curTick() + schedulingPeriod * 10);
}

bool GlobalTaskScheduler::hasMissionCriticalMiss() const
{
    return stats.missionCriticalMisses > 0;
}

void GlobalTaskScheduler::resetStats()
{
    stats.totalContextSwitches = 0;
    stats.totalPreemptions = 0;
    stats.totalDeadlineMisses = 0;
    stats.missionCriticalMisses = 0;
    stats.tasksCompleted = 0;
    stats.tasksShed = 0;
    stats.totalSchedulingOverhead = 0;
    stats.perCoreUtilization.clear();
    stats.missesPerCriticality.clear();
}

void GlobalTaskScheduler::printStatistics()
{
    DPRINTF(TaskScheduler, "=== Scheduler Statistics ===\n");
    DPRINTF(TaskScheduler, "Context Switches: %lu\n", stats.totalContextSwitches);
    DPRINTF(TaskScheduler, "Preemptions: %lu\n", stats.totalPreemptions);
    DPRINTF(TaskScheduler, "Deadline Misses: %lu\n", stats.totalDeadlineMisses);
    DPRINTF(TaskScheduler, "Mission Critical Misses: %lu\n", stats.missionCriticalMisses);
    DPRINTF(TaskScheduler, "Tasks Completed: %lu\n", stats.tasksCompleted);
    DPRINTF(TaskScheduler, "Tasks Shed: %lu\n", stats.tasksShed);
}

void GlobalTaskScheduler::onTaskComplete(int taskId, int coreId)
{
    SpacecraftTask *task = getTask(taskId);
    if (!task) return;
    
    Tick responseTime = curTick() - task->nextRelease;
    task->maxResponseTime = std::max(task->maxResponseTime, responseTime);
    task->minResponseTime = std::min(task->minResponseTime, responseTime);
    task->totalExecutionTime += task->wcet - task->remainingTime;
    
    task->state = TaskState::COMPLETED;
    stats.tasksCompleted++;
    
    // Release accelerators
    releaseAccelerators(task);
    
    // Clear from running
    runningTasks[coreId] = nullptr;
    
    DPRINTF(TaskScheduler, "Task %s completed on core %d, response=%lu\n",
            task->name.c_str(), coreId, responseTime);
    
    // Schedule next invocation for periodic tasks
    if (task->period > 0) {
        task->nextRelease += task->period;
        task->remainingTime = task->wcet;
        task->state = TaskState::READY;
    }
    
    // Trigger rescheduling
    runScheduler();
}

void GlobalTaskScheduler::onAccelAvailable(AccelType type)
{
    // Check if any waiting tasks can now run
    for (auto &[id, task] : tasks) {
        if (task.state == TaskState::WAITING_ACCEL) {
            if (tryAcquireAccelerators(&task)) {
                task.state = TaskState::READY;
                int coreId = selectCoreForTask(&task);
                readyQueues[coreId].push_back(&task);
            }
        }
    }
    runScheduler();
}

void GlobalTaskScheduler::setAcceleratorManager(AcceleratorResourceManager *mgr)
{
    accelManager = mgr;
}

// Scheduling Implementations

void GlobalTaskScheduler::scheduleFixedPriority()
{
    for (int coreId = 0; coreId < numCores; coreId++) {
        if (readyQueues[coreId].empty()) continue;
        
        // Sort by priority (lower priority number = higher priority)
        std::sort(readyQueues[coreId].begin(), readyQueues[coreId].end(),
            [this](SpacecraftTask* a, SpacecraftTask* b) {
                return getPriority(a) < getPriority(b);
            });
        
        SpacecraftTask *highest = readyQueues[coreId].front();
        SpacecraftTask *current = runningTasks[coreId];
        
        if (current == nullptr) {
            // Core idle, dispatch highest priority
            readyQueues[coreId].erase(readyQueues[coreId].begin());
            dispatch(highest, coreId);
        } else if (getPriority(highest) < getPriority(current)) {
            // Higher priority task ready, preempt if possible
            if (current->isPreemptible) {
                preempt(current, coreId);
                readyQueues[coreId].erase(readyQueues[coreId].begin());
                dispatch(highest, coreId);
            }
        }
    }
}

void GlobalTaskScheduler::scheduleMixedCriticalityEDF()
{
    for (int coreId = 0; coreId < numCores; coreId++) {
        if (readyQueues[coreId].empty()) continue;
        
        // Sort by: (1) Criticality, (2) Deadline
        std::sort(readyQueues[coreId].begin(), readyQueues[coreId].end(),
            [](SpacecraftTask* a, SpacecraftTask* b) {
                if (a->criticality != b->criticality) {
                    return static_cast<int>(a->criticality) < static_cast<int>(b->criticality);
                }
                return (a->nextRelease + a->deadline) < (b->nextRelease + b->deadline);
            });
        
        SpacecraftTask *highest = readyQueues[coreId].front();
        SpacecraftTask *current = runningTasks[coreId];
        
        if (current == nullptr) {
            readyQueues[coreId].erase(readyQueues[coreId].begin());
            dispatch(highest, coreId);
        } else {
            // Check if we should preempt
            bool shouldPreempt = false;
            if (static_cast<int>(highest->criticality) < static_cast<int>(current->criticality)) {
                shouldPreempt = true;
            } else if (highest->criticality == current->criticality) {
                Tick highestDeadline = highest->nextRelease + highest->deadline;
                Tick currentDeadline = current->nextRelease + current->deadline;
                if (highestDeadline < currentDeadline) {
                    shouldPreempt = true;
                }
            }
            
            if (shouldPreempt && current->isPreemptible) {
                preempt(current, coreId);
                readyQueues[coreId].erase(readyQueues[coreId].begin());
                dispatch(highest, coreId);
            }
        }
    }
}

void GlobalTaskScheduler::scheduleEnergyAwareEDF()
{
    // First, check power budget
    float currentPower = getCurrentPower();
    
    if (currentPower > powerBudget) {
        shedLowPriorityTasks();
    }
    
    // Then apply EDF
    scheduleMixedCriticalityEDF();
}

void GlobalTaskScheduler::scheduleMinimal()
{
    // Only allow mission-critical and safety-critical tasks
    for (auto &[id, task] : tasks) {
        if (task.criticality > CriticalityLevel::SAFETY_CRITICAL) {
            if (task.state == TaskState::READY) {
                task.state = TaskState::SUSPENDED;
            }
        }
    }
    
    scheduleFixedPriority();
}

void GlobalTaskScheduler::scheduleRoundRobin()
{
    static int roundRobinIndex = 0;
    
    for (int coreId = 0; coreId < numCores; coreId++) {
        if (readyQueues[coreId].empty()) continue;
        
        if (runningTasks[coreId] == nullptr) {
            // Dispatch next in round-robin order
            int idx = roundRobinIndex % readyQueues[coreId].size();
            SpacecraftTask *task = readyQueues[coreId][idx];
            readyQueues[coreId].erase(readyQueues[coreId].begin() + idx);
            dispatch(task, coreId);
            roundRobinIndex++;
        }
    }
}

int GlobalTaskScheduler::getPriority(const SpacecraftTask *task) const
{
    // Priority = Criticality * 1000 + Period / TICK_MS
    // Lower number = higher priority
    int critPriority = static_cast<int>(task->criticality) * 1000;
    int periodPriority = task->period > 0 ? task->period / 1000000 : 1000;  // Convert to ms
    return critPriority + periodPriority;
}

int GlobalTaskScheduler::selectCore()
{
    // Select least loaded core
    int selectedCore = 0;
    size_t minLoad = readyQueues[0].size() + (runningTasks[0] ? 1 : 0);
    
    for (int i = 1; i < numCores; i++) {
        size_t load = readyQueues[i].size() + (runningTasks[i] ? 1 : 0);
        if (load < minLoad) {
            minLoad = load;
            selectedCore = i;
        }
    }
    
    return selectedCore;
}

int GlobalTaskScheduler::selectCoreForTask(const SpacecraftTask *task)
{
    if (task->coreAffinity >= 0 && task->coreAffinity < numCores) {
        return task->coreAffinity;
    }
    return selectCore();
}

void GlobalTaskScheduler::dispatch(SpacecraftTask *task, int coreId)
{
    // Try to acquire accelerators first
    if (!task->accelNeeds.empty() && !tryAcquireAccelerators(task)) {
        task->state = TaskState::WAITING_ACCEL;
        readyQueues[coreId].push_back(task);  // Put back in queue
        DPRINTF(TaskScheduler, "Task %s waiting for accelerators\n", task->name.c_str());
        return;
    }
    
    runningTasks[coreId] = task;
    task->state = TaskState::RUNNING;
    stats.totalContextSwitches++;
    
    DPRINTF(TaskScheduler, "Dispatched task %s to core %d (criticality=%d, deadline=%lu)\n",
            task->name.c_str(), coreId, static_cast<int>(task->criticality), task->deadline);
}

void GlobalTaskScheduler::preempt(SpacecraftTask *task, int coreId)
{
    // Save context (simulated as overhead)
    task->state = TaskState::READY;
    
    // Release accelerators
    releaseAccelerators(task);
    
    // Put back in ready queue
    readyQueues[coreId].push_back(task);
    runningTasks[coreId] = nullptr;
    
    stats.totalPreemptions++;
    stats.totalContextSwitches++;
    
    DPRINTF(TaskScheduler, "Preempted task %s on core %d\n", task->name.c_str(), coreId);
}

void GlobalTaskScheduler::releasePeriodicTasks()
{
    Tick currentTime = curTick();
    
    for (auto &[id, task] : tasks) {
        if (task.period > 0 && 
            activeTaskIds.find(id) != activeTaskIds.end() &&
            currentTime >= task.nextRelease) {
            
            if (task.state == TaskState::COMPLETED || task.state == TaskState::READY) {
                task.state = TaskState::READY;
                task.remainingTime = task.wcet;
                task.numReleases++;
                
                int coreId = selectCoreForTask(&task);
                readyQueues[coreId].push_back(&task);
                
                DPRINTF(TaskScheduler, "Released task %s (release #%lu)\n",
                        task.name.c_str(), task.numReleases);
            }
        }
    }
    
    runScheduler();
    
    // Reschedule periodic release check
    schedule(periodicReleaseEvent, curTick() + schedulingPeriod);
}

void GlobalTaskScheduler::shedLowPriorityTasks()
{
    // Suspend lowest priority tasks to meet power budget
    std::vector<SpacecraftTask*> runningByPriority;
    
    for (int i = 0; i < numCores; i++) {
        if (runningTasks[i]) {
            runningByPriority.push_back(runningTasks[i]);
        }
    }
    
    // Sort by priority (highest priority value = lowest priority)
    std::sort(runningByPriority.begin(), runningByPriority.end(),
        [this](SpacecraftTask* a, SpacecraftTask* b) {
            return getPriority(a) > getPriority(b);
        });
    
    float currentPower = getCurrentPower();
    
    for (SpacecraftTask *task : runningByPriority) {
        if (currentPower <= powerBudget) break;
        
        // Don't shed critical tasks
        if (task->criticality <= CriticalityLevel::SAFETY_CRITICAL) continue;
        
        task->state = TaskState::SUSPENDED;
        currentPower -= task->powerWatts;
        stats.tasksShed++;
        
        DPRINTF(TaskScheduler, "Shed task %s due to power constraints\n", task->name.c_str());
    }
}

bool GlobalTaskScheduler::tryAcquireAccelerators(SpacecraftTask *task)
{
    // Placeholder - would call accelManager->tryAcquire()
    // For now, always succeed
    return true;
}

void GlobalTaskScheduler::releaseAccelerators(SpacecraftTask *task)
{
    // Placeholder - would call accelManager->release()
}

// Comparator implementations
bool GlobalTaskScheduler::PriorityComparator::operator()(
    SpacecraftTask* a, SpacecraftTask* b) const
{
    return scheduler->getPriority(a) > scheduler->getPriority(b);
}

bool GlobalTaskScheduler::EDFComparator::operator()(
    SpacecraftTask* a, SpacecraftTask* b) const
{
    return (a->nextRelease + a->deadline) > (b->nextRelease + b->deadline);
}

} // namespace spacecraft
} // namespace gem5


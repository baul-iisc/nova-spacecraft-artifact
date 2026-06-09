/*
 * Copyright (c) 2024 NOVA Processor Research
 * Real-Time Scheduling for Spacecraft Accelerators
 *
 * RTScheduler implementation
 */

#include "rt_scheduler.hh"
#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdint>
#include <iostream>
#include <iomanip>

namespace gem5
{

RTScheduler::RTScheduler(int numCores, SchedulingPolicy policy)
    : m_policy(policy),
      m_numCores(numCores),
      m_currentUtilization(0.0),
      m_trigAccelOwner(-1),
      m_matAccelOwner(-1),
      m_vpuOwner(-1),
      m_npuOwner(-1),
      m_totalSchedulingDecisions(0),
      m_totalContextSwitches(0),
      m_totalPreemptions(0)
{
}

RTScheduler::~RTScheduler()
{
}

void RTScheduler::addTask(RTTask& task)
{
    // Assign task ID if not set
    if (task.taskId == 0) {
        task.taskId = m_tasks.size() + 1;
    }
    
    // Adjust WCET for contention
    adjustWCETForContention(task);
    
    // Set default deadline if not specified
    if (task.deadline == 0) {
        task.deadline = task.period;
    }
    
    m_tasks.push_back(task);
    m_originalPriorities.push_back(task.priority);
    
    // Reassign priorities for RM
    if (m_policy == SchedulingPolicy::CA_RM || m_policy == SchedulingPolicy::RM) {
        assignRMPriorities();
    }
    
    // Update system utilization
    updateContentionPredictions();
}

void RTScheduler::removeTask(int taskId)
{
    auto it = std::find_if(m_tasks.begin(), m_tasks.end(),
                           [taskId](const RTTask& t) { return t.taskId == taskId; });
    if (it != m_tasks.end()) {
        int idx = std::distance(m_tasks.begin(), it);
        m_tasks.erase(it);
        m_originalPriorities.erase(m_originalPriorities.begin() + idx);
    }
}

RTTask* RTScheduler::getTask(int taskId)
{
    for (auto& task : m_tasks) {
        if (task.taskId == taskId) {
            return &task;
        }
    }
    return nullptr;
}

void RTScheduler::adjustWCETForContention(RTTask& task)
{
    // Calculate average accelerator latency for this task
    int totalLatency = 0;
    int accelCount = 0;
    
    for (auto accel : task.requiredAccelerators) {
        switch (accel) {
            case AcceleratorType::TRIG_ACCEL:
                totalLatency += ContentionPredictor::TRIG_LATENCY;
                accelCount++;
                break;
            case AcceleratorType::MAT_ACCEL:
                totalLatency += ContentionPredictor::MAT_LATENCY;
                accelCount++;
                break;
            case AcceleratorType::VPU:
                totalLatency += ContentionPredictor::VPU_LATENCY;
                accelCount++;
                break;
            case AcceleratorType::NPU:
                totalLatency += ContentionPredictor::NPU_LATENCY;
                accelCount++;
                break;
            default:
                break;
        }
    }
    
    int avgLatency = (accelCount > 0) ? (totalLatency / accelCount) : 30;
    
    // Estimate utilization for this task's accelerator usage
    double util = ContentionPredictor::estimateUtilization(
        task.accelOpsPerPeriod, avgLatency, task.period, m_numCores);
    
    // Use contention-aware WCET if CA policy
    if (m_policy == SchedulingPolicy::CA_RM || 
        m_policy == SchedulingPolicy::CA_EDF ||
        m_policy == SchedulingPolicy::CA_LLF) {
        task.adjustedWCET = ContentionPredictor::predictWCET(
            task.baseWCET, m_numCores, util);
    } else {
        task.adjustedWCET = task.baseWCET;
    }
}

void RTScheduler::assignRMPriorities()
{
    // Sort by period (shorter period = higher priority = lower priority value)
    std::vector<RTTask*> sortedTasks;
    for (auto& task : m_tasks) {
        sortedTasks.push_back(&task);
    }
    
    std::sort(sortedTasks.begin(), sortedTasks.end(),
              [](RTTask* a, RTTask* b) { return a->period < b->period; });
    
    for (size_t i = 0; i < sortedTasks.size(); i++) {
        sortedTasks[i]->priority = i;
    }
}

RTTask* RTScheduler::selectNextTask(uint64_t currentTime)
{
    m_totalSchedulingDecisions++;
    
    // Check for deadline misses
    for (auto& task : m_tasks) {
        if (task.isActive && currentTime > task.currentDeadline) {
            task.deadlineMisses++;
            task.isActive = false;
            
            // For safety-critical tasks, log warning
            if (task.criticality == Criticality::SAFETY_CRITICAL) {
                std::cerr << "[RT-WARN] Safety-critical task " << task.name 
                          << " missed deadline at tick " << currentTime << std::endl;
            }
        }
    }
    
    // Select based on policy
    switch (m_policy) {
        case SchedulingPolicy::CA_RM:
        case SchedulingPolicy::RM:
            return selectByRM(currentTime);
            
        case SchedulingPolicy::CA_EDF:
        case SchedulingPolicy::EDF:
            return selectByEDF(currentTime);
            
        case SchedulingPolicy::CA_LLF:
            return selectByLLF(currentTime);
            
        default:
            return selectByRM(currentTime);
    }
}

RTTask* RTScheduler::selectByRM(uint64_t currentTime)
{
    RTTask* selectedTask = nullptr;
    int highestPriority = INT_MAX;
    
    for (auto& task : m_tasks) {
        // Check if task is ready
        if (task.isReady && currentTime >= task.nextRelease) {
            // For mixed-criticality: prioritize safety-critical during overload
            int effectivePriority = task.priority;
            if (getSystemLoad() > 1.0 && 
                task.criticality == Criticality::SAFETY_CRITICAL) {
                effectivePriority -= 100; // Boost priority
            }
            
            if (effectivePriority < highestPriority) {
                highestPriority = effectivePriority;
                selectedTask = &task;
            }
        }
    }
    
    return selectedTask;
}

RTTask* RTScheduler::selectByEDF(uint64_t currentTime)
{
    RTTask* selectedTask = nullptr;
    uint64_t earliestDeadline = UINT64_MAX;
    
    for (auto& task : m_tasks) {
        if (task.isReady && currentTime >= task.nextRelease) {
            if (task.currentDeadline < earliestDeadline) {
                earliestDeadline = task.currentDeadline;
                selectedTask = &task;
            }
        }
    }
    
    return selectedTask;
}

RTTask* RTScheduler::selectByLLF(uint64_t currentTime)
{
    RTTask* selectedTask = nullptr;
    int64_t minLaxity = INT64_MAX;
    
    for (auto& task : m_tasks) {
        if (task.isReady && currentTime >= task.nextRelease) {
            int64_t laxity = task.currentDeadline - currentTime - task.adjustedWCET;
            if (laxity < minLaxity) {
                minLaxity = laxity;
                selectedTask = &task;
            }
        }
    }
    
    return selectedTask;
}

void RTScheduler::releaseTask(int taskId, uint64_t releaseTime)
{
    RTTask* task = getTask(taskId);
    if (task) {
        task->isReady = true;
        task->nextRelease = releaseTime;
        task->currentDeadline = releaseTime + task->deadline;
    }
}

void RTScheduler::completeTask(int taskId, uint64_t completionTime)
{
    RTTask* task = getTask(taskId);
    if (task) {
        // Update statistics
        task->completedInstances++;
        uint64_t responseTime = completionTime - (task->currentDeadline - task->deadline);
        task->totalResponseTime += responseTime;
        if (responseTime > task->worstResponseTime) {
            task->worstResponseTime = responseTime;
        }
        
        // Check for deadline miss
        if (completionTime > task->currentDeadline) {
            task->deadlineMisses++;
        }
        
        // Reset for next period
        task->isActive = false;
        task->isReady = false;
        task->nextRelease = task->currentDeadline; // Next release at previous deadline
    }
}

void RTScheduler::updateContentionPredictions()
{
    // Calculate total system utilization
    m_currentUtilization = 0.0;
    for (auto& task : m_tasks) {
        m_currentUtilization += static_cast<double>(task.adjustedWCET) / task.period;
    }
    
    // Re-adjust all WCETs with new utilization estimate
    for (auto& task : m_tasks) {
        adjustWCETForContention(task);
    }
}

double RTScheduler::getCurrentUtilization() const
{
    return m_currentUtilization;
}

double RTScheduler::getSystemLoad() const
{
    // Load factor: utilization / number of cores
    return m_currentUtilization / m_numCores;
}

bool RTScheduler::checkSchedulability() const
{
    switch (m_policy) {
        case SchedulingPolicy::CA_RM:
        case SchedulingPolicy::RM:
            return rmSchedulabilityTest();
            
        case SchedulingPolicy::CA_EDF:
        case SchedulingPolicy::EDF:
            return edfSchedulabilityTest();
            
        default:
            return rmSchedulabilityTest();
    }
}

bool RTScheduler::rmSchedulabilityTest() const
{
    // Liu & Layland bound: U ≤ n(2^(1/n) - 1)
    int n = m_tasks.size();
    if (n == 0) return true;
    
    double bound = n * (std::pow(2.0, 1.0 / n) - 1.0);
    double utilization = 0.0;
    
    for (const auto& task : m_tasks) {
        utilization += static_cast<double>(task.adjustedWCET) / task.period;
    }
    
    return utilization <= bound;
}

bool RTScheduler::edfSchedulabilityTest() const
{
    // EDF is optimal: schedulable if U ≤ 1
    double utilization = 0.0;
    
    for (const auto& task : m_tasks) {
        utilization += static_cast<double>(task.adjustedWCET) / task.period;
    }
    
    return utilization <= 1.0;
}

double RTScheduler::getSchedulabilityBound() const
{
    int n = m_tasks.size();
    if (n == 0) return 1.0;
    
    if (m_policy == SchedulingPolicy::CA_EDF || m_policy == SchedulingPolicy::EDF) {
        return 1.0;
    }
    
    // RM bound
    return n * (std::pow(2.0, 1.0 / n) - 1.0);
}

double RTScheduler::getUtilizationBound() const
{
    return getSchedulabilityBound();
}

bool RTScheduler::requestAccelerator(int taskId, AcceleratorType accel)
{
    int* owner = nullptr;
    
    switch (accel) {
        case AcceleratorType::TRIG_ACCEL:
            owner = &m_trigAccelOwner;
            break;
        case AcceleratorType::MAT_ACCEL:
            owner = &m_matAccelOwner;
            break;
        case AcceleratorType::VPU:
            owner = &m_vpuOwner;
            break;
        case AcceleratorType::NPU:
            owner = &m_npuOwner;
            break;
        default:
            return false;
    }
    
    if (*owner == -1) {
        *owner = taskId;
        return true;
    } else if (*owner == taskId) {
        return true; // Already owns it
    } else {
        // Apply priority inheritance
        applyPriorityInheritance(taskId, accel);
        return false;
    }
}

void RTScheduler::releaseAccelerator(int taskId, AcceleratorType accel)
{
    int* owner = nullptr;
    
    switch (accel) {
        case AcceleratorType::TRIG_ACCEL:
            owner = &m_trigAccelOwner;
            break;
        case AcceleratorType::MAT_ACCEL:
            owner = &m_matAccelOwner;
            break;
        case AcceleratorType::VPU:
            owner = &m_vpuOwner;
            break;
        case AcceleratorType::NPU:
            owner = &m_npuOwner;
            break;
        default:
            return;
    }
    
    if (*owner == taskId) {
        *owner = -1;
        restorePriority(taskId);
    }
}

int RTScheduler::getAcceleratorOwner(AcceleratorType accel) const
{
    switch (accel) {
        case AcceleratorType::TRIG_ACCEL:
            return m_trigAccelOwner;
        case AcceleratorType::MAT_ACCEL:
            return m_matAccelOwner;
        case AcceleratorType::VPU:
            return m_vpuOwner;
        case AcceleratorType::NPU:
            return m_npuOwner;
        default:
            return -1;
    }
}

void RTScheduler::applyPriorityInheritance(int waitingTaskId, AcceleratorType accel)
{
    int ownerId = getAcceleratorOwner(accel);
    if (ownerId == -1) return;
    
    RTTask* waitingTask = getTask(waitingTaskId);
    RTTask* ownerTask = getTask(ownerId);
    
    if (waitingTask && ownerTask) {
        // If waiting task has higher priority, inherit it
        if (waitingTask->priority < ownerTask->priority) {
            ownerTask->priority = waitingTask->priority;
        }
    }
}

void RTScheduler::restorePriority(int taskId)
{
    for (size_t i = 0; i < m_tasks.size(); i++) {
        if (m_tasks[i].taskId == taskId) {
            m_tasks[i].priority = m_originalPriorities[i];
            break;
        }
    }
}

void RTScheduler::printStatistics() const
{
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║           NOVA Real-Time Scheduler Statistics                    ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    
    std::cout << "║ Scheduling Policy: ";
    switch (m_policy) {
        case SchedulingPolicy::CA_RM:  std::cout << "Contention-Aware RM"; break;
        case SchedulingPolicy::CA_EDF: std::cout << "Contention-Aware EDF"; break;
        case SchedulingPolicy::CA_LLF: std::cout << "Contention-Aware LLF"; break;
        case SchedulingPolicy::RM:     std::cout << "Rate Monotonic"; break;
        case SchedulingPolicy::EDF:    std::cout << "Earliest Deadline First"; break;
    }
    std::cout << std::setw(24) << "║\n";
    
    std::cout << "║ Number of Cores: " << m_numCores << std::setw(48) << "║\n";
    std::cout << "║ System Utilization: " << std::fixed << std::setprecision(2) 
              << (m_currentUtilization * 100) << "%" << std::setw(43) << "║\n";
    std::cout << "║ Schedulability Bound: " << std::fixed << std::setprecision(2)
              << (getSchedulabilityBound() * 100) << "%" << std::setw(41) << "║\n";
    std::cout << "║ Schedulable: " << (checkSchedulability() ? "YES" : "NO") 
              << std::setw(51) << "║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Task Statistics:                                                 ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Name          │ Period │ WCET  │ Adj.WCET │ Completed │ Missed  ║\n";
    std::cout << "╠───────────────┼────────┼───────┼──────────┼───────────┼─────────╣\n";
    
    for (const auto& task : m_tasks) {
        std::cout << "║ " << std::left << std::setw(13) << task.name.substr(0, 13)
                  << " │ " << std::right << std::setw(6) << task.period
                  << " │ " << std::setw(5) << task.baseWCET
                  << " │ " << std::setw(8) << task.adjustedWCET
                  << " │ " << std::setw(9) << task.completedInstances
                  << " │ " << std::setw(7) << task.deadlineMisses << " ║\n";
    }
    
    std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Total Scheduling Decisions: " << std::setw(10) << m_totalSchedulingDecisions 
              << std::setw(27) << "║\n";
    std::cout << "║ Total Deadline Misses: " << std::setw(10) << getTotalDeadlineMisses()
              << std::setw(32) << "║\n";
    std::cout << "║ Deadline Miss Rate: " << std::fixed << std::setprecision(2)
              << (getDeadlineMissRate() * 100) << "%" << std::setw(43) << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
}

uint64_t RTScheduler::getTotalDeadlineMisses() const
{
    uint64_t total = 0;
    for (const auto& task : m_tasks) {
        total += task.deadlineMisses;
    }
    return total;
}

double RTScheduler::getDeadlineMissRate() const
{
    uint64_t totalCompleted = 0;
    uint64_t totalMisses = 0;
    
    for (const auto& task : m_tasks) {
        totalCompleted += task.completedInstances;
        totalMisses += task.deadlineMisses;
    }
    
    if (totalCompleted + totalMisses == 0) return 0.0;
    return static_cast<double>(totalMisses) / (totalCompleted + totalMisses);
}

} // namespace gem5


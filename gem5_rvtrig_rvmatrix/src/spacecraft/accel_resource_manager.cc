/*
 * NOVA Processor - Accelerator Resource Manager Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/accel_resource_manager.hh"
#include "base/trace.hh"
#include "debug/AccelManager.hh"

#include <algorithm>

namespace gem5
{

namespace spacecraft
{

AcceleratorResourceManager::AcceleratorResourceManager(const Params &p)
    : ClockedObject(p),
      currentMode(SharingMode::ADAPTIVE),
      arbitrationPolicy(ArbitrationPolicy::PRIORITY_BASED),
      numCores(p.num_cores),
      highContentionThreshold(p.high_contention_threshold),
      lowContentionThreshold(p.low_contention_threshold),
      idleThreshold(p.idle_threshold),
      contentionCheckEvent([this]{ checkContention(); }, name() + ".contentionCheckEvent"),
      powerManagementEvent([this]{ /* power management */ }, name() + ".powerManagementEvent")
{
    // Initialize accelerator configurations
    accelConfigs[AccelType::TRIG_ACCEL] = {
        p.num_trig_accels,
        100.0,  // 100mW active
        10.0,   // 10mW idle
        Cycles(10)
    };
    
    accelConfigs[AccelType::MAT_ACCEL] = {
        p.num_mat_accels,
        150.0,
        15.0,
        Cycles(27)  // 3x3 matrix multiply
    };
    
    accelConfigs[AccelType::VPU] = {
        p.num_vpus,
        200.0,
        20.0,
        Cycles(1000)
    };
    
    accelConfigs[AccelType::NPU] = {
        p.num_npus,
        500.0,
        50.0,
        Cycles(10000)
    };
    
    // Create accelerator instances
    for (auto &[type, config] : accelConfigs) {
        for (int i = 0; i < config.numInstances; i++) {
            AccelInstance inst;
            inst.type = type;
            inst.instanceId = i;
            inst.isBusy = false;
            inst.currentOwnerTask = -1;
            inst.dedicatedToCoreId = -1;
            inst.powerState = AccelPowerState::IDLE;
            inst.currentPowerMW = config.idlePowerMW;
            
            accelerators[type].push_back(inst);
        }
    }
    
    // Parse initial sharing mode
    std::string modeStr = p.sharing_mode;
    if (modeStr == "FULLY_SHARED") {
        currentMode = SharingMode::FULLY_SHARED;
    } else if (modeStr == "FULLY_DEDICATED") {
        currentMode = SharingMode::FULLY_DEDICATED;
    } else if (modeStr == "HYBRID_SHARED") {
        currentMode = SharingMode::HYBRID_SHARED;
    } else {
        currentMode = SharingMode::ADAPTIVE;
    }
    
    resetStats();
    
    // Schedule periodic contention check
    schedule(contentionCheckEvent, curTick() + p.contention_check_period);
    
    DPRINTF(AccelManager, "Accelerator Resource Manager initialized: mode=%s, cores=%d\n",
            sharingModeToString(currentMode).c_str(), numCores);
    DPRINTF(AccelManager, "  TRIG: %d, MAT: %d, VPU: %d, NPU: %d\n",
            p.num_trig_accels, p.num_mat_accels, p.num_vpus, p.num_npus);
}

AcceleratorResourceManager::~AcceleratorResourceManager()
{
}

bool AcceleratorResourceManager::tryAcquire(AccelType type, int taskId, int coreId,
                                             CriticalityLevel criticality)
{
    if (accelerators.find(type) == accelerators.end()) {
        return false;
    }
    
    bool success = false;
    
    switch (currentMode) {
        case SharingMode::FULLY_SHARED:
            success = tryAcquireShared(type, taskId, coreId, criticality);
            break;
        case SharingMode::FULLY_DEDICATED:
            success = tryAcquireDedicated(type, taskId, coreId, criticality);
            break;
        case SharingMode::HYBRID_SHARED:
            success = tryAcquireHybrid(type, taskId, coreId, criticality);
            break;
        case SharingMode::ADAPTIVE:
            success = tryAcquireAdaptive(type, taskId, coreId, criticality);
            break;
    }
    
    if (success) {
        stats.totalRequestsPerType[type]++;
    }
    
    return success;
}

bool AcceleratorResourceManager::tryAcquireWithDeadline(AccelType type, int taskId,
                                                         int coreId, CriticalityLevel criticality,
                                                         Tick deadline)
{
    // Same as tryAcquire but stores deadline for scheduling
    return tryAcquire(type, taskId, coreId, criticality);
}

void AcceleratorResourceManager::release(AccelType type, int taskId)
{
    if (accelerators.find(type) == accelerators.end()) {
        return;
    }
    
    auto &instances = accelerators[type];
    
    for (auto &inst : instances) {
        if (inst.currentOwnerTask == taskId) {
            inst.isBusy = false;
            inst.currentOwnerTask = -1;
            inst.lastUsedTime = curTick();
            inst.powerState = AccelPowerState::IDLE;
            inst.currentPowerMW = accelConfigs[type].idlePowerMW;
            
            DPRINTF(AccelManager, "Task %d released %s[%d]\n",
                    taskId, accelTypeToString(type).c_str(), inst.instanceId);
            
            // Process wait queue
            processWaitQueue(inst);
            break;
        }
    }
}

void AcceleratorResourceManager::queueRequest(AccelRequest &req)
{
    if (accelerators.find(req.type) == accelerators.end()) {
        if (req.callback) req.callback(false);
        return;
    }
    
    auto &instances = accelerators[req.type];
    
    // Find least loaded instance and add to queue
    AccelInstance &target = findLeastLoadedInstance(instances);
    req.requestTime = curTick();
    target.waitQueue.push(req);
    
    stats.totalContentionEvents++;
    
    DPRINTF(AccelManager, "Queued request for %s (task %d, queue size=%lu)\n",
            accelTypeToString(req.type).c_str(), req.taskId, target.waitQueue.size());
}

bool AcceleratorResourceManager::isAccelAvailable(AccelType type) const
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return false;
    
    for (const auto &inst : it->second) {
        if (!inst.isBusy && inst.powerState != AccelPowerState::POWER_GATED) {
            return true;
        }
    }
    return false;
}

size_t AcceleratorResourceManager::getQueueDepth(AccelType type) const
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return 0;
    
    size_t totalDepth = 0;
    for (const auto &inst : it->second) {
        totalDepth += inst.waitQueue.size();
    }
    return totalDepth;
}

float AcceleratorResourceManager::getContention(AccelType type) const
{
    return calculateContention(type);
}

float AcceleratorResourceManager::getCurrentPower() const
{
    float total = 0.0f;
    for (const auto &[type, instances] : accelerators) {
        for (const auto &inst : instances) {
            total += inst.currentPowerMW;
        }
    }
    return total / 1000.0f;  // Convert to Watts
}

void AcceleratorResourceManager::setSharingMode(SharingMode mode)
{
    if (mode != currentMode) {
        DPRINTF(AccelManager, "Sharing mode changed: %s -> %s\n",
                sharingModeToString(currentMode).c_str(),
                sharingModeToString(mode).c_str());
        currentMode = mode;
        stats.adaptiveModeChanges++;
    }
}

void AcceleratorResourceManager::setArbitrationPolicy(ArbitrationPolicy policy)
{
    arbitrationPolicy = policy;
}

void AcceleratorResourceManager::reconfigure(MissionPhase phase)
{
    DPRINTF(AccelManager, "Reconfiguring for phase %d\n", static_cast<int>(phase));
    
    switch (phase) {
        case MissionPhase::LAUNCH:
        case MissionPhase::LANDING:
            // Critical phases - use dedicated for predictability
            currentMode = SharingMode::FULLY_DEDICATED;
            arbitrationPolicy = ArbitrationPolicy::PRIORITY_BASED;
            break;
            
        case MissionPhase::NORMAL_OPS:
            // Normal operations - adaptive sharing
            currentMode = SharingMode::ADAPTIVE;
            arbitrationPolicy = ArbitrationPolicy::DEADLINE_AWARE;
            break;
            
        case MissionPhase::SURFACE_OPS:
            // Power constrained - hybrid with power gating
            currentMode = SharingMode::HYBRID_SHARED;
            arbitrationPolicy = ArbitrationPolicy::PROPORTIONAL_SHARE;
            powerGateNonEssential();
            break;
            
        case MissionPhase::SAFE_MODE:
            // Minimal operations
            currentMode = SharingMode::FULLY_SHARED;
            powerGateNonEssential();
            break;
            
        default:
            currentMode = SharingMode::ADAPTIVE;
            break;
    }
}

void AcceleratorResourceManager::powerGateAccel(AccelType type, int instanceId)
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return;
    
    for (auto &inst : it->second) {
        if (inst.instanceId == instanceId) {
            if (inst.powerState != AccelPowerState::POWER_GATED) {
                inst.powerState = AccelPowerState::POWER_GATED;
                inst.currentPowerMW = 0.0f;
                stats.powerGatingEvents++;
                
                DPRINTF(AccelManager, "Power gated %s[%d]\n",
                        accelTypeToString(type).c_str(), instanceId);
            }
            break;
        }
    }
}

void AcceleratorResourceManager::powerUpAccel(AccelType type, int instanceId)
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return;
    
    for (auto &inst : it->second) {
        if (inst.instanceId == instanceId) {
            if (inst.powerState == AccelPowerState::POWER_GATED) {
                inst.powerState = AccelPowerState::IDLE;
                inst.currentPowerMW = accelConfigs[type].idlePowerMW;
                
                DPRINTF(AccelManager, "Powered up %s[%d]\n",
                        accelTypeToString(type).c_str(), instanceId);
            }
            break;
        }
    }
}

void AcceleratorResourceManager::powerGateNonEssential()
{
    // Keep TRIG and MAT for navigation, power gate VPU and NPU
    for (auto &inst : accelerators[AccelType::VPU]) {
        powerGateAccel(AccelType::VPU, inst.instanceId);
    }
    for (auto &inst : accelerators[AccelType::NPU]) {
        powerGateAccel(AccelType::NPU, inst.instanceId);
    }
}

void AcceleratorResourceManager::powerUpAll()
{
    for (auto &[type, instances] : accelerators) {
        for (auto &inst : instances) {
            powerUpAccel(type, inst.instanceId);
        }
    }
}

void AcceleratorResourceManager::resetStats()
{
    stats.totalRequestsPerType.clear();
    stats.totalBusyCyclesPerType.clear();
    stats.avgUtilizationPerType.clear();
    stats.avgQueueDepthPerType.clear();
    stats.avgWaitTimePerType.clear();
    stats.totalContentionEvents = 0;
    stats.adaptiveModeChanges = 0;
    stats.totalPowerConsumed = 0.0f;
    stats.powerGatingEvents = 0;
}

void AcceleratorResourceManager::printStatistics()
{
    DPRINTF(AccelManager, "=== Accelerator Resource Manager Statistics ===\n");
    
    for (const auto &[type, instances] : accelerators) {
        std::string typeName = accelTypeToString(type);
        DPRINTF(AccelManager, "%s:\n", typeName.c_str());
        
        for (const auto &inst : instances) {
            float utilization = (float)inst.totalBusyCycles / curTick();
            DPRINTF(AccelManager, "  Instance %d: Requests=%lu, Utilization=%.2f%%, AvgLatency=%.1f\n",
                    inst.instanceId, inst.totalRequests, utilization * 100, inst.avgLatency);
        }
    }
    
    DPRINTF(AccelManager, "Contention Events: %lu\n", stats.totalContentionEvents);
    DPRINTF(AccelManager, "Mode Changes: %lu\n", stats.adaptiveModeChanges);
    DPRINTF(AccelManager, "Power Gating Events: %lu\n", stats.powerGatingEvents);
}

void AcceleratorResourceManager::addAccelInstance(AccelType type, int instanceId)
{
    AccelInstance inst;
    inst.type = type;
    inst.instanceId = instanceId;
    inst.isBusy = false;
    inst.powerState = AccelPowerState::IDLE;
    
    if (accelConfigs.find(type) != accelConfigs.end()) {
        inst.currentPowerMW = accelConfigs[type].idlePowerMW;
    }
    
    accelerators[type].push_back(inst);
}

void AcceleratorResourceManager::setDedicated(AccelType type, int instanceId, int coreId)
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return;
    
    for (auto &inst : it->second) {
        if (inst.instanceId == instanceId) {
            inst.dedicatedToCoreId = coreId;
            DPRINTF(AccelManager, "Set %s[%d] dedicated to core %d\n",
                    accelTypeToString(type).c_str(), instanceId, coreId);
            break;
        }
    }
}

// Acquisition Strategies

bool AcceleratorResourceManager::tryAcquireShared(AccelType type, int taskId,
                                                   int coreId, CriticalityLevel criticality)
{
    auto &instances = accelerators[type];
    
    // Find any available instance
    for (auto &inst : instances) {
        if (!inst.isBusy && inst.powerState != AccelPowerState::POWER_GATED) {
            allocate(inst, taskId);
            return true;
        }
    }
    
    // All busy - queue request
    AccelRequest req = {taskId, coreId, type, criticality, curTick(), 0, nullptr};
    AccelInstance &leastLoaded = findLeastLoadedInstance(instances);
    leastLoaded.waitQueue.push(req);
    
    DPRINTF(AccelManager, "Task %d queued for %s (queue size=%lu)\n",
            taskId, accelTypeToString(type).c_str(), leastLoaded.waitQueue.size());
    
    return false;
}

bool AcceleratorResourceManager::tryAcquireDedicated(AccelType type, int taskId,
                                                      int coreId, CriticalityLevel criticality)
{
    auto &instances = accelerators[type];
    
    // Find instance dedicated to this core
    for (auto &inst : instances) {
        if (inst.dedicatedToCoreId == coreId) {
            if (!inst.isBusy) {
                allocate(inst, taskId);
                return true;
            } else {
                // Queue on dedicated instance only
                AccelRequest req = {taskId, coreId, type, criticality, curTick(), 0, nullptr};
                inst.waitQueue.push(req);
                return false;
            }
        }
    }
    
    // No dedicated instance - fall back to shared
    return tryAcquireShared(type, taskId, coreId, criticality);
}

bool AcceleratorResourceManager::tryAcquireHybrid(AccelType type, int taskId,
                                                   int coreId, CriticalityLevel criticality)
{
    // High-criticality tasks get dedicated access
    if (criticality <= CriticalityLevel::SAFETY_CRITICAL) {
        if (tryAcquireDedicated(type, taskId, coreId, criticality)) {
            return true;
        }
    }
    
    // Others use shared pool
    return tryAcquireShared(type, taskId, coreId, criticality);
}

bool AcceleratorResourceManager::tryAcquireAdaptive(AccelType type, int taskId,
                                                     int coreId, CriticalityLevel criticality)
{
    float contention = calculateContention(type);
    
    if (contention > highContentionThreshold) {
        // High contention - try to use/create dedicated
        DPRINTF(AccelManager, "High contention (%.2f) on %s - using hybrid\n",
                contention, accelTypeToString(type).c_str());
        powerUpDedicatedInstance(type);
        return tryAcquireHybrid(type, taskId, coreId, criticality);
    } else if (contention < lowContentionThreshold) {
        // Low contention - consolidate to shared
        powerGateUnderutilizedInstances(type);
        return tryAcquireShared(type, taskId, coreId, criticality);
    }
    
    // Normal contention - use hybrid
    return tryAcquireHybrid(type, taskId, coreId, criticality);
}

// Queue Processing

void AcceleratorResourceManager::processWaitQueue(AccelInstance &inst)
{
    if (inst.waitQueue.empty()) {
        return;
    }
    
    switch (arbitrationPolicy) {
        case ArbitrationPolicy::FCFS:
            processFCFS(inst);
            break;
        case ArbitrationPolicy::PRIORITY_BASED:
            processPriorityBased(inst);
            break;
        case ArbitrationPolicy::DEADLINE_AWARE:
            processDeadlineAware(inst);
            break;
        case ArbitrationPolicy::PROPORTIONAL_SHARE:
            processProportionalShare(inst);
            break;
    }
}

void AcceleratorResourceManager::processFCFS(AccelInstance &inst)
{
    if (!inst.waitQueue.empty()) {
        AccelRequest req = inst.waitQueue.front();
        inst.waitQueue.pop();
        allocate(inst, req.taskId);
        
        if (req.callback) {
            req.callback(true);
        }
    }
}

void AcceleratorResourceManager::processPriorityBased(AccelInstance &inst)
{
    if (inst.waitQueue.empty()) return;
    
    // Extract all requests
    std::vector<AccelRequest> requests;
    while (!inst.waitQueue.empty()) {
        requests.push_back(inst.waitQueue.front());
        inst.waitQueue.pop();
    }
    
    // Sort by criticality (lower = higher priority)
    std::sort(requests.begin(), requests.end(),
        [](const AccelRequest &a, const AccelRequest &b) {
            return static_cast<int>(a.criticality) < static_cast<int>(b.criticality);
        });
    
    // Allocate highest priority
    allocate(inst, requests[0].taskId);
    if (requests[0].callback) {
        requests[0].callback(true);
    }
    
    // Put rest back
    for (size_t i = 1; i < requests.size(); i++) {
        inst.waitQueue.push(requests[i]);
    }
}

void AcceleratorResourceManager::processDeadlineAware(AccelInstance &inst)
{
    if (inst.waitQueue.empty()) return;
    
    // Extract all requests
    std::vector<AccelRequest> requests;
    while (!inst.waitQueue.empty()) {
        requests.push_back(inst.waitQueue.front());
        inst.waitQueue.pop();
    }
    
    // Sort by: (1) Criticality, (2) Deadline
    std::sort(requests.begin(), requests.end(),
        [](const AccelRequest &a, const AccelRequest &b) {
            if (a.criticality != b.criticality) {
                return static_cast<int>(a.criticality) < static_cast<int>(b.criticality);
            }
            return a.deadline < b.deadline;
        });
    
    // Allocate first
    allocate(inst, requests[0].taskId);
    if (requests[0].callback) {
        requests[0].callback(true);
    }
    
    // Put rest back
    for (size_t i = 1; i < requests.size(); i++) {
        inst.waitQueue.push(requests[i]);
    }
}

void AcceleratorResourceManager::processProportionalShare(AccelInstance &inst)
{
    // Simple round-robin for proportional share
    processFCFS(inst);
}

// Helper Functions

void AcceleratorResourceManager::allocate(AccelInstance &inst, int taskId)
{
    inst.isBusy = true;
    inst.currentOwnerTask = taskId;
    inst.powerState = AccelPowerState::ACTIVE;
    inst.currentPowerMW = accelConfigs[inst.type].activePowerMW;
    inst.totalRequests++;
    
    DPRINTF(AccelManager, "Allocated %s[%d] to task %d\n",
            accelTypeToString(inst.type).c_str(), inst.instanceId, taskId);
}

AccelInstance& AcceleratorResourceManager::findLeastLoadedInstance(
    std::vector<AccelInstance> &instances)
{
    AccelInstance *leastLoaded = &instances[0];
    size_t minLoad = instances[0].waitQueue.size();
    
    for (auto &inst : instances) {
        if (inst.powerState == AccelPowerState::POWER_GATED) continue;
        
        size_t load = inst.waitQueue.size() + (inst.isBusy ? 1 : 0);
        if (load < minLoad) {
            minLoad = load;
            leastLoaded = &inst;
        }
    }
    
    return *leastLoaded;
}

int AcceleratorResourceManager::findMostLoadedCore()
{
    // Would need scheduler integration to determine
    return 0;  // Placeholder
}

float AcceleratorResourceManager::calculateContention(AccelType type) const
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return 0.0f;
    
    const auto &instances = it->second;
    int totalBusy = 0;
    size_t totalQueued = 0;
    int activeCount = 0;
    
    for (const auto &inst : instances) {
        if (inst.powerState != AccelPowerState::POWER_GATED) {
            activeCount++;
            if (inst.isBusy) totalBusy++;
            totalQueued += inst.waitQueue.size();
        }
    }
    
    if (activeCount == 0) return 0.0f;
    
    // Contention = (busy + queued) / active instances
    float contention = (float)(totalBusy + totalQueued) / activeCount;
    return std::min(contention, 1.0f);
}

void AcceleratorResourceManager::checkContention()
{
    updateContentionStats();
    
    if (currentMode == SharingMode::ADAPTIVE) {
        adaptSharingMode();
    }
    
    // Reschedule
    schedule(contentionCheckEvent, curTick() + 1000000);  // Check every 1ms
}

void AcceleratorResourceManager::adaptSharingMode()
{
    // Check contention across all accelerator types
    bool highContention = false;
    bool lowContention = true;
    
    for (const auto &[type, instances] : accelerators) {
        float contention = calculateContention(type);
        if (contention > highContentionThreshold) {
            highContention = true;
            lowContention = false;
        } else if (contention > lowContentionThreshold) {
            lowContention = false;
        }
    }
    
    // Adapt mode based on overall contention
    if (highContention && currentMode != SharingMode::HYBRID_SHARED) {
        DPRINTF(AccelManager, "Adaptive: switching to HYBRID due to high contention\n");
        currentMode = SharingMode::HYBRID_SHARED;
        stats.adaptiveModeChanges++;
    } else if (lowContention && currentMode != SharingMode::FULLY_SHARED) {
        DPRINTF(AccelManager, "Adaptive: switching to SHARED due to low contention\n");
        currentMode = SharingMode::FULLY_SHARED;
        stats.adaptiveModeChanges++;
    }
}

void AcceleratorResourceManager::updateContentionStats()
{
    for (auto &[type, instances] : accelerators) {
        size_t totalQueue = 0;
        for (const auto &inst : instances) {
            totalQueue += inst.waitQueue.size();
        }
        stats.avgQueueDepthPerType[type] = (float)totalQueue / instances.size();
    }
}

void AcceleratorResourceManager::powerUpDedicatedInstance(AccelType type)
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return;
    
    for (auto &inst : it->second) {
        if (inst.powerState == AccelPowerState::POWER_GATED) {
            powerUpAccel(type, inst.instanceId);
            
            // Dedicate to most loaded core
            int targetCore = findMostLoadedCore();
            inst.dedicatedToCoreId = targetCore;
            
            DPRINTF(AccelManager, "Powered up and dedicated %s[%d] to core %d\n",
                    accelTypeToString(type).c_str(), inst.instanceId, targetCore);
            return;
        }
    }
}

void AcceleratorResourceManager::powerGateUnderutilizedInstances(AccelType type)
{
    auto it = accelerators.find(type);
    if (it == accelerators.end()) return;
    
    Tick now = curTick();
    
    for (auto &inst : it->second) {
        if (!inst.isBusy && inst.waitQueue.empty() && 
            inst.powerState != AccelPowerState::POWER_GATED) {
            
            if (now - inst.lastUsedTime > idleThreshold) {
                powerGateAccel(type, inst.instanceId);
            }
        }
    }
}

float AcceleratorResourceManager::getActivePower(AccelType type) const
{
    auto it = accelConfigs.find(type);
    return it != accelConfigs.end() ? it->second.activePowerMW : 0.0f;
}

float AcceleratorResourceManager::getIdlePower(AccelType type) const
{
    auto it = accelConfigs.find(type);
    return it != accelConfigs.end() ? it->second.idlePowerMW : 0.0f;
}

std::string AcceleratorResourceManager::accelTypeToString(AccelType type)
{
    switch (type) {
        case AccelType::TRIG_ACCEL: return "TRIG_ACCEL";
        case AccelType::MAT_ACCEL: return "MAT_ACCEL";
        case AccelType::VPU: return "VPU";
        case AccelType::NPU: return "NPU";
        default: return "UNKNOWN";
    }
}

std::string AcceleratorResourceManager::sharingModeToString(SharingMode mode)
{
    switch (mode) {
        case SharingMode::FULLY_SHARED: return "FULLY_SHARED";
        case SharingMode::FULLY_DEDICATED: return "FULLY_DEDICATED";
        case SharingMode::HYBRID_SHARED: return "HYBRID_SHARED";
        case SharingMode::ADAPTIVE: return "ADAPTIVE";
        default: return "UNKNOWN";
    }
}

} // namespace spacecraft
} // namespace gem5





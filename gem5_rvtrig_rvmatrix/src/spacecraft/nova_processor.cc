/*
 * NOVA Processor - Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/nova_processor.hh"
#include "base/trace.hh"
#include "debug/Spacecraft.hh"

namespace gem5
{

namespace spacecraft
{

NOVAProcessor::NOVAProcessor(const Params &p)
    : ClockedObject(p),
      powerMonitorEvent([this]{ monitorPower(); }, name() + ".powerMonitorEvent"),
      statsCollectEvent([this]{ collectStats(); }, name() + ".statsCollectEvent")
{
    // Parse configuration from parameters
    config.numHighPerfCores = p.num_high_perf_cores;
    config.numEnergyEffCores = p.num_energy_eff_cores;
    config.numTrigAccels = p.num_trig_accels;
    config.numMatAccels = p.num_mat_accels;
    config.numVPUs = p.num_vpus;
    config.numNPUs = p.num_npus;
    config.l1ICacheKB = p.l1i_cache_kb;
    config.l1DCacheKB = p.l1d_cache_kb;
    config.l2CacheKB = p.l2_cache_kb;
    config.l3CacheMB = p.l3_cache_mb;
    config.vpuScratchpadKB = p.vpu_scratchpad_kb;
    config.npuWeightCacheKB = p.npu_weight_cache_kb;
    config.npuActivationSpadKB = p.npu_activation_spad_kb;
    config.maxPowerWatts = p.max_power_watts;
    config.nominalPowerWatts = p.nominal_power_watts;
    config.minPowerWatts = p.min_power_watts;
    
    DPRINTF(Spacecraft, "NOVA Processor created:\n");
    DPRINTF(Spacecraft, "  High-Perf Cores: %d, Energy-Eff Cores: %d\n",
            config.numHighPerfCores, config.numEnergyEffCores);
    DPRINTF(Spacecraft, "  TrigAccels: %d, MatAccels: %d, VPUs: %d, NPUs: %d\n",
            config.numTrigAccels, config.numMatAccels, config.numVPUs, config.numNPUs);
    DPRINTF(Spacecraft, "  Power: %.1fW max, %.1fW nominal\n",
            config.maxPowerWatts, config.nominalPowerWatts);
    
    // Reset statistics
    resetStats();
}

NOVAProcessor::~NOVAProcessor()
{
}

void NOVAProcessor::init()
{
    ClockedObject::init();
    initializeComponents();
}

void NOVAProcessor::startup()
{
    ClockedObject::startup();
    
    connectComponents();
    registerISROTasks();
    
    // Schedule periodic events
    schedule(powerMonitorEvent, curTick() + 1000000000);  // 1ms
    schedule(statsCollectEvent, curTick() + 10000000000); // 10ms
    
    DPRINTF(Spacecraft, "NOVA Processor startup complete\n");
}

void NOVAProcessor::initializeComponents()
{
    // Note: In actual gem5 usage, these components would be created
    // and connected via Python configuration. This is a placeholder
    // for demonstration of the architecture.
    
    DPRINTF(Spacecraft, "Initializing NOVA Processor components\n");
}

void NOVAProcessor::connectComponents()
{
    // Connect phase manager to scheduler and accel manager
    if (missionManager && scheduler) {
        missionManager->setScheduler(scheduler.get());
    }
    
    if (missionManager && accelManager) {
        missionManager->setAcceleratorManager(accelManager.get());
    }
    
    if (scheduler && accelManager) {
        scheduler->setAcceleratorManager(accelManager.get());
    }
    
    // Register phase change callback
    if (missionManager) {
        missionManager->registerPhaseChangeCallback(
            [this](MissionPhase old, MissionPhase newPhase) {
                onPhaseChange(old, newPhase);
            });
    }
    
    DPRINTF(Spacecraft, "NOVA Processor components connected\n");
}

void NOVAProcessor::registerISROTasks()
{
    if (scheduler) {
        ISROTaskDefinitions::registerAllTasks(scheduler.get());
        DPRINTF(Spacecraft, "ISRO tasks registered with scheduler\n");
    }
}

void NOVAProcessor::setConfig(const NOVAConfig &cfg)
{
    config = cfg;
    
    // Apply configuration changes
    if (accelManager) {
        // Reconfigure accelerator manager
    }
    
    DPRINTF(Spacecraft, "NOVA Processor configuration updated\n");
}

VisionProcessingUnit* NOVAProcessor::getVPU(int idx)
{
    if (idx >= 0 && idx < vpus.size()) {
        return vpus[idx].get();
    }
    return nullptr;
}

NeuralProcessingUnit* NOVAProcessor::getNPU(int idx)
{
    if (idx >= 0 && idx < npus.size()) {
        return npus[idx].get();
    }
    return nullptr;
}

void NOVAProcessor::setMissionPhase(MissionPhase phase)
{
    if (missionManager) {
        missionManager->transitionToPhase(phase, "External command");
    }
}

MissionPhase NOVAProcessor::getMissionPhase() const
{
    if (missionManager) {
        return missionManager->getCurrentPhase();
    }
    return MissionPhase::NORMAL_OPS;
}

void NOVAProcessor::setPowerBudget(float watts)
{
    if (scheduler) {
        scheduler->setPowerBudget(watts);
    }
    if (missionManager) {
        missionManager->setPowerBudget(watts);
    }
    
    DPRINTF(Spacecraft, "Power budget set to %.1fW\n", watts);
}

float NOVAProcessor::getCurrentPower() const
{
    float totalPower = 0.0f;
    
    // Core power (placeholder)
    int totalCores = config.numHighPerfCores + config.numEnergyEffCores;
    totalPower += totalCores * 2.0f;  // Assume 2W per core
    
    // Accelerator power
    if (accelManager) {
        totalPower += accelManager->getCurrentPower();
    }
    
    return totalPower;
}

NOVAStats NOVAProcessor::getStats() const
{
    return stats;
}

void NOVAProcessor::resetStats()
{
    stats.totalCycles = 0;
    stats.totalInstructions = 0;
    stats.ipc = 0.0f;
    stats.accelUtilization.clear();
    stats.accelOperations.clear();
    stats.deadlineMisses = 0;
    stats.contextSwitches = 0;
    stats.preemptions = 0;
    stats.avgPowerWatts = 0.0f;
    stats.peakPowerWatts = 0.0f;
    stats.totalEnergyJoules = 0.0f;
    stats.timeInPhase.clear();
    stats.phaseTransitions = 0;
}

void NOVAProcessor::printStats()
{
    DPRINTF(Spacecraft, "=== NOVA Processor Statistics ===\n");
    DPRINTF(Spacecraft, "Total Cycles: %lu\n", stats.totalCycles);
    DPRINTF(Spacecraft, "Total Instructions: %lu\n", stats.totalInstructions);
    DPRINTF(Spacecraft, "IPC: %.2f\n", stats.ipc);
    DPRINTF(Spacecraft, "Deadline Misses: %lu\n", stats.deadlineMisses);
    DPRINTF(Spacecraft, "Context Switches: %lu\n", stats.contextSwitches);
    DPRINTF(Spacecraft, "Avg Power: %.2fW\n", stats.avgPowerWatts);
    DPRINTF(Spacecraft, "Total Energy: %.2fJ\n", stats.totalEnergyJoules);
    
    if (scheduler) {
        scheduler->printStatistics();
    }
    
    if (accelManager) {
        accelManager->printStatistics();
    }
}

void NOVAProcessor::runVisionNavPipeline(Addr imageAddr, Addr resultAddr)
{
    // Vision-based navigation pipeline:
    // 1. VPU: Feature extraction from camera image
    // 2. NPU: Crater/landmark classification
    // 3. MatAccel: Pose estimation (PnP)
    // 4. TrigAccel: Coordinate transformation
    
    DPRINTF(Spacecraft, "Starting Vision-Nav Pipeline: image@0x%lx -> result@0x%lx\n",
            imageAddr, resultAddr);
    
    // This would trigger the actual accelerator operations
    // For now, just log the intent
    
    if (!vpus.empty() && vpus[0]) {
        VPURequest vreq;
        vreq.requestId = 0;
        vreq.operation = VPUOpType::ORB_FEATURES;
        vreq.width = 640;
        vreq.height = 480;
        vreq.channels = 1;
        vreq.inputAddr = imageAddr;
        vpus[0]->submitRequest(vreq);
    }
}

void NOVAProcessor::onPhaseChange(MissionPhase oldPhase, MissionPhase newPhase)
{
    DPRINTF(Spacecraft, "Phase change: %s -> %s\n",
            missionManager ? missionManager->getPhaseString().c_str() : "?",
            missionManager ? missionManager->getPhaseString().c_str() : "?");
    
    stats.phaseTransitions++;
    
    // Reconfigure accelerator manager for new phase
    if (accelManager) {
        accelManager->reconfigure(newPhase);
    }
    
    // Update power budget based on phase
    if (missionManager) {
        float newBudget = missionManager->getCurrentPowerBudget();
        setPowerBudget(newBudget);
    }
}

void NOVAProcessor::monitorPower()
{
    float currentPower = getCurrentPower();
    
    // Track peak
    if (currentPower > stats.peakPowerWatts) {
        stats.peakPowerWatts = currentPower;
    }
    
    // Update running average
    stats.avgPowerWatts = (stats.avgPowerWatts * 0.99f) + (currentPower * 0.01f);
    
    // Accumulate energy
    stats.totalEnergyJoules += currentPower * 0.001f;  // 1ms sample period
    
    // Check for power violations
    if (currentPower > config.maxPowerWatts) {
        DPRINTF(Spacecraft, "WARNING: Power violation! Current: %.1fW, Max: %.1fW\n",
                currentPower, config.maxPowerWatts);
        
        // Trigger power reduction
        if (accelManager) {
            accelManager->powerGateNonEssential();
        }
    }
    
    // Reschedule
    schedule(powerMonitorEvent, curTick() + 1000000000);  // 1ms
}

void NOVAProcessor::collectStats()
{
    // Collect statistics from all components
    
    if (scheduler) {
        const SchedulerStats &schedStats = scheduler->getStats();
        stats.deadlineMisses = schedStats.totalDeadlineMisses;
        stats.contextSwitches = schedStats.totalContextSwitches;
        stats.preemptions = schedStats.totalPreemptions;
    }
    
    if (missionManager) {
        stats.phaseTransitions = missionManager->getPhaseTransitionCount();
    }
    
    // Calculate IPC (placeholder - would come from CPU stats)
    if (stats.totalCycles > 0) {
        stats.ipc = (float)stats.totalInstructions / stats.totalCycles;
    }
    
    // Reschedule
    schedule(statsCollectEvent, curTick() + 10000000000);  // 10ms
}

} // namespace spacecraft
} // namespace gem5





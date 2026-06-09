/*
 * NOVA Processor - Navigation-Optimized Vision-Augmented Processor
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * Main processor class that integrates all components:
 * - RISC-V cores (RV64GC)
 * - Navigation accelerators (TrigAccel, MatAccel)
 * - Vision Processing Unit (VPU)
 * - Neural Processing Unit (NPU)
 * - Mission Phase Manager
 * - Global Task Scheduler
 * - Accelerator Resource Manager
 */

#ifndef __SPACECRAFT_NOVA_PROCESSOR_HH__
#define __SPACECRAFT_NOVA_PROCESSOR_HH__

#include "params/NOVAProcessor.hh"
#include "sim/clocked_object.hh"

#include "spacecraft/vision_processing_unit.hh"
#include "spacecraft/neural_processing_unit.hh"
#include "spacecraft/mission_phase_manager.hh"
#include "spacecraft/global_task_scheduler.hh"
#include "spacecraft/accel_resource_manager.hh"
#include "spacecraft/isro_task_definitions.hh"

#include <vector>
#include <memory>

namespace gem5
{

namespace spacecraft
{

/**
 * NOVA Processor Configuration
 */
struct NOVAConfig {
    // Core configuration
    int numHighPerfCores;       // For critical navigation/control
    int numEnergyEffCores;      // For background tasks
    
    // Accelerator counts
    int numTrigAccels;
    int numMatAccels;
    int numVPUs;
    int numNPUs;
    
    // Memory configuration
    int l1ICacheKB;
    int l1DCacheKB;
    int l2CacheKB;
    int l3CacheMB;
    int vpuScratchpadKB;
    int npuWeightCacheKB;
    int npuActivationSpadKB;
    
    // Power configuration
    float maxPowerWatts;
    float nominalPowerWatts;
    float minPowerWatts;
    
    // Default values
    NOVAConfig()
        : numHighPerfCores(4), numEnergyEffCores(0),
          numTrigAccels(2), numMatAccels(2), numVPUs(1), numNPUs(1),
          l1ICacheKB(32), l1DCacheKB(32), l2CacheKB(512), l3CacheMB(2),
          vpuScratchpadKB(64), npuWeightCacheKB(64), npuActivationSpadKB(32),
          maxPowerWatts(100), nominalPowerWatts(50), minPowerWatts(10) {}
};

/**
 * NOVA Processor Statistics
 */
struct NOVAStats {
    // Core statistics
    uint64_t totalCycles;
    uint64_t totalInstructions;
    float ipc;
    
    // Accelerator statistics
    std::map<std::string, float> accelUtilization;
    std::map<std::string, uint64_t> accelOperations;
    
    // Scheduling statistics
    uint64_t deadlineMisses;
    uint64_t contextSwitches;
    uint64_t preemptions;
    
    // Power statistics
    float avgPowerWatts;
    float peakPowerWatts;
    float totalEnergyJoules;
    
    // Mission statistics
    std::map<std::string, Tick> timeInPhase;
    uint64_t phaseTransitions;
};

/**
 * NOVA Processor - Main Processor Class
 */
class NOVAProcessor : public ClockedObject
{
  public:
    PARAMS(NOVAProcessor);
    NOVAProcessor(const Params &p);
    ~NOVAProcessor();
    
    // Lifecycle
    void startup() override;
    void init() override;
    
    // Configuration
    NOVAConfig getConfig() const { return config; }
    void setConfig(const NOVAConfig &cfg);
    
    // Component access
    MissionPhaseManager* getMissionManager() { return missionManager.get(); }
    GlobalTaskScheduler* getScheduler() { return scheduler.get(); }
    AcceleratorResourceManager* getAccelManager() { return accelManager.get(); }
    
    // Vision pipeline
    VisionProcessingUnit* getVPU(int idx = 0);
    NeuralProcessingUnit* getNPU(int idx = 0);
    
    // Mission control
    void setMissionPhase(MissionPhase phase);
    MissionPhase getMissionPhase() const;
    
    // Power management
    void setPowerBudget(float watts);
    float getCurrentPower() const;
    
    // Statistics
    NOVAStats getStats() const;
    void resetStats();
    void printStats();
    
    // Vision-based navigation pipeline
    // Camera → VPU → NPU → MatAccel → TrigAccel → Navigation solution
    void runVisionNavPipeline(Addr imageAddr, Addr resultAddr);
    
  private:
    // Configuration
    NOVAConfig config;
    
    // Core components (managed externally by gem5 system)
    // Just track references here
    
    // Accelerators
    std::vector<std::unique_ptr<VisionProcessingUnit>> vpus;
    std::vector<std::unique_ptr<NeuralProcessingUnit>> npus;
    
    // Management components
    std::unique_ptr<MissionPhaseManager> missionManager;
    std::unique_ptr<GlobalTaskScheduler> scheduler;
    std::unique_ptr<AcceleratorResourceManager> accelManager;
    
    // Statistics tracking
    NOVAStats stats;
    
    // Events
    EventFunctionWrapper powerMonitorEvent;
    EventFunctionWrapper statsCollectEvent;
    
    // Internal methods
    void initializeComponents();
    void connectComponents();
    void registerISROTasks();
    void monitorPower();
    void collectStats();
    
    // Phase change callback
    void onPhaseChange(MissionPhase oldPhase, MissionPhase newPhase);
};

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_NOVA_PROCESSOR_HH__





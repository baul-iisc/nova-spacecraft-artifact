/*
 * Autonomous Mission Adapter for Spacecraft Processor
 * 
 * Automatically adapts processor configuration based on mission phase:
 * - Scheduling policy selection
 * - Power management strategy
 * - Accelerator prioritization
 * - Resource allocation
 * 
 * Part of PhD Research: Autonomous Mission Adaptation for Spacecraft
 */

#ifndef __CPU_MISSION_ADAPTER_MISSION_ADAPTER_HH__
#define __CPU_MISSION_ADAPTER_MISSION_ADAPTER_HH__

#include "../power_manager/power_manager.hh"
#include "../rt_scheduler/rt_scheduler.hh"
#include <cstdint>
#include <string>
#include <functional>
#include <vector>

namespace gem5 {

//==============================================================================
// Mission Phases
//==============================================================================

enum class MissionPhase : uint8_t {
    LAUNCH,             // High-vibration, critical monitoring
    ASCENT,             // Aerodynamic stress, rapid telemetry
    ORBIT_INSERTION,    // Critical burns, precise navigation
    CRUISE,             // Low activity, power saving
    APPROACH,           // Increasing activity, target tracking
    ORBIT_OPERATIONS,   // Nominal science, periodic maneuvers
    DESCENT,            // Critical phase, maximum responsiveness
    LANDING,            // Ultra-critical, real-time control
    SURFACE_OPERATIONS, // Science-focused, power-limited
    SAFE_MODE,          // Minimum operations, survival
    COMMUNICATION_WINDOW, // High-bandwidth data transfer
    ECLIPSE,            // Battery-only, power-critical
    NUM_PHASES
};

//==============================================================================
// Mission Phase Configuration
//==============================================================================

struct MissionPhaseConfig {
    const char* name;
    PowerPolicy powerPolicy;
    SchedulingPolicy schedulingPolicy;
    double energyBudgetFactor;       // Multiplier for baseline budget
    uint8_t maxActiveCores;          // Limit active cores
    bool allowTurboMode;             // TURBO power state allowed?
    bool enableNonCriticalAccel;     // Allow best-effort accelerators?
    uint32_t maxLatencyToleranceUs;  // Maximum tolerable latency
    
    // Accelerator priority overrides (higher = more priority)
    uint8_t accelPriority[7];  // Per accelerator type
};

constexpr MissionPhaseConfig PHASE_CONFIGS[] = {
    // LAUNCH - Survival mode, minimum operations
    {
        .name = "LAUNCH",
        .powerPolicy = PowerPolicy::POWER_SAVER,
        .schedulingPolicy = SchedulingPolicy::CA_RM,
        .energyBudgetFactor = 0.5,
        .maxActiveCores = 2,
        .allowTurboMode = false,
        .enableNonCriticalAccel = false,
        .maxLatencyToleranceUs = 1000,
        .accelPriority = {10, 8, 5, 3, 7, 0, 6}  // CORDIC, Matrix high
    },
    // ASCENT - Telemetry-focused
    {
        .name = "ASCENT",
        .powerPolicy = PowerPolicy::BALANCED,
        .schedulingPolicy = SchedulingPolicy::CA_EDF,
        .energyBudgetFactor = 0.8,
        .maxActiveCores = 4,
        .allowTurboMode = false,
        .enableNonCriticalAccel = false,
        .maxLatencyToleranceUs = 500,
        .accelPriority = {8, 7, 10, 5, 6, 0, 5}  // SpaceWire high
    },
    // ORBIT_INSERTION - Critical burns
    {
        .name = "ORBIT_INSERTION",
        .powerPolicy = PowerPolicy::PERFORMANCE,
        .schedulingPolicy = SchedulingPolicy::CA_EDF,
        .energyBudgetFactor = 1.5,
        .maxActiveCores = 8,
        .allowTurboMode = true,
        .enableNonCriticalAccel = false,
        .maxLatencyToleranceUs = 100,
        .accelPriority = {10, 10, 5, 4, 6, 0, 10}  // GNC critical
    },
    // CRUISE - Power conservation
    {
        .name = "CRUISE",
        .powerPolicy = PowerPolicy::AGGRESSIVE_SAVING,
        .schedulingPolicy = SchedulingPolicy::CA_RM,
        .energyBudgetFactor = 0.3,
        .maxActiveCores = 1,
        .allowTurboMode = false,
        .enableNonCriticalAccel = true,
        .maxLatencyToleranceUs = 10000,
        .accelPriority = {3, 2, 2, 2, 4, 5, 2}  // Low across board
    },
    // APPROACH - Increasing activity
    {
        .name = "APPROACH",
        .powerPolicy = PowerPolicy::BALANCED,
        .schedulingPolicy = SchedulingPolicy::CA_EDF,
        .energyBudgetFactor = 1.0,
        .maxActiveCores = 4,
        .allowTurboMode = true,
        .enableNonCriticalAccel = true,
        .maxLatencyToleranceUs = 500,
        .accelPriority = {8, 8, 5, 4, 5, 6, 8}
    },
    // ORBIT_OPERATIONS - Science focus
    {
        .name = "ORBIT_OPERATIONS",
        .powerPolicy = PowerPolicy::BALANCED,
        .schedulingPolicy = SchedulingPolicy::CA_RM,
        .energyBudgetFactor = 1.0,
        .maxActiveCores = 4,
        .allowTurboMode = false,
        .enableNonCriticalAccel = true,
        .maxLatencyToleranceUs = 2000,
        .accelPriority = {5, 5, 7, 8, 6, 10, 5}  // Image, RS high
    },
    // DESCENT - Critical phase
    {
        .name = "DESCENT",
        .powerPolicy = PowerPolicy::PERFORMANCE,
        .schedulingPolicy = SchedulingPolicy::CA_EDF,
        .energyBudgetFactor = 1.5,
        .maxActiveCores = 8,
        .allowTurboMode = true,
        .enableNonCriticalAccel = false,
        .maxLatencyToleranceUs = 100,
        .accelPriority = {10, 10, 4, 3, 5, 0, 10}
    },
    // LANDING - Ultra-critical
    {
        .name = "LANDING",
        .powerPolicy = PowerPolicy::PERFORMANCE,
        .schedulingPolicy = SchedulingPolicy::CA_LLF,
        .energyBudgetFactor = 2.0,
        .maxActiveCores = 8,
        .allowTurboMode = true,
        .enableNonCriticalAccel = false,
        .maxLatencyToleranceUs = 50,
        .accelPriority = {10, 10, 3, 2, 4, 0, 10}  // GNC/Kalman max
    },
    // SURFACE_OPERATIONS - Science, power-limited
    {
        .name = "SURFACE_OPERATIONS",
        .powerPolicy = PowerPolicy::POWER_SAVER,
        .schedulingPolicy = SchedulingPolicy::CA_RM,
        .energyBudgetFactor = 0.6,
        .maxActiveCores = 2,
        .allowTurboMode = false,
        .enableNonCriticalAccel = true,
        .maxLatencyToleranceUs = 5000,
        .accelPriority = {4, 4, 6, 7, 5, 10, 3}  // Image high
    },
    // SAFE_MODE - Survival
    {
        .name = "SAFE_MODE",
        .powerPolicy = PowerPolicy::AGGRESSIVE_SAVING,
        .schedulingPolicy = SchedulingPolicy::RM,
        .energyBudgetFactor = 0.2,
        .maxActiveCores = 1,
        .allowTurboMode = false,
        .enableNonCriticalAccel = false,
        .maxLatencyToleranceUs = 50000,
        .accelPriority = {5, 3, 8, 2, 3, 0, 2}  // Comms only
    },
    // COMMUNICATION_WINDOW - High bandwidth
    {
        .name = "COMMUNICATION_WINDOW",
        .powerPolicy = PowerPolicy::BALANCED,
        .schedulingPolicy = SchedulingPolicy::CA_EDF,
        .energyBudgetFactor = 1.2,
        .maxActiveCores = 4,
        .allowTurboMode = true,
        .enableNonCriticalAccel = true,
        .maxLatencyToleranceUs = 1000,
        .accelPriority = {3, 3, 10, 10, 8, 8, 2}  // Comms, RS, AES high
    },
    // ECLIPSE - Battery only
    {
        .name = "ECLIPSE",
        .powerPolicy = PowerPolicy::AGGRESSIVE_SAVING,
        .schedulingPolicy = SchedulingPolicy::CA_RM,
        .energyBudgetFactor = 0.25,
        .maxActiveCores = 1,
        .allowTurboMode = false,
        .enableNonCriticalAccel = false,
        .maxLatencyToleranceUs = 20000,
        .accelPriority = {4, 3, 3, 2, 3, 0, 4}
    }
};

//==============================================================================
// Mission Event (for autonomous transitions)
//==============================================================================

struct MissionEvent {
    uint64_t triggerTick;          // When to trigger
    MissionPhase targetPhase;      // Phase to transition to
    const char* description;
    bool executed;
    
    MissionEvent(uint64_t tick, MissionPhase phase, const char* desc)
        : triggerTick(tick), targetPhase(phase), description(desc), executed(false)
    {}
};

//==============================================================================
// Mission Adapter Class
//==============================================================================

class MissionAdapter {
private:
    MissionPhase currentPhase;
    const MissionPhaseConfig* currentConfig;
    
    PowerManager* powerManager;
    RTScheduler* rtScheduler;
    
    // Scheduled events
    std::vector<MissionEvent> scheduledEvents;
    
    // Statistics
    uint64_t phaseTransitions;
    uint64_t autonomousDecisions;
    uint64_t currentPhaseDuration;
    uint64_t phaseStartTick;
    
    // Baseline energy budget (mission-specific)
    double baselineEnergyBudgetMicroJ;

public:
    MissionAdapter(PowerManager* pm = nullptr, RTScheduler* rts = nullptr)
        : currentPhase(MissionPhase::CRUISE)
        , currentConfig(&PHASE_CONFIGS[static_cast<int>(MissionPhase::CRUISE)])
        , powerManager(pm)
        , rtScheduler(rts)
        , phaseTransitions(0)
        , autonomousDecisions(0)
        , currentPhaseDuration(0)
        , phaseStartTick(0)
        , baselineEnergyBudgetMicroJ(1000000.0)
    {}
    
    //==========================================================================
    // Phase Management
    //==========================================================================
    
    /**
     * Transition to a new mission phase
     */
    void transitionToPhase(MissionPhase newPhase, uint64_t currentTick) {
        if (newPhase == currentPhase) return;
        
        // Update duration tracking
        currentPhaseDuration = currentTick - phaseStartTick;
        phaseStartTick = currentTick;
        
        // Update phase
        currentPhase = newPhase;
        currentConfig = &PHASE_CONFIGS[static_cast<int>(newPhase)];
        phaseTransitions++;
        
        // Apply configuration
        applyPhaseConfiguration();
        
        std::cout << "[MissionAdapter] Phase transition: " 
                  << currentConfig->name << " at tick " << currentTick << "\n";
    }
    
    /**
     * Get current mission phase
     */
    MissionPhase getCurrentPhase() const {
        return currentPhase;
    }
    
    /**
     * Get current phase configuration
     */
    const MissionPhaseConfig& getPhaseConfig() const {
        return *currentConfig;
    }
    
    //==========================================================================
    // Scheduled Events
    //==========================================================================
    
    /**
     * Schedule a phase transition at a specific tick
     */
    void schedulePhaseTransition(uint64_t tick, MissionPhase phase, const char* desc) {
        scheduledEvents.emplace_back(tick, phase, desc);
    }
    
    /**
     * Process any pending scheduled events
     */
    void processEvents(uint64_t currentTick) {
        for (auto& event : scheduledEvents) {
            if (!event.executed && currentTick >= event.triggerTick) {
                std::cout << "[MissionAdapter] Autonomous event: " 
                          << event.description << "\n";
                transitionToPhase(event.targetPhase, currentTick);
                event.executed = true;
                autonomousDecisions++;
            }
        }
    }
    
    //==========================================================================
    // Autonomous Decisions
    //==========================================================================
    
    /**
     * Check if we should transition based on system state
     * Called periodically to enable autonomous adaptation
     */
    void evaluateAutonomousTransition(uint64_t currentTick, 
                                       double currentPowerMW,
                                       double remainingEnergyMicroJ) {
        // Example autonomous decisions:
        
        // 1. If power budget critical, go to SAFE_MODE
        if (remainingEnergyMicroJ < baselineEnergyBudgetMicroJ * 0.05) {
            if (currentPhase != MissionPhase::SAFE_MODE) {
                std::cout << "[MissionAdapter] AUTONOMOUS: Low energy, entering SAFE_MODE\n";
                transitionToPhase(MissionPhase::SAFE_MODE, currentTick);
                autonomousDecisions++;
            }
        }
        
        // 2. If in ECLIPSE and power is OK, can do limited operations
        if (currentPhase == MissionPhase::ECLIPSE) {
            if (currentPowerMW < 50.0 && remainingEnergyMicroJ > baselineEnergyBudgetMicroJ * 0.3) {
                // Could upgrade to CRUISE if conditions permit
            }
        }
    }
    
    /**
     * Determine if a task should be allowed based on current phase
     */
    bool shouldAllowTask(uint8_t criticalityLevel, AcceleratorType accelType) const {
        // Safety-critical always allowed
        if (criticalityLevel >= 2) return true;
        
        // Check if non-critical accelerators are enabled
        if (!currentConfig->enableNonCriticalAccel) {
            const auto& spec = getAccelSpec(accelType);
            if (spec.criticalityLevel == 0) return false;
        }
        
        return true;
    }
    
    /**
     * Get priority adjustment for an accelerator in current phase
     */
    uint8_t getAcceleratorPriority(AcceleratorType type) const {
        return currentConfig->accelPriority[static_cast<int>(type)];
    }
    
    //==========================================================================
    // Configuration
    //==========================================================================
    
    void setPowerManager(PowerManager* pm) {
        powerManager = pm;
        if (pm) applyPhaseConfiguration();
    }
    
    void setRTScheduler(RTScheduler* rts) {
        rtScheduler = rts;
        if (rts) applyPhaseConfiguration();
    }
    
    void setBaselineEnergyBudget(double microJ) {
        baselineEnergyBudgetMicroJ = microJ;
    }
    
    //==========================================================================
    // Statistics
    //==========================================================================
    
    void printStatistics() const {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║           MISSION ADAPTER STATISTICS                             ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ Current Phase:      " << std::setw(20) << currentConfig->name 
                  << "                      ║\n";
        std::cout << "║ Phase Transitions:  " << std::setw(10) << phaseTransitions 
                  << "                                  ║\n";
        std::cout << "║ Autonomous Decisions: " << std::setw(8) << autonomousDecisions 
                  << "                                  ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ PHASE CONFIGURATION                                              ║\n";
        std::cout << "║   Power Policy:     " << std::setw(20) 
                  << getPolicyNameStr(currentConfig->powerPolicy) << "                      ║\n";
        std::cout << "║   Scheduling:       " << std::setw(20)
                  << getSchedulingPolicyName(currentConfig->schedulingPolicy) << "                      ║\n";
        std::cout << "║   Max Active Cores: " << std::setw(10) 
                  << (int)currentConfig->maxActiveCores << "                                  ║\n";
        std::cout << "║   Turbo Allowed:    " << std::setw(10)
                  << (currentConfig->allowTurboMode ? "YES" : "NO") << "                                  ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
    }

private:
    void applyPhaseConfiguration() {
        // Apply power policy
        if (powerManager) {
            powerManager->setPolicy(currentConfig->powerPolicy);
            
            double phaseBudget = baselineEnergyBudgetMicroJ * currentConfig->energyBudgetFactor;
            powerManager->setEnergyBudget(phaseBudget);
        }
        
        // Apply scheduling policy
        if (rtScheduler) {
            rtScheduler->setPolicy(currentConfig->schedulingPolicy);
        }
    }
    
    static const char* getPolicyNameStr(PowerPolicy policy) {
        switch (policy) {
            case PowerPolicy::PERFORMANCE: return "PERFORMANCE";
            case PowerPolicy::BALANCED: return "BALANCED";
            case PowerPolicy::POWER_SAVER: return "POWER_SAVER";
            case PowerPolicy::AGGRESSIVE_SAVING: return "AGGRESSIVE_SAVING";
            case PowerPolicy::MISSION_ADAPTIVE: return "MISSION_ADAPTIVE";
            default: return "UNKNOWN";
        }
    }
    
    static const char* getSchedulingPolicyName(SchedulingPolicy policy) {
        switch (policy) {
            case SchedulingPolicy::CA_RM: return "CA-RM";
            case SchedulingPolicy::CA_EDF: return "CA-EDF";
            case SchedulingPolicy::CA_LLF: return "CA-LLF";
            case SchedulingPolicy::RM: return "RM";
            case SchedulingPolicy::EDF: return "EDF";
            default: return "UNKNOWN";
        }
    }
};

} // namespace gem5

#endif // __CPU_MISSION_ADAPTER_MISSION_ADAPTER_HH__

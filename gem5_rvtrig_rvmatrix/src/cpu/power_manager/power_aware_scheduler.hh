/*
 * Power-Aware Scheduling Algorithms
 * 
 * Scheduling policies that optimize for energy efficiency:
 * - MED: Minimum Energy under Deadline
 * - Race-to-Idle: Complete quickly then sleep
 * - Pace-to-Idle: Spread work to minimize peak power
 * - Energy-Proportional: Scale with utilization
 * 
 * Part of PhD Research: Power-Aware Scheduling for Heterogeneous Accelerators
 */

#ifndef __CPU_POWER_MANAGER_POWER_AWARE_SCHEDULER_HH__
#define __CPU_POWER_MANAGER_POWER_AWARE_SCHEDULER_HH__

#include "power_manager.hh"
#include "spacecraft_accelerators.hh"
#include "../rt_scheduler/rt_scheduler.hh"
#include <vector>
#include <algorithm>
#include <cmath>

namespace gem5 {

//==============================================================================
// Power-Aware Scheduling Policies
//==============================================================================

enum class PowerAwarePolicy : uint8_t {
    // Deadline-aware policies
    MED,                // Minimum Energy under Deadline
    MED_CONTENTION,     // MED with contention awareness
    
    // Sleep-oriented policies  
    RACE_TO_IDLE,       // Complete fast, sleep long
    PACE_TO_IDLE,       // Spread work, avoid peaks
    
    // Proportional policies
    ENERGY_PROPORTIONAL,  // Scale power with load
    
    // Hybrid policies
    HYBRID_CRITICAL,    // Performance for critical, save for others
    
    NUM_POLICIES
};

//==============================================================================
// Task Energy Profile
//==============================================================================

struct TaskEnergyProfile {
    uint32_t taskId;
    
    // Base execution characteristics
    uint64_t baseWCET;              // Cycles at NOMINAL
    uint64_t deadline;              // Absolute deadline (cycles)
    uint8_t criticalityLevel;       // 0-2
    
    // Accelerator usage
    AcceleratorType primaryAccel;
    uint32_t accelInvocations;      // Number of accelerator calls
    double accelUtilization;        // Fraction of time using accelerator
    
    // Energy estimates at different power states
    double energyMicroJ[6];         // Energy for each PowerState
    uint64_t executionCycles[6];    // Execution time for each PowerState
    
    // Computed optimal settings
    PowerState optimalPowerState;
    double optimalEnergy;
    uint64_t optimalExecTime;
    bool meetsDeadline;
    
    TaskEnergyProfile()
        : taskId(0)
        , baseWCET(0)
        , deadline(0)
        , criticalityLevel(0)
        , primaryAccel(AcceleratorType::CORDIC)
        , accelInvocations(0)
        , accelUtilization(0.0)
        , optimalPowerState(PowerState::NOMINAL)
        , optimalEnergy(0.0)
        , optimalExecTime(0)
        , meetsDeadline(true)
    {
        for (int i = 0; i < 6; i++) {
            energyMicroJ[i] = 0.0;
            executionCycles[i] = 0;
        }
    }
};

//==============================================================================
// Power-Aware Scheduler Class
//==============================================================================

class PowerAwareScheduler {
private:
    PowerManager* powerManager;
    RTScheduler* rtScheduler;
    
    PowerAwarePolicy currentPolicy;
    
    // Task energy profiles
    std::vector<TaskEnergyProfile> taskProfiles;
    
    // Statistics
    uint64_t schedulingDecisions;
    double totalEnergySaved;        // vs PERFORMANCE baseline
    uint64_t deadlinesMet;
    uint64_t deadlinesMissed;
    
    // Configuration
    double energySavingWeight;      // 0.0-1.0, higher = more energy savings
    double deadlineMargin;          // Safety margin for deadlines

public:
    PowerAwareScheduler(PowerManager* pm = nullptr, RTScheduler* rts = nullptr)
        : powerManager(pm)
        , rtScheduler(rts)
        , currentPolicy(PowerAwarePolicy::MED_CONTENTION)
        , schedulingDecisions(0)
        , totalEnergySaved(0.0)
        , deadlinesMet(0)
        , deadlinesMissed(0)
        , energySavingWeight(0.7)
        , deadlineMargin(0.1)
    {}
    
    //==========================================================================
    // Task Profile Management
    //==========================================================================
    
    /**
     * Create energy profile for a task
     */
    TaskEnergyProfile createTaskProfile(
        uint32_t taskId,
        uint64_t wcet,
        uint64_t deadline,
        uint8_t criticality,
        AcceleratorType accel,
        uint32_t accelCalls,
        double accelUtil)
    {
        TaskEnergyProfile profile;
        profile.taskId = taskId;
        profile.baseWCET = wcet;
        profile.deadline = deadline;
        profile.criticalityLevel = criticality;
        profile.primaryAccel = accel;
        profile.accelInvocations = accelCalls;
        profile.accelUtilization = accelUtil;
        
        // Calculate energy and execution time for each power state
        const auto& accelSpec = getAccelSpec(accel);
        
        for (int s = 0; s < 6; s++) {
            PowerState state = static_cast<PowerState>(s);
            const auto& dvfs = DVFS_LEVELS[s];
            
            if (dvfs.frequencyRatio <= 0) {
                profile.executionCycles[s] = UINT64_MAX;
                profile.energyMicroJ[s] = 0.0;
                continue;
            }
            
            // Scale execution time by frequency
            double freqScale = 1.0 / dvfs.frequencyRatio;
            profile.executionCycles[s] = static_cast<uint64_t>(wcet * freqScale);
            
            // Calculate energy: Power * Time
            // Power includes CPU (assumed constant) + accelerator usage
            double cpuPowerMW = 100.0 * dvfs.powerRatio;  // Base CPU power
            double accelPowerMW = accelSpec.powerMW[s] * accelUtil;
            double totalPowerMW = cpuPowerMW + accelPowerMW;
            
            // Energy = Power * Time (convert cycles to ns at 1GHz)
            double execTimeNs = profile.executionCycles[s];  // Assume 1GHz
            profile.energyMicroJ[s] = totalPowerMW * execTimeNs / 1000.0;
        }
        
        // Find optimal power state
        computeOptimalPowerState(profile);
        
        return profile;
    }
    
    /**
     * Register a task profile
     */
    void registerTask(const TaskEnergyProfile& profile) {
        taskProfiles.push_back(profile);
    }
    
    //==========================================================================
    // Scheduling Decisions
    //==========================================================================
    
    /**
     * Decide power state for a task based on current policy
     */
    PowerState decidePowerState(const TaskEnergyProfile& profile, 
                                 uint64_t currentTime,
                                 double remainingEnergy) {
        schedulingDecisions++;
        
        switch (currentPolicy) {
            case PowerAwarePolicy::MED:
                return decideMED(profile, currentTime, remainingEnergy);
                
            case PowerAwarePolicy::MED_CONTENTION:
                return decideMEDWithContention(profile, currentTime, remainingEnergy);
                
            case PowerAwarePolicy::RACE_TO_IDLE:
                return decideRaceToIdle(profile, currentTime);
                
            case PowerAwarePolicy::PACE_TO_IDLE:
                return decidePaceToIdle(profile, currentTime);
                
            case PowerAwarePolicy::ENERGY_PROPORTIONAL:
                return decideEnergyProportional(profile, remainingEnergy);
                
            case PowerAwarePolicy::HYBRID_CRITICAL:
                return decideHybridCritical(profile, currentTime);
                
            default:
                return PowerState::NOMINAL;
        }
    }
    
    /**
     * Get accelerator power state recommendation for upcoming operations
     */
    PowerState recommendAccelPowerState(AcceleratorType type, 
                                         uint32_t expectedCalls,
                                         uint64_t timeWindow) {
        // If few calls expected, keep in lower power state
        double callDensity = static_cast<double>(expectedCalls) / timeWindow;
        
        if (callDensity < 0.001) {
            return PowerState::SLEEP;
        } else if (callDensity < 0.01) {
            return PowerState::IDLE;
        } else if (callDensity < 0.1) {
            return PowerState::LOW_POWER;
        } else if (callDensity < 0.5) {
            return PowerState::NOMINAL;
        } else {
            return PowerState::TURBO;
        }
    }
    
    //==========================================================================
    // Policy Configuration
    //==========================================================================
    
    void setPolicy(PowerAwarePolicy policy) {
        currentPolicy = policy;
    }
    
    PowerAwarePolicy getPolicy() const {
        return currentPolicy;
    }
    
    void setEnergySavingWeight(double weight) {
        energySavingWeight = std::max(0.0, std::min(1.0, weight));
    }
    
    void setDeadlineMargin(double margin) {
        deadlineMargin = std::max(0.0, std::min(0.5, margin));
    }
    
    void setPowerManager(PowerManager* pm) {
        powerManager = pm;
    }
    
    void setRTScheduler(RTScheduler* rts) {
        rtScheduler = rts;
    }
    
    //==========================================================================
    // Statistics
    //==========================================================================
    
    void recordDeadlineResult(bool met) {
        if (met) deadlinesMet++;
        else deadlinesMissed++;
    }
    
    void recordEnergySaving(double savingMicroJ) {
        totalEnergySaved += savingMicroJ;
    }
    
    void printStatistics() const {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║        POWER-AWARE SCHEDULER STATISTICS                          ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ Policy: " << std::setw(20) << getPolicyName(currentPolicy)
                  << "                              ║\n";
        std::cout << "║ Scheduling Decisions: " << std::setw(15) << schedulingDecisions
                  << "                        ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ DEADLINE PERFORMANCE                                             ║\n";
        std::cout << "║   Met:    " << std::setw(15) << deadlinesMet << "                              ║\n";
        std::cout << "║   Missed: " << std::setw(15) << deadlinesMissed << "                              ║\n";
        
        double successRate = (deadlinesMet + deadlinesMissed > 0) 
            ? (100.0 * deadlinesMet / (deadlinesMet + deadlinesMissed)) : 100.0;
        std::cout << "║   Rate:   " << std::setw(14) << std::fixed << std::setprecision(1) 
                  << successRate << "%                              ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ ENERGY SAVINGS                                                   ║\n";
        std::cout << "║   Total Saved: " << std::setw(15) << std::setprecision(2) 
                  << totalEnergySaved << " µJ                       ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
    }

private:
    //==========================================================================
    // Policy Implementations
    //==========================================================================
    
    /**
     * MED: Minimum Energy under Deadline
     * Find lowest power state that still meets deadline
     */
    PowerState decideMED(const TaskEnergyProfile& profile, 
                          uint64_t currentTime,
                          double remainingEnergy) {
        uint64_t slack = 0;
        if (profile.deadline > currentTime) {
            slack = profile.deadline - currentTime;
        }
        
        // Add safety margin
        uint64_t effectiveDeadline = static_cast<uint64_t>(slack * (1.0 - deadlineMargin));
        
        // Find lowest power state that meets deadline
        for (int s = static_cast<int>(PowerState::LOW_POWER); 
             s <= static_cast<int>(PowerState::TURBO); s++) {
            if (profile.executionCycles[s] <= effectiveDeadline) {
                return static_cast<PowerState>(s);
            }
        }
        
        // Can't meet deadline, use TURBO
        return PowerState::TURBO;
    }
    
    /**
     * MED with Contention Awareness
     * Account for expected contention delays
     */
    PowerState decideMEDWithContention(const TaskEnergyProfile& profile,
                                        uint64_t currentTime,
                                        double remainingEnergy) {
        uint64_t slack = 0;
        if (profile.deadline > currentTime) {
            slack = profile.deadline - currentTime;
        }
        
        // Estimate contention overhead (use RT scheduler if available)
        double contentionFactor = 1.0;
        if (rtScheduler) {
            // Get contention predictor estimate
            // Assuming 4 cores and moderate utilization
            contentionFactor = 1.0 + rtScheduler->getContentionPredictor()
                .predictOverhead(4, profile.accelUtilization) / 100.0;
        }
        
        // Adjust deadline for contention
        uint64_t effectiveDeadline = static_cast<uint64_t>(
            slack * (1.0 - deadlineMargin) / contentionFactor);
        
        // Find lowest power state
        for (int s = static_cast<int>(PowerState::LOW_POWER);
             s <= static_cast<int>(PowerState::TURBO); s++) {
            if (profile.executionCycles[s] <= effectiveDeadline) {
                return static_cast<PowerState>(s);
            }
        }
        
        return PowerState::TURBO;
    }
    
    /**
     * Race-to-Idle: Complete as fast as possible, then sleep
     * Good when idle power is low and wakeup is cheap
     */
    PowerState decideRaceToIdle(const TaskEnergyProfile& profile,
                                 uint64_t currentTime) {
        // Safety-critical tasks get TURBO
        if (profile.criticalityLevel >= 2) {
            return PowerState::TURBO;
        }
        
        // High accelerator utilization benefits from TURBO
        if (profile.accelUtilization > 0.5) {
            return PowerState::TURBO;
        }
        
        return PowerState::NOMINAL;
    }
    
    /**
     * Pace-to-Idle: Spread execution to avoid power spikes
     * Good for thermal management and peak power limits
     */
    PowerState decidePaceToIdle(const TaskEnergyProfile& profile,
                                 uint64_t currentTime) {
        uint64_t slack = 0;
        if (profile.deadline > currentTime) {
            slack = profile.deadline - currentTime;
        }
        
        // Find power state that uses most of the slack
        for (int s = static_cast<int>(PowerState::LOW_POWER);
             s <= static_cast<int>(PowerState::TURBO); s++) {
            uint64_t execTime = profile.executionCycles[s];
            // Aim for 80-95% slack utilization
            if (execTime >= slack * 0.8 && execTime <= slack * 0.95) {
                return static_cast<PowerState>(s);
            }
        }
        
        // Default: lowest power that meets deadline
        return decideMED(profile, currentTime, 0);
    }
    
    /**
     * Energy-Proportional: Scale power with available budget
     */
    PowerState decideEnergyProportional(const TaskEnergyProfile& profile,
                                         double remainingEnergy) {
        // Calculate energy budget per task (simple equal distribution)
        double taskBudget = remainingEnergy / std::max(1UL, taskProfiles.size());
        
        // Find power state within budget
        for (int s = static_cast<int>(PowerState::LOW_POWER);
             s <= static_cast<int>(PowerState::TURBO); s++) {
            if (profile.energyMicroJ[s] <= taskBudget) {
                return static_cast<PowerState>(s);
            }
        }
        
        return PowerState::LOW_POWER;  // Minimum power if budget tight
    }
    
    /**
     * Hybrid Critical: Performance for critical, save for others
     */
    PowerState decideHybridCritical(const TaskEnergyProfile& profile,
                                     uint64_t currentTime) {
        if (profile.criticalityLevel >= 2) {
            // Safety-critical: always performance
            return PowerState::TURBO;
        } else if (profile.criticalityLevel == 1) {
            // Mission-critical: nominal
            return PowerState::NOMINAL;
        } else {
            // Best-effort: power saving
            return decideMED(profile, currentTime, 0);
        }
    }
    
    //==========================================================================
    // Helper Methods
    //==========================================================================
    
    void computeOptimalPowerState(TaskEnergyProfile& profile) {
        profile.optimalEnergy = profile.energyMicroJ[static_cast<int>(PowerState::TURBO)];
        profile.optimalPowerState = PowerState::TURBO;
        profile.meetsDeadline = false;
        
        // Find minimum energy state that meets deadline
        for (int s = static_cast<int>(PowerState::LOW_POWER);
             s <= static_cast<int>(PowerState::TURBO); s++) {
            if (profile.executionCycles[s] <= profile.deadline) {
                profile.meetsDeadline = true;
                if (profile.energyMicroJ[s] < profile.optimalEnergy) {
                    profile.optimalEnergy = profile.energyMicroJ[s];
                    profile.optimalPowerState = static_cast<PowerState>(s);
                    profile.optimalExecTime = profile.executionCycles[s];
                }
            }
        }
    }
    
    static const char* getPolicyName(PowerAwarePolicy policy) {
        switch (policy) {
            case PowerAwarePolicy::MED: return "MED";
            case PowerAwarePolicy::MED_CONTENTION: return "MED+Contention";
            case PowerAwarePolicy::RACE_TO_IDLE: return "Race-to-Idle";
            case PowerAwarePolicy::PACE_TO_IDLE: return "Pace-to-Idle";
            case PowerAwarePolicy::ENERGY_PROPORTIONAL: return "Energy-Proportional";
            case PowerAwarePolicy::HYBRID_CRITICAL: return "Hybrid-Critical";
            default: return "UNKNOWN";
        }
    }
};

} // namespace gem5

#endif // __CPU_POWER_MANAGER_POWER_AWARE_SCHEDULER_HH__

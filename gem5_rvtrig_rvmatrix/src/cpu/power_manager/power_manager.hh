/*
 * Power Manager for Spacecraft Accelerators
 * 
 * Manages power states of all accelerators with:
 * - DVFS (Dynamic Voltage/Frequency Scaling)
 * - Power gating for unused accelerators
 * - Energy budget tracking
 * - Thermal awareness (future extension)
 * 
 * Part of PhD Research: Power-Aware Scheduling for Heterogeneous Accelerators
 */

#ifndef __CPU_POWER_MANAGER_POWER_MANAGER_HH__
#define __CPU_POWER_MANAGER_POWER_MANAGER_HH__

#include "spacecraft_accelerators.hh"
#include <array>
#include <mutex>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace gem5 {

//==============================================================================
// Power Management Policies
//==============================================================================

enum class PowerPolicy : uint8_t {
    PERFORMANCE,        // Always TURBO, maximum speed
    BALANCED,           // NOMINAL when active, IDLE when unused
    POWER_SAVER,        // LOW_POWER default, NOMINAL for critical
    AGGRESSIVE_SAVING,  // Sleep aggressively, wake on demand
    MISSION_ADAPTIVE,   // Adapt based on mission phase
    NUM_POLICIES
};

//==============================================================================
// Energy Budget Management
//==============================================================================

struct EnergyBudget {
    double totalBudgetMicroJ;      // Mission phase energy budget
    double consumedMicroJ;         // Energy consumed so far
    double reservedMicroJ;         // Reserved for critical tasks
    double warningThreshold;       // Fraction (0.8 = 80% warning)
    double criticalThreshold;      // Fraction (0.95 = 95% critical)
    
    EnergyBudget()
        : totalBudgetMicroJ(1000000.0)  // 1 Joule default
        , consumedMicroJ(0.0)
        , reservedMicroJ(100000.0)      // 100mJ reserved
        , warningThreshold(0.8)
        , criticalThreshold(0.95)
    {}
    
    double remainingBudget() const {
        return totalBudgetMicroJ - consumedMicroJ;
    }
    
    double usageFraction() const {
        return consumedMicroJ / totalBudgetMicroJ;
    }
    
    bool isWarning() const {
        return usageFraction() >= warningThreshold;
    }
    
    bool isCritical() const {
        return usageFraction() >= criticalThreshold;
    }
    
    bool canAfford(double energyMicroJ) const {
        return (consumedMicroJ + energyMicroJ + reservedMicroJ) <= totalBudgetMicroJ;
    }
};

//==============================================================================
// Power Manager Class
//==============================================================================

class PowerManager {
private:
    // Accelerator states
    std::array<AcceleratorState, static_cast<size_t>(AcceleratorType::NUM_ACCELERATORS)> 
        acceleratorStates;
    
    // Global power management
    PowerPolicy currentPolicy;
    EnergyBudget energyBudget;
    
    // Timing
    uint64_t clockPeriodPs;  // Clock period in picoseconds
    uint64_t currentTick;
    
    // Thread safety
    mutable std::mutex powerMutex;
    
    // Statistics
    uint64_t totalPolicyChanges;
    uint64_t totalStateTransitions;
    double peakPowerMW;
    double avgPowerMW;
    uint64_t powerSamples;

public:
    PowerManager(uint64_t clockPeriodPs = 1000)  // 1GHz default
        : currentPolicy(PowerPolicy::BALANCED)
        , clockPeriodPs(clockPeriodPs)
        , currentTick(0)
        , totalPolicyChanges(0)
        , totalStateTransitions(0)
        , peakPowerMW(0.0)
        , avgPowerMW(0.0)
        , powerSamples(0)
    {
        // Initialize all accelerators
        for (size_t i = 0; i < static_cast<size_t>(AcceleratorType::NUM_ACCELERATORS); i++) {
            acceleratorStates[i].type = static_cast<AcceleratorType>(i);
            acceleratorStates[i].currentPowerState = PowerState::IDLE;
        }
    }
    
    //==========================================================================
    // Power State Management
    //==========================================================================
    
    /**
     * Request power state transition for an accelerator
     * Returns wakeup latency in cycles (0 if already at or above target state)
     */
    uint32_t requestPowerState(AcceleratorType type, PowerState targetState) {
        std::lock_guard<std::mutex> lock(powerMutex);
        
        auto& state = acceleratorStates[static_cast<size_t>(type)];
        PowerState currentState = state.currentPowerState;
        
        if (targetState == currentState) {
            return 0;  // No transition needed
        }
        
        // Calculate wakeup latency
        uint32_t wakeupCycles = getWakeupLatency(type, currentState, targetState);
        
        // Update state tracking
        uint64_t timeInPrevState = currentTick - state.lastStateChangeTime;
        state.timeInState[static_cast<int>(currentState)] += timeInPrevState;
        state.stateTransitions[static_cast<int>(targetState)]++;
        state.lastStateChangeTime = currentTick;
        
        // Transition
        state.currentPowerState = targetState;
        totalStateTransitions++;
        
        return wakeupCycles;
    }
    
    /**
     * Get current power state of an accelerator
     */
    PowerState getPowerState(AcceleratorType type) const {
        std::lock_guard<std::mutex> lock(powerMutex);
        return acceleratorStates[static_cast<size_t>(type)].currentPowerState;
    }
    
    /**
     * Ensure accelerator is at minimum required power state
     * Returns additional wakeup latency if transition was needed
     */
    uint32_t ensureMinimumPowerState(AcceleratorType type, PowerState minState) {
        PowerState current = getPowerState(type);
        if (current >= minState) {
            return 0;
        }
        return requestPowerState(type, minState);
    }
    
    //==========================================================================
    // Accelerator Access with Power Awareness
    //==========================================================================
    
    /**
     * Try to acquire an accelerator, handling power state transitions
     * Returns total delay (wakeup + contention + latency) in cycles
     */
    uint32_t acquireAccelerator(AcceleratorType type, int cpuId) {
        std::lock_guard<std::mutex> lock(powerMutex);
        
        auto& state = acceleratorStates[static_cast<size_t>(type)];
        const auto& spec = getAccelSpec(type);
        
        // Determine target power state based on policy
        PowerState targetState = getTargetPowerState(type);
        
        // Calculate wakeup latency if needed
        uint32_t wakeupCycles = 0;
        if (state.currentPowerState < targetState) {
            wakeupCycles = getWakeupLatency(type, state.currentPowerState, targetState);
            state.currentPowerState = targetState;
            totalStateTransitions++;
        }
        
        // Calculate contention delay
        uint32_t contentionCycles = 0;
        if (state.busy && state.busyUntilTick > currentTick) {
            uint64_t waitTicks = state.busyUntilTick - currentTick;
            contentionCycles = static_cast<uint32_t>(waitTicks / (clockPeriodPs / 1000));
            state.queuedRequests++;
            state.totalWaitCycles += contentionCycles;
        }
        
        // Get operation latency at current power state
        uint32_t operationLatency = getScaledLatency(type, state.currentPowerState);
        
        // Update state
        uint32_t totalCycles = wakeupCycles + contentionCycles + operationLatency;
        state.busyUntilTick = currentTick + (totalCycles * clockPeriodPs / 1000);
        state.busy = true;
        state.ownerCpuId = cpuId;
        state.totalRequests++;
        state.totalActiveCycles += operationLatency;
        
        // Track energy consumption
        double opDurationNs = operationLatency * clockPeriodPs / 1000.0;
        double energyMicroJ = getInstantPower(type, state.currentPowerState) * opDurationNs / 1000.0;
        state.totalEnergyMicroJ += energyMicroJ;
        energyBudget.consumedMicroJ += energyMicroJ;
        
        // Update power statistics
        updatePowerStats();
        
        return totalCycles;
    }
    
    /**
     * Release an accelerator after use
     */
    void releaseAccelerator(AcceleratorType type, int cpuId) {
        std::lock_guard<std::mutex> lock(powerMutex);
        
        auto& state = acceleratorStates[static_cast<size_t>(type)];
        
        if (state.ownerCpuId == cpuId) {
            state.busy = false;
            state.ownerCpuId = -1;
            
            // Consider powering down based on policy
            if (currentPolicy == PowerPolicy::AGGRESSIVE_SAVING) {
                // Immediately go to SLEEP
                state.currentPowerState = PowerState::SLEEP;
            } else if (currentPolicy == PowerPolicy::POWER_SAVER) {
                // Go to LOW_POWER
                state.currentPowerState = PowerState::LOW_POWER;
            }
            // BALANCED and PERFORMANCE stay at current state
        }
    }
    
    //==========================================================================
    // Policy Management
    //==========================================================================
    
    void setPolicy(PowerPolicy policy) {
        std::lock_guard<std::mutex> lock(powerMutex);
        if (currentPolicy != policy) {
            currentPolicy = policy;
            totalPolicyChanges++;
            
            // Adjust all accelerator states based on new policy
            applyPolicyToAllAccelerators();
        }
    }
    
    PowerPolicy getPolicy() const {
        return currentPolicy;
    }
    
    void setEnergyBudget(double totalMicroJ, double reservedMicroJ = 0.0) {
        std::lock_guard<std::mutex> lock(powerMutex);
        energyBudget.totalBudgetMicroJ = totalMicroJ;
        energyBudget.reservedMicroJ = reservedMicroJ;
    }
    
    //==========================================================================
    // Timing
    //==========================================================================
    
    void updateTick(uint64_t tick) {
        currentTick = tick;
    }
    
    //==========================================================================
    // Statistics
    //==========================================================================
    
    double getTotalEnergyMicroJ() const {
        return energyBudget.consumedMicroJ;
    }
    
    double getCurrentPowerMW() const {
        std::lock_guard<std::mutex> lock(powerMutex);
        double totalPower = 0.0;
        for (size_t i = 0; i < static_cast<size_t>(AcceleratorType::NUM_ACCELERATORS); i++) {
            totalPower += getInstantPower(
                static_cast<AcceleratorType>(i),
                acceleratorStates[i].currentPowerState
            );
        }
        return totalPower;
    }
    
    const AcceleratorState& getAcceleratorState(AcceleratorType type) const {
        return acceleratorStates[static_cast<size_t>(type)];
    }
    
    void printStatistics() const {
        std::lock_guard<std::mutex> lock(powerMutex);
        
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║           POWER MANAGER STATISTICS                               ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        
        std::cout << "║ Policy: " << std::setw(20) << getPolicyName(currentPolicy) 
                  << "    Changes: " << std::setw(8) << totalPolicyChanges << "       ║\n";
        std::cout << "║ State Transitions: " << std::setw(10) << totalStateTransitions 
                  << "                                 ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ ENERGY BUDGET                                                    ║\n";
        std::cout << "║   Total:    " << std::setw(12) << std::fixed << std::setprecision(2) 
                  << energyBudget.totalBudgetMicroJ << " µJ                              ║\n";
        std::cout << "║   Consumed: " << std::setw(12) << energyBudget.consumedMicroJ 
                  << " µJ (" << std::setw(5) << std::setprecision(1) 
                  << (energyBudget.usageFraction() * 100) << "%)                   ║\n";
        std::cout << "║   Status:   " << std::setw(12) 
                  << (energyBudget.isCritical() ? "CRITICAL" : 
                      (energyBudget.isWarning() ? "WARNING" : "OK"))
                  << "                                    ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║ ACCELERATOR STATISTICS                                           ║\n";
        std::cout << "╠──────────────┬─────────┬────────┬──────────┬─────────────────────╣\n";
        std::cout << "║ Accelerator  │ State   │Requests│ Queued   │ Energy (µJ)         ║\n";
        std::cout << "╠──────────────┼─────────┼────────┼──────────┼─────────────────────╣\n";
        
        for (size_t i = 0; i < static_cast<size_t>(AcceleratorType::NUM_ACCELERATORS); i++) {
            const auto& state = acceleratorStates[i];
            std::cout << "║ " << std::setw(12) << getAccelName(static_cast<AcceleratorType>(i))
                      << " │ " << std::setw(7) << getPowerStateName(state.currentPowerState)
                      << " │ " << std::setw(6) << state.totalRequests
                      << " │ " << std::setw(8) << state.queuedRequests
                      << " │ " << std::setw(17) << std::setprecision(2) << state.totalEnergyMicroJ
                      << " ║\n";
        }
        
        std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
    }

private:
    PowerState getTargetPowerState(AcceleratorType type) const {
        const auto& spec = getAccelSpec(type);
        
        switch (currentPolicy) {
            case PowerPolicy::PERFORMANCE:
                return PowerState::TURBO;
                
            case PowerPolicy::BALANCED:
                return PowerState::NOMINAL;
                
            case PowerPolicy::POWER_SAVER:
                // Safety-critical accelerators stay at NOMINAL
                if (spec.criticalityLevel >= 2) {
                    return PowerState::NOMINAL;
                }
                return PowerState::LOW_POWER;
                
            case PowerPolicy::AGGRESSIVE_SAVING:
                // Only safety-critical get LOW_POWER, others SLEEP
                if (spec.criticalityLevel >= 2) {
                    return PowerState::LOW_POWER;
                }
                return PowerState::IDLE;
                
            case PowerPolicy::MISSION_ADAPTIVE:
                // Will be overridden by MissionAdapter
                return PowerState::NOMINAL;
                
            default:
                return PowerState::NOMINAL;
        }
    }
    
    void applyPolicyToAllAccelerators() {
        for (size_t i = 0; i < static_cast<size_t>(AcceleratorType::NUM_ACCELERATORS); i++) {
            auto type = static_cast<AcceleratorType>(i);
            PowerState target = getTargetPowerState(type);
            acceleratorStates[i].currentPowerState = target;
        }
    }
    
    void updatePowerStats() {
        double currentPower = 0.0;
        for (size_t i = 0; i < static_cast<size_t>(AcceleratorType::NUM_ACCELERATORS); i++) {
            currentPower += getInstantPower(
                static_cast<AcceleratorType>(i),
                acceleratorStates[i].currentPowerState
            );
        }
        
        if (currentPower > peakPowerMW) {
            peakPowerMW = currentPower;
        }
        
        // Running average
        avgPowerMW = (avgPowerMW * powerSamples + currentPower) / (powerSamples + 1);
        powerSamples++;
    }
    
    static const char* getPolicyName(PowerPolicy policy) {
        switch (policy) {
            case PowerPolicy::PERFORMANCE: return "PERFORMANCE";
            case PowerPolicy::BALANCED: return "BALANCED";
            case PowerPolicy::POWER_SAVER: return "POWER_SAVER";
            case PowerPolicy::AGGRESSIVE_SAVING: return "AGGRESSIVE_SAVING";
            case PowerPolicy::MISSION_ADAPTIVE: return "MISSION_ADAPTIVE";
            default: return "UNKNOWN";
        }
    }
};

} // namespace gem5

#endif // __CPU_POWER_MANAGER_POWER_MANAGER_HH__

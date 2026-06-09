/*
 * NOVA Processor - Mission Phase Manager Implementation
 * PhD Research: Futuristic Spacecraft Processor
 */

#include "spacecraft/mission_phase_manager.hh"
#include "base/trace.hh"
#include "debug/MissionPhase.hh"

namespace gem5
{

namespace spacecraft
{

MissionPhaseManager::MissionPhaseManager(const Params &p)
    : ClockedObject(p),
      currentPhase(MissionPhase::NORMAL_OPS),
      previousPhase(MissionPhase::NORMAL_OPS),
      phaseStartTime(0),
      missionStartTime(0),
      currentPowerBudget(100.0),
      inEmergency(false),
      scheduler(nullptr),
      accelManager(nullptr),
      transitionEvent([this]{ processScheduledTransition(); }, name() + ".transitionEvent")
{
    initializePhaseConfigs();
    
    // Set initial phase from parameter
    std::string initialPhaseStr = p.initial_phase;
    if (initialPhaseStr == "LAUNCH") {
        currentPhase = MissionPhase::LAUNCH;
    } else if (initialPhaseStr == "ORBIT_INSERTION") {
        currentPhase = MissionPhase::ORBIT_INSERTION;
    } else if (initialPhaseStr == "LANDING") {
        currentPhase = MissionPhase::LANDING;
    } else if (initialPhaseStr == "SURFACE_OPS") {
        currentPhase = MissionPhase::SURFACE_OPS;
    } else if (initialPhaseStr == "SAFE_MODE") {
        currentPhase = MissionPhase::SAFE_MODE;
    } else {
        currentPhase = MissionPhase::NORMAL_OPS;
    }
    
    currentPowerBudget = phaseConfigs[currentPhase].powerBudgetWatts;
    
    DPRINTF(MissionPhase, "Mission Phase Manager initialized, phase=%s, power=%.1fW\n",
            phaseToString(currentPhase).c_str(), currentPowerBudget);
}

MissionPhaseManager::~MissionPhaseManager()
{
}

void MissionPhaseManager::initializePhaseConfigs()
{
    // LAUNCH phase - critical, full power
    phaseConfigs[MissionPhase::LAUNCH] = {
        MissionPhase::LAUNCH,
        100.0,  // Full power
        SchedPolicy::FIXED_PRIORITY_PREEMPTIVE,
        {0, 8, 9},  // GNC, Kalman, Attitude Control
        false,  // VPU disabled
        false,  // NPU disabled
        1.0,    // Full CPU speed
        "Launch and ascent phase"
    };
    
    // ORBIT_INSERTION - critical, full power
    phaseConfigs[MissionPhase::ORBIT_INSERTION] = {
        MissionPhase::ORBIT_INSERTION,
        100.0,
        SchedPolicy::FIXED_PRIORITY_PREEMPTIVE,
        {0, 1, 5, 8, 9},  // GNC, Star Tracker, Orbit Prop, Kalman, Attitude
        false,
        false,
        1.0,
        "Orbit insertion maneuvers"
    };
    
    // NORMAL_OPS - mixed, moderate power
    phaseConfigs[MissionPhase::NORMAL_OPS] = {
        MissionPhase::NORMAL_OPS,
        50.0,
        SchedPolicy::MIXED_CRITICALITY_EDF,
        {0, 1, 5, 6, 7, 8, 9},  // Most tasks active
        true,   // VPU enabled for star tracking
        true,   // NPU enabled for science
        0.8,    // Moderate CPU speed
        "Normal orbital operations"
    };
    
    // LANDING - critical, full power, all accelerators
    phaseConfigs[MissionPhase::LANDING] = {
        MissionPhase::LANDING,
        80.0,
        SchedPolicy::FIXED_PRIORITY_PREEMPTIVE,
        {0, 2, 3, 4, 8, 9},  // GNC, Crater, Hazard, TRN, Kalman, Attitude
        true,   // VPU for terrain analysis
        true,   // NPU for crater/hazard detection
        1.0,
        "Powered descent and landing"
    };
    
    // SURFACE_OPS - power constrained
    phaseConfigs[MissionPhase::SURFACE_OPS] = {
        MissionPhase::SURFACE_OPS,
        20.0,  // Very limited power
        SchedPolicy::ENERGY_AWARE_EDF,
        {6, 7},  // Science and telemetry only
        true,   // VPU for imaging
        true,   // NPU for science processing
        0.5,    // Low CPU speed
        "Surface/rover operations"
    };
    
    // SAFE_MODE - minimal operations
    phaseConfigs[MissionPhase::SAFE_MODE] = {
        MissionPhase::SAFE_MODE,
        10.0,  // Minimal power
        SchedPolicy::MINIMAL_OPERATIONS,
        {7},   // Telemetry only
        false,
        false,
        0.3,   // Minimum CPU speed
        "Emergency safe mode"
    };
    
    // ECLIPSE - power constrained during shadow
    phaseConfigs[MissionPhase::ECLIPSE] = {
        MissionPhase::ECLIPSE,
        30.0,  // Battery-only power
        SchedPolicy::ENERGY_AWARE_EDF,
        {0, 7, 8, 9},  // Essential only
        false,
        false,
        0.5,
        "Eclipse period (battery power)"
    };
    
    // COMM_WINDOW - high priority for downlink
    phaseConfigs[MissionPhase::COMM_WINDOW] = {
        MissionPhase::COMM_WINDOW,
        60.0,
        SchedPolicy::FIXED_PRIORITY_PREEMPTIVE,
        {7},  // Focus on telemetry
        false,
        false,
        0.7,
        "Communication window"
    };
    
    // SCIENCE_OPS - focus on science processing
    phaseConfigs[MissionPhase::SCIENCE_OPS] = {
        MissionPhase::SCIENCE_OPS,
        40.0,
        SchedPolicy::MIXED_CRITICALITY_EDF,
        {1, 5, 6, 7},  // Science-focused
        true,
        true,
        0.6,
        "Science observation mode"
    };
    
    // STANDBY - low power standby
    phaseConfigs[MissionPhase::STANDBY] = {
        MissionPhase::STANDBY,
        5.0,
        SchedPolicy::MINIMAL_OPERATIONS,
        {},  // No tasks
        false,
        false,
        0.2,
        "Low-power standby"
    };
    
    // Initialize time tracking
    for (auto& [phase, config] : phaseConfigs) {
        timeInPhase[phase] = 0;
    }
}

std::string MissionPhaseManager::getPhaseString() const
{
    return phaseToString(currentPhase);
}

void MissionPhaseManager::transitionToPhase(MissionPhase newPhase, const std::string &reason)
{
    if (newPhase == currentPhase) {
        DPRINTF(MissionPhase, "Already in phase %s, ignoring transition\n",
                phaseToString(newPhase).c_str());
        return;
    }
    
    MissionPhase oldPhase = currentPhase;
    
    // Update time tracking
    timeInPhase[oldPhase] += curTick() - phaseStartTime;
    
    // Record transition
    PhaseTransition transition = {
        curTick(),
        oldPhase,
        newPhase,
        reason
    };
    transitionHistory.push_back(transition);
    
    DPRINTF(MissionPhase, "Phase transition: %s -> %s (reason: %s)\n",
            phaseToString(oldPhase).c_str(),
            phaseToString(newPhase).c_str(),
            reason.c_str());
    
    // Update state
    previousPhase = oldPhase;
    currentPhase = newPhase;
    phaseStartTime = curTick();
    
    // Apply new phase configuration
    const PhaseConfig &config = phaseConfigs[newPhase];
    applyPhaseConfig(config);
    
    // Notify callbacks
    notifyPhaseChange(oldPhase, newPhase);
}

void MissionPhaseManager::scheduleTransition(Tick time, MissionPhase phase, const std::string &reason)
{
    ScheduledTransition transition = {time, phase, reason};
    scheduledTransitions.push_back(transition);
    
    // Schedule event if this is the next transition
    if (!transitionEvent.scheduled() || time < transitionEvent.when()) {
        if (transitionEvent.scheduled()) {
            deschedule(transitionEvent);
        }
        schedule(transitionEvent, time);
    }
    
    DPRINTF(MissionPhase, "Scheduled transition to %s at tick %lu (reason: %s)\n",
            phaseToString(phase).c_str(), time, reason.c_str());
}

void MissionPhaseManager::handleEmergency(const std::string &reason)
{
    if (inEmergency) {
        DPRINTF(MissionPhase, "Already in emergency mode\n");
        return;
    }
    
    inEmergency = true;
    
    DPRINTF(MissionPhase, "EMERGENCY: %s - transitioning to SAFE_MODE\n", reason.c_str());
    
    // Immediate transition to safe mode
    transitionToPhase(MissionPhase::SAFE_MODE, "Emergency: " + reason);
}

void MissionPhaseManager::exitEmergency()
{
    if (!inEmergency) {
        return;
    }
    
    inEmergency = false;
    
    DPRINTF(MissionPhase, "Exiting emergency mode, returning to %s\n",
            phaseToString(previousPhase).c_str());
    
    // Return to previous phase
    transitionToPhase(previousPhase, "Emergency cleared");
}

float MissionPhaseManager::getCurrentPowerBudget() const
{
    return currentPowerBudget;
}

void MissionPhaseManager::setPowerBudget(float watts)
{
    currentPowerBudget = watts;
    DPRINTF(MissionPhase, "Power budget set to %.1fW\n", watts);
}

bool MissionPhaseManager::isPowerConstrained() const
{
    return currentPowerBudget < 30.0;  // Consider constrained below 30W
}

SchedPolicy MissionPhaseManager::getCurrentPolicy() const
{
    return phaseConfigs.at(currentPhase).policy;
}

const PhaseConfig& MissionPhaseManager::getPhaseConfig(MissionPhase phase) const
{
    return phaseConfigs.at(phase);
}

void MissionPhaseManager::setPhaseConfig(MissionPhase phase, const PhaseConfig &config)
{
    phaseConfigs[phase] = config;
}

void MissionPhaseManager::setScheduler(GlobalTaskScheduler *sched)
{
    scheduler = sched;
}

void MissionPhaseManager::setAcceleratorManager(AcceleratorResourceManager *mgr)
{
    accelManager = mgr;
}

Tick MissionPhaseManager::getTimeInPhase(MissionPhase phase) const
{
    Tick time = timeInPhase.at(phase);
    if (phase == currentPhase) {
        time += curTick() - phaseStartTime;
    }
    return time;
}

void MissionPhaseManager::registerPhaseChangeCallback(
    std::function<void(MissionPhase, MissionPhase)> callback)
{
    phaseChangeCallbacks.push_back(callback);
}

void MissionPhaseManager::applyPhaseConfig(const PhaseConfig &config)
{
    currentPowerBudget = config.powerBudgetWatts;
    
    DPRINTF(MissionPhase, "Applying phase config: power=%.1fW, policy=%s, CPU_scale=%.1f\n",
            config.powerBudgetWatts,
            policyToString(config.policy).c_str(),
            config.cpuFrequencyScale);
    
    // Note: Actual scheduler and accelerator manager configuration
    // would be done through their respective APIs
    // This is a placeholder for the integration
}

void MissionPhaseManager::notifyPhaseChange(MissionPhase oldPhase, MissionPhase newPhase)
{
    for (auto &callback : phaseChangeCallbacks) {
        callback(oldPhase, newPhase);
    }
}

void MissionPhaseManager::processScheduledTransition()
{
    if (scheduledTransitions.empty()) {
        return;
    }
    
    // Find transitions due now
    Tick now = curTick();
    for (auto it = scheduledTransitions.begin(); it != scheduledTransitions.end(); ) {
        if (it->time <= now) {
            transitionToPhase(it->phase, it->reason);
            it = scheduledTransitions.erase(it);
        } else {
            ++it;
        }
    }
    
    // Schedule next transition if any remain
    if (!scheduledTransitions.empty()) {
        Tick nextTime = scheduledTransitions[0].time;
        for (const auto &t : scheduledTransitions) {
            if (t.time < nextTime) {
                nextTime = t.time;
            }
        }
        schedule(transitionEvent, nextTime);
    }
}

std::string MissionPhaseManager::phaseToString(MissionPhase phase)
{
    switch (phase) {
        case MissionPhase::LAUNCH: return "LAUNCH";
        case MissionPhase::ORBIT_INSERTION: return "ORBIT_INSERTION";
        case MissionPhase::NORMAL_OPS: return "NORMAL_OPS";
        case MissionPhase::LANDING: return "LANDING";
        case MissionPhase::SURFACE_OPS: return "SURFACE_OPS";
        case MissionPhase::SAFE_MODE: return "SAFE_MODE";
        case MissionPhase::ECLIPSE: return "ECLIPSE";
        case MissionPhase::COMM_WINDOW: return "COMM_WINDOW";
        case MissionPhase::SCIENCE_OPS: return "SCIENCE_OPS";
        case MissionPhase::STANDBY: return "STANDBY";
        default: return "UNKNOWN";
    }
}

std::string MissionPhaseManager::policyToString(SchedPolicy policy)
{
    switch (policy) {
        case SchedPolicy::FIXED_PRIORITY_PREEMPTIVE: return "FIXED_PRIORITY";
        case SchedPolicy::MIXED_CRITICALITY_EDF: return "MC_EDF";
        case SchedPolicy::ENERGY_AWARE_EDF: return "ENERGY_EDF";
        case SchedPolicy::MINIMAL_OPERATIONS: return "MINIMAL";
        case SchedPolicy::ROUND_ROBIN: return "ROUND_ROBIN";
        default: return "UNKNOWN";
    }
}

} // namespace spacecraft
} // namespace gem5





/*
 * NOVA Processor - Mission Phase Manager
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * Manages spacecraft mission phases and coordinates system behavior:
 * - Phase transitions (LAUNCH, ORBIT, LANDING, SURFACE, SAFE_MODE)
 * - Power budget management per phase
 * - Scheduling policy selection per phase
 * - Emergency handling
 */

#ifndef __SPACECRAFT_MISSION_PHASE_MANAGER_HH__
#define __SPACECRAFT_MISSION_PHASE_MANAGER_HH__

#include "params/MissionPhaseManager.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include <map>
#include <vector>
#include <set>
#include <string>
#include <functional>

namespace gem5
{

namespace spacecraft
{

// Forward declarations
class GlobalTaskScheduler;
class AcceleratorResourceManager;

/**
 * Mission Phases for ISRO spacecraft operations
 */
enum class MissionPhase {
    LAUNCH,             // Launch and ascent (0-30 min)
    ORBIT_INSERTION,    // Orbit insertion maneuvers
    NORMAL_OPS,         // Normal orbital operations
    LANDING,            // Descent and landing
    SURFACE_OPS,        // Surface/rover operations
    SAFE_MODE,          // Emergency/safe mode
    ECLIPSE,            // Eclipse period (power constrained)
    COMM_WINDOW,        // Communication window (high priority downlink)
    SCIENCE_OPS,        // Science observation mode
    STANDBY             // Low-power standby
};

/**
 * Scheduling policies available
 */
enum class SchedPolicy {
    FIXED_PRIORITY_PREEMPTIVE,  // Rate Monotonic / Deadline Monotonic
    MIXED_CRITICALITY_EDF,      // EDF with criticality awareness
    ENERGY_AWARE_EDF,           // EDF with power constraints
    MINIMAL_OPERATIONS,         // Safe mode - essentials only
    ROUND_ROBIN                 // Fair sharing
};

/**
 * Phase configuration
 */
struct PhaseConfig {
    MissionPhase phase;
    float powerBudgetWatts;
    SchedPolicy policy;
    std::set<int> activeTaskIds;
    bool enableVPU;
    bool enableNPU;
    float cpuFrequencyScale;
    std::string description;
};

/**
 * Phase transition event
 */
struct PhaseTransition {
    Tick time;
    MissionPhase fromPhase;
    MissionPhase toPhase;
    std::string reason;
};

/**
 * Mission Phase Manager - Coordinates system behavior based on mission state
 */
class MissionPhaseManager : public ClockedObject
{
  public:
    PARAMS(MissionPhaseManager);
    MissionPhaseManager(const Params &p);
    ~MissionPhaseManager();
    
    // Phase management
    MissionPhase getCurrentPhase() const { return currentPhase; }
    std::string getPhaseString() const;
    
    // Transition to new phase
    void transitionToPhase(MissionPhase newPhase, const std::string &reason = "");
    
    // Schedule future phase transition
    void scheduleTransition(Tick time, MissionPhase phase, const std::string &reason);
    
    // Emergency handling
    void handleEmergency(const std::string &reason);
    void exitEmergency();
    
    // Power budget
    float getCurrentPowerBudget() const;
    void setPowerBudget(float watts);
    bool isPowerConstrained() const;
    
    // Policy access
    SchedPolicy getCurrentPolicy() const;
    
    // Phase configuration
    const PhaseConfig& getPhaseConfig(MissionPhase phase) const;
    void setPhaseConfig(MissionPhase phase, const PhaseConfig &config);
    
    // Connect to other managers
    void setScheduler(GlobalTaskScheduler *scheduler);
    void setAcceleratorManager(AcceleratorResourceManager *accelMgr);
    
    // Mission timeline
    Tick getMissionTime() const { return curTick() - missionStartTime; }
    void setMissionStartTime(Tick time) { missionStartTime = time; }
    
    // Statistics
    uint64_t getPhaseTransitionCount() const { return transitionHistory.size(); }
    const std::vector<PhaseTransition>& getTransitionHistory() const { return transitionHistory; }
    Tick getTimeInPhase(MissionPhase phase) const;
    
    // Callback registration
    void registerPhaseChangeCallback(std::function<void(MissionPhase, MissionPhase)> callback);
    
  private:
    // Current state
    MissionPhase currentPhase;
    MissionPhase previousPhase;
    Tick phaseStartTime;
    Tick missionStartTime;
    
    // Phase configurations
    std::map<MissionPhase, PhaseConfig> phaseConfigs;
    
    // Current power budget (may differ from phase default)
    float currentPowerBudget;
    bool inEmergency;
    
    // Connected managers
    GlobalTaskScheduler *scheduler;
    AcceleratorResourceManager *accelManager;
    
    // Transition history
    std::vector<PhaseTransition> transitionHistory;
    
    // Time tracking per phase
    std::map<MissionPhase, Tick> timeInPhase;
    
    // Phase change callbacks
    std::vector<std::function<void(MissionPhase, MissionPhase)>> phaseChangeCallbacks;
    
    // Scheduled transitions
    struct ScheduledTransition {
        Tick time;
        MissionPhase phase;
        std::string reason;
    };
    std::vector<ScheduledTransition> scheduledTransitions;
    
    // Events
    EventFunctionWrapper transitionEvent;
    
    // Internal methods
    void initializePhaseConfigs();
    void applyPhaseConfig(const PhaseConfig &config);
    void notifyPhaseChange(MissionPhase oldPhase, MissionPhase newPhase);
    void processScheduledTransition();
    
    // Convert enums to strings
    static std::string phaseToString(MissionPhase phase);
    static std::string policyToString(SchedPolicy policy);
};

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_MISSION_PHASE_MANAGER_HH__





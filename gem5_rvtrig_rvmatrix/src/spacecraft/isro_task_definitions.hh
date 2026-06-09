/*
 * NOVA Processor - ISRO Task Definitions
 * PhD Research: Futuristic Spacecraft Processor
 * 
 * Defines all spacecraft tasks for ISRO missions:
 * - GNC (Guidance, Navigation, Control)
 * - Star Tracker Processing
 * - Crater Detection and Matching
 * - Hazard Avoidance
 * - Terrain Relative Navigation
 * - Orbit Propagation
 * - Science Image Processing
 * - Telemetry and Housekeeping
 * - Kalman Filter Update
 * - Attitude Control
 */

#ifndef __SPACECRAFT_ISRO_TASK_DEFINITIONS_HH__
#define __SPACECRAFT_ISRO_TASK_DEFINITIONS_HH__

#include "params/ISROTaskDefinitions.hh"
#include "sim/sim_object.hh"
#include "spacecraft/global_task_scheduler.hh"
#include "spacecraft/mission_phase_manager.hh"

#include <vector>
#include <string>

namespace gem5
{

namespace spacecraft
{

// Tick conversion constants (assuming 1 tick = 1 ps)
const Tick TICK_PER_US = 1000000;
const Tick TICK_PER_MS = 1000000000;
const Tick TICK_PER_S = 1000000000000;

/**
 * ISRO Task IDs
 */
enum class ISROTaskId {
    GNC_LOOP = 0,
    STAR_TRACKER = 1,
    CRATER_DETECTION = 2,
    HAZARD_AVOIDANCE = 3,
    TERRAIN_REL_NAV = 4,
    ORBIT_PROPAGATION = 5,
    SCIENCE_IMAGE = 6,
    TELEMETRY = 7,
    KALMAN_FILTER = 8,
    ATTITUDE_CONTROL = 9
};

/**
 * Factory class for creating ISRO spacecraft tasks
 */
class ISROTaskDefinitions : public SimObject
{
  public:
    PARAMS(ISROTaskDefinitions);
    ISROTaskDefinitions(const Params &p);
    
    // Create individual tasks
    static SpacecraftTask createGNCTask();
    static SpacecraftTask createStarTrackerTask();
    static SpacecraftTask createCraterDetectionTask();
    static SpacecraftTask createHazardAvoidanceTask();
    static SpacecraftTask createTerrainRelNavTask();
    static SpacecraftTask createOrbitPropagationTask();
    static SpacecraftTask createScienceImageTask();
    static SpacecraftTask createTelemetryTask();
    static SpacecraftTask createKalmanFilterTask();
    static SpacecraftTask createAttitudeControlTask();
    
    // Get all tasks for a mission phase
    static std::vector<SpacecraftTask> getTasksForPhase(MissionPhase phase);
    
    // Get all defined tasks
    static std::vector<SpacecraftTask> getAllTasks();
    
    // Get task by ID
    static SpacecraftTask getTaskById(ISROTaskId id);
    
    // Register all tasks with a scheduler
    static void registerAllTasks(GlobalTaskScheduler *scheduler);
    static void registerTasksForPhase(GlobalTaskScheduler *scheduler, MissionPhase phase);
    
    // Task timing presets (based on mission requirements)
    struct TaskTimingPreset {
        std::string name;
        Tick period;
        Tick deadline;
        Tick wcet;
    };
    
    static TaskTimingPreset getTimingPreset(ISROTaskId id, const std::string &profile);
};

/**
 * Detailed task characteristics for analysis
 */
struct TaskCharacteristics {
    ISROTaskId id;
    std::string name;
    std::string description;
    
    // Computational characteristics
    uint64_t estimatedInstructions;
    uint64_t estimatedMemoryAccesses;
    float computeIntensity;  // Compute/Memory ratio
    
    // Accelerator usage
    bool usesTrigAccel;
    bool usesMatAccel;
    bool usesVPU;
    bool usesNPU;
    
    // Data characteristics
    uint64_t inputDataSizeBytes;
    uint64_t outputDataSizeBytes;
    
    // Real-time requirements
    bool isHardRealTime;
    float jitterTolerancePercent;
};

/**
 * Get detailed characteristics for a task
 */
TaskCharacteristics getTaskCharacteristics(ISROTaskId id);

} // namespace spacecraft
} // namespace gem5

#endif // __SPACECRAFT_ISRO_TASK_DEFINITIONS_HH__




